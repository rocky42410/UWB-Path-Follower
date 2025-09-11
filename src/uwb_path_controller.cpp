// File: src/uwb_path_controller.cpp
#include <cmath>
#include <thread>
#include <chrono>
#include <atomic>
#include <mutex>
#include <fstream>
#include <iomanip>
#include <csignal>
#include <memory>
#include <iostream>
#include <algorithm>
#include <vector>
#include <deque>
#include <optional>
#include <cstring>
#include <ifaddrs.h>

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/idl/go2/UwbState_.hpp>
#include <unitree/robot/go2/sport/sport_client.hpp>

// Include our headers
#include "uwb_path_follower/types.hpp"
#include "uwb_path_follower/config.hpp"
#include "uwb_path_follower/uwb_receiver.hpp"
#include "uwb_path_follower/ts_ring_buffer.hpp"
#include "uwb_path_follower/math2d.hpp"
#include "uwb_path_follower/all_components.hpp"

using namespace unitree::robot;
using namespace unitree::common;
using namespace uwb_path;

std::atomic<bool> g_running{true};
void signalHandler(int sig) {
  if (sig == SIGINT) {
    std::cout << "\n[Signal] SIGINT\n";
    g_running = false;
  }
}

static bool interfaceExists(const std::string& iface) {
  struct ifaddrs* ifaddr = nullptr; 
  bool found = false;
  std::vector<std::string> list;
  
  if (getifaddrs(&ifaddr) == 0) {
    for (auto* p = ifaddr; p; p = p->ifa_next) {
      if (!p->ifa_name) continue;
      std::string name = p->ifa_name;
      if (name == iface) found = true;
      list.push_back(name);
    }
    freeifaddrs(ifaddr);
  }
  
  if (!found && !list.empty()) {
    std::sort(list.begin(), list.end());
    list.erase(std::unique(list.begin(), list.end()), list.end());
    std::cerr << "Available interfaces: ";
    for (size_t i = 0; i < list.size(); ++i) { 
      if (i) std::cerr << ", "; 
      std::cerr << list[i];
    }
    std::cerr << "\n";
  }
  return found;
}

class UWBPathController {
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  Config cfg;
  unitree::robot::go2::SportClient sport_client;
  std::shared_ptr<ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>> sport_sub;

  std::unique_ptr<UWBReceiver> uwb_rx;
  StateEstimator estimator;
  UWBConverter converter;
  DataLogger logger;
  LowPassFilter lp_vy, lp_wz;

  std::atomic<bool>& running;
  std::mutex mtx;

  // IMU and sensor data
  IMUData imu_latest{};
  std::array<double,4> foot_forces{};
  double body_height = 0.28;
  int gait_type = 0;

  // Control state
  Twist2D last_cmd{};
  std::string mode;
  std::string iface;

  // Enhanced control state
  enum ControlState { TURN = 0, FWD = 1 };
  ControlState state_ = TURN;
  
  double psi_fused_ = 0.0;        // fused heading (rad)
  double err_filt_ = 0.0;          // filtered heading error
  
  TimePoint last_loop_{};
  TimePoint last_state_change_{};
  
  // Stall detection
  std::deque<double> error_history_;
  
  // Current goal (simple waypoint system)
  int waypoint_index_ = 0;
  std::vector<Vec2> waypoints_;

public:
  UWBPathController(const std::string& net_if,
                    const std::string& cfg_file,
                    const std::string& run_mode,
                    std::atomic<bool>& run_flag)
  : cfg(),
    estimator(cfg),
    converter(cfg),
    logger("uwb_path_log.csv"),
    lp_vy(cfg.LP_TAU),
    lp_wz(cfg.LP_TAU),
    running(run_flag),
    mode(run_mode),
    iface(net_if) {

    if (!cfg_file.empty()) {
      cfg.loadFromFile(cfg_file);
    }

    // Initialize sport client
    sport_client.SetTimeout(10.0f);
    sport_client.Init();
    std::cout << "[Init] Sport client OK\n";

    // Subscribe to sport state
    sport_sub = std::make_shared<ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>>("rt/sportmodestate");
    sport_sub->InitChannel([this](const void* msg){ 
      if (msg) handleSportState(msg); 
    });
    std::cout << "[Init] Sport state subscriber OK\n";

    // Initialize UWB receiver
    uwb_rx = std::make_unique<UWBReceiver>(cfg.UWB_R_GATE, cfg.UWB_BETA_GATE, 
                                           cfg.UWB_ALPHA_GATE, cfg.UWB_GAMMA_GATE);
    uwb_rx->updateSphericalGates(cfg);
    std::cout << "[Init] UWB receiver OK\n";

    // Setup waypoints (simple out-and-back path)
    waypoints_.push_back({0.0, 0.0});
    waypoints_.push_back({cfg.PATH_LENGTH, 0.0});
    waypoints_.push_back({0.0, 0.0});

    // Initialize timing
    last_loop_ = Clock::now();
    last_state_change_ = Clock::now();

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    if (uwb_rx->hasData()) {
      std::cout << "[Init] UWB data available | Mode=" << uwb_rx->getModeName() 
                << " | " << uwb_rx->getErrorDescription() << "\n";
    } else {
      std::cout << "[Init] No UWB yet; check base station\n";
    }
    std::cout << "[Init] Mode: " << mode << "\n";
  }

  ~UWBPathController() {
    std::cout << "[Shutdown] Controller stopping...\n";
    running = false;
    try { 
      sport_client.Move(0, 0, 0); 
    } catch(...) {}
    
    if (sport_sub) sport_sub.reset();
    if (uwb_rx) { 
      uwb_rx->shutdown(); 
      uwb_rx.reset(); 
    }
    std::cout << "[Shutdown] Done\n";
  }

  void handleSportState(const void* m) {
    if (!running) return;
    const auto& msg = *(const unitree_go::msg::dds_::SportModeState_*)m;
    std::lock_guard<std::mutex> lock(mtx);
    
    imu_latest.gyro_z = msg.imu_state().gyroscope()[2];
    imu_latest.roll   = msg.imu_state().rpy()[0];
    imu_latest.pitch  = msg.imu_state().rpy()[1];
    
    for (int i = 0; i < 4; i++) {
      imu_latest.quaternion[i] = msg.imu_state().quaternion()[i];
      foot_forces[i] = msg.foot_force()[i];
    }
    
    body_height = msg.body_height();
    gait_type = msg.gait_type();
  }

  bool checkZUPT() {
    double total = 0; 
    for (auto f : foot_forces) total += f;
    
    bool low_motion = (std::fabs(last_cmd.vx) < cfg.ZUPT_VEL_THRESH) &&
                      (std::fabs(last_cmd.vy) < cfg.ZUPT_VEL_THRESH) &&
                      (std::fabs(imu_latest.gyro_z) < cfg.ZUPT_GYRO_THRESH);
    
    return (total > cfg.ZUPT_FOOT_THRESH) && low_motion;
  }

  Vec2 getCurrentGoal() {
    if (waypoint_index_ >= static_cast<int>(waypoints_.size())) {
      waypoint_index_ = 0;
    }
    return waypoints_[waypoint_index_];
  }

  Vec2 getCurrentPosition() {
    // Get best estimate of current position
    // For now, use UWB position if available, otherwise dead reckon
    auto now = Clock::now();
    auto pos_opt = uwb_rx->positionAt(now);
    
    if (pos_opt) {
      return *pos_opt;
    } else {
      // Fall back to last known or zero
      return uwb_rx->getLastWorldPosition();
    }
  }

  bool checkGoalReached(const Vec2& pos, const Vec2& goal) {
    double dist = distance(pos, goal);
    return dist < cfg.WP_TOLERANCE;
  }

  bool isHeadingNotImproving(double window_sec, double min_improvement_rad) {
    // Check if heading error is not improving over time window
    if (error_history_.size() < 50) return false; // Need enough history
    
    // Simple check: compare average error now vs window_sec ago
    size_t samples = static_cast<size_t>(window_sec * 50); // Assuming 50Hz
    if (error_history_.size() < samples) return false;
    
    double recent_avg = 0;
    double old_avg = 0;
    size_t n = samples / 2;
    
    for (size_t i = 0; i < n; ++i) {
      old_avg += std::abs(error_history_[i]);
      recent_avg += std::abs(error_history_[error_history_.size() - 1 - i]);
    }
    
    old_avg /= n;
    recent_avg /= n;
    
    return (old_avg - recent_avg) < min_improvement_rad;
  }

  void doProbeMove(double distance_m, double speed_m_s) {
    // Simple forward probe move to unstick from local minima
    std::cout << "[Probe] Moving forward " << distance_m << "m at " << speed_m_s << "m/s\n";
    
    double duration = distance_m / speed_m_s;
    auto start = Clock::now();
    
    while (running && std::chrono::duration<double>(Clock::now() - start).count() < duration) {
      sport_client.Move(speed_m_s, 0, 0);
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    
    sport_client.Move(0, 0, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  void run() {
    const double dt_nom = 0.02; // 50 Hz
    auto next_time = Clock::now();
    int cycles = 0;
    
    // Initialize heading with current IMU yaw
    { 
      std::lock_guard<std::mutex> lock(mtx);
      psi_fused_ = imu_latest.roll; // Assuming yaw is in roll field (check your mapping!)
    }

    while (running) {
      next_time += std::chrono::milliseconds(20);
      cycles++;

      // Get current time
      auto t_now = Clock::now();
      double dt = std::chrono::duration<double>(t_now - last_loop_).count();
      if (dt <= 0.0 || dt > 0.2) dt = dt_nom; // Guard against bad dt
      last_loop_ = t_now;

      // Get IMU data
      IMUData imu;
      std::array<double,4> feet{};
      { 
        std::lock_guard<std::mutex> lock(mtx);
        imu = imu_latest;
        feet = foot_forces;
      }

      // 1) Fast heading update from gyro
      double gyro_z = imu.gyro_z;
      psi_fused_ = wrapAngle(psi_fused_ + gyro_z * dt);

      // 2) Slow absolute correction from UWB displacement over window
      auto t_then = t_now - std::chrono::duration_cast<Clock::duration>(
          std::chrono::duration<double>(cfg.DISP_WINDOW_SEC));

      auto p_now  = uwb_rx->positionAt(t_now);
      auto p_then = uwb_rx->positionAt(t_then);

      if (p_now && p_then) {
        Vec2 d = *p_now - *p_then;
        if (norm(d) >= cfg.DISP_MIN_M) {
          double psi_uwb = std::atan2(d.y, d.x);
          double dpsi = wrapAngle(psi_uwb - psi_fused_);
          psi_fused_ = wrapAngle(psi_fused_ + cfg.K_UWB * dpsi);
        }
      }

      // 3) Get current position and goal
      Vec2 pos = getCurrentPosition();
      Vec2 goal = getCurrentGoal();

      // Check if we reached current waypoint
      if (checkGoalReached(pos, goal)) {
        waypoint_index_++;
        if (waypoint_index_ >= static_cast<int>(waypoints_.size())) {
          waypoint_index_ = 0; // Loop back
        }
        goal = getCurrentGoal();
        std::cout << "[Waypoint] Reached waypoint " << waypoint_index_ 
                  << " at (" << goal.x << ", " << goal.y << ")\n";
      }

      // 4) Compute desired bearing to goal
      double psi_des = std::atan2(goal.y - pos.y, goal.x - pos.x);

      // 5) Error calculation with low-pass filter
      double err = wrapAngle(psi_des - psi_fused_);
      err_filt_ += cfg.ERR_ALPHA * (err - err_filt_);

      // Store error history for stall detection
      error_history_.push_back(err);
      if (error_history_.size() > 200) { // Keep last 4 seconds at 50Hz
        error_history_.pop_front();
      }

      // 6) Hysteresis state machine
      ControlState prev_state = state_;
      if (state_ == TURN && std::abs(deg(err_filt_)) < cfg.ENTER_TOL_DEG) {
        state_ = FWD;
        last_state_change_ = t_now;
        std::cout << "[State] TURN -> FWD (err=" << deg(err_filt_) << "°)\n";
      }
      if (state_ == FWD && std::abs(deg(err_filt_)) > cfg.EXIT_TOL_DEG) {
        state_ = TURN;
        last_state_change_ = t_now;
        std::cout << "[State] FWD -> TURN (err=" << deg(err_filt_) << "°)\n";
      }

      // 7) Curvature control with floors
      double v_cmd = 0.0, w_cmd = 0.0;
      
      if (state_ == TURN) {
        // Pure turning with deadband floor
        w_cmd = std::clamp(cfg.K_TURN * std::sin(err), -cfg.W_MAX, cfg.W_MAX);
        if (std::abs(w_cmd) < cfg.W_MIN) {
          w_cmd = std::copysign(cfg.W_MIN, w_cmd);
        }
        v_cmd = 0.0;
      } else {
        // Forward with curvature - always some forward motion
        double scale = std::pow(std::cos(0.5 * err), 2.0); // in [0,1]
        v_cmd = cfg.V_MIN + (cfg.V_NOM - cfg.V_MIN) * scale;
        
        // Proportional heading correction
        w_cmd = std::clamp(cfg.K_TURN * std::sin(err), -cfg.W_MAX, cfg.W_MAX);
        
        // De-chatter small corrections
        if (std::abs(w_cmd) < 0.15) {
          w_cmd = 0.0;
        }
      }

      // 8) Stall detection and probe move
      if (state_ == TURN) {
        double turn_duration = std::chrono::duration<double>(t_now - last_state_change_).count();
        
        if (turn_duration > cfg.STALL_CHECK_TIME) {
          bool not_improving = isHeadingNotImproving(0.5, rad(3.0));
          
          if (not_improving) {
            std::cout << "[Stall] Detected, executing probe move\n";
            doProbeMove(cfg.PROBE_DISTANCE, cfg.PROBE_SPEED);
            last_state_change_ = Clock::now();
            error_history_.clear(); // Reset history after probe
          }
        }
      }

      // 9) Apply low-pass filter to lateral and yaw commands
      double vy_filt = lp_vy.update(0.0, dt); // No lateral for this simple controller
      double wz_filt = lp_wz.update(w_cmd, dt);

      // Final command saturation
      v_cmd = clamp(v_cmd, 0.0, 0.5);
      vy_filt = clamp(vy_filt, -0.1, 0.1);
      wz_filt = clamp(wz_filt, -cfg.W_MAX, cfg.W_MAX);

      // 10) Send command
      sport_client.Move(v_cmd, vy_filt, wz_filt);
      last_cmd = {v_cmd, vy_filt, wz_filt};

      // 11) ZUPT if stationary
      bool zupt = false;
      if (checkZUPT()) {
        estimator.applyZUPT(imu.gyro_z);
        zupt = true;
      }

      // 12) Logging
      auto timestamp = std::chrono::duration<double>(
          std::chrono::system_clock::now().time_since_epoch()).count();
      
      // Create dummy values for compatibility with existing logger
      UWBMeasurement uwb_meas{};
      if (p_now) {
        // Fill in some basic UWB data if available
        uwb_meas.r = norm(*p_now);
        uwb_meas.beta = std::atan2(p_now->y, p_now->x);
      }
      
      Pose2D est_pose{pos.x, pos.y, psi_fused_};
      Pose2D pred_pose = est_pose; // For this simplified version
      
      logger.log(
        timestamp,
        mode, 
        static_cast<int>(state_),
        est_pose, 
        pred_pose,
        uwb_meas, 
        0.0, // latency
        est_pose, // uwb_body_pose
        {err, err, err}, // innovation (simplified)
        true, false, // innov_applied, clipped
        {0, 0, 0}, // shadow_innov
        false, false, // shadow_applied, clipped
        {v_cmd, 0, w_cmd}, // cmd_pre
        last_cmd, // cmd_post
        imu, 
        estimator.getYawBias(), 
        gyro_z,
        feet, 
        zupt, 
        body_height, 
        gait_type
      );

      // Status output
      if (cycles % 100 == 0) {
        uint64_t tmsg = 0, rej = 0; 
        uwb_rx->getStatistics(tmsg, rej);
        
        std::cout << "[Status] cyc=" << cycles
                  << " | State=" << (state_ == TURN ? "TURN" : "FWD")
                  << " | Pos=(" << std::fixed << std::setprecision(2) 
                  << pos.x << "," << pos.y << ")"
                  << " | ψ=" << deg(psi_fused_) << "°"
                  << " | Err=" << deg(err_filt_) << "°"
                  << " | Cmd=(v=" << v_cmd << ",w=" << wz_filt << ")"
                  << " | UWB(msg=" << tmsg << ",rej=" << rej << ")\n";
      }

      std::this_thread::sleep_until(next_time);
    }

    sport_client.Move(0, 0, 0);
    std::cout << "[Controller] Stopped\n";
  }
};

int main(int argc, char* argv[]) {
  std::string iface, cfg_file, mode = "SMOOTH_LOOP";

  for (int i = 1; i < argc; i++) {
    std::string a = argv[i];
    if (a == "--interface" && i + 1 < argc) {
      iface = argv[++i];
    } else if (a == "--config" && i + 1 < argc) {
      cfg_file = argv[++i];
    } else if (a == "--mode" && i + 1 < argc) {
      mode = argv[++i];
    } else if (a == "--help") {
      std::cout << "Usage: " << argv[0] 
                << " --interface <iface> [--config file] [--mode SMOOTH_LOOP|OPEN_LOOP|SHADOW]\n";
      return 0;
    }
  }

  if (iface.empty()) {
    std::cerr << "Error: Network interface required\n";
    std::cerr << "Usage: " << argv[0] << " --interface eth0\n";
    return 2;
  }
  
  if (!interfaceExists(iface)) {
    std::cerr << "Error: Interface '" << iface << "' not found\n";
    return 2;
  }
  
  if (mode != "SMOOTH_LOOP" && mode != "OPEN_LOOP" && mode != "SHADOW") {
    std::cerr << "Error: Invalid mode '" << mode << "'\n";
    return 2;
  }

  std::cout << "=====================================\n";
  std::cout << "UWB Path Follower for Unitree Go2\n";
  std::cout << "=====================================\n";
  std::cout << "Interface: " << iface << "\n";
  std::cout << "Config: " << (cfg_file.empty() ? "default" : cfg_file) << "\n";
  std::cout << "Mode: " << mode << "\n";
  std::cout << "Ctrl+C to stop\n";
  std::cout << "=====================================\n";

  std::signal(SIGINT, signalHandler);

  try {
    unitree::robot::ChannelFactory::Instance()->Init(0, iface.c_str());
    std::cout << "[Init] DDS initialized on interface: " << iface << std::endl;
  } catch (const std::exception& e) {
    std::cerr << "[Error] Failed to init DDS: " << e.what() << std::endl;
    return 1;
  }

  try {
    UWBPathController controller(iface, cfg_file, mode, g_running);
    controller.run();
  } catch (const std::exception& e) {
    std::cerr << "[Fatal] " << e.what() << std::endl;
    return 1;
  }

  std::cout << "[Main] Clean exit\n";
  return 0;
}