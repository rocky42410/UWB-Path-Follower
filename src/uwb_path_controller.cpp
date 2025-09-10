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
#include <ifaddrs.h>

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/idl/go2/UwbState_.hpp>
#include <unitree/robot/go2/sport/sport_client.hpp>

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
  struct ifaddrs* ifaddr = nullptr; bool found=false;
  std::vector<std::string> list;
  if (getifaddrs(&ifaddr)==0) {
    for (auto* p=ifaddr; p; p=p->ifa_next) {
      if (!p->ifa_name) continue;
      std::string name = p->ifa_name;
      if (name==iface) found=true;
      list.push_back(name);
    }
    freeifaddrs(ifaddr);
  }
  if (!found && !list.empty()) {
    std::sort(list.begin(), list.end());
    list.erase(std::unique(list.begin(), list.end()), list.end());
    std::cerr << "Available interfaces: ";
    for (size_t i=0;i<list.size();++i) { if (i) std::cerr<<", "; std::cerr<<list[i];}
    std::cerr << "\n";
  }
  return found;
}

class UWBPathController {
  Config cfg;
  unitree::robot::go2::SportClient sport_client;
  std::shared_ptr<ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>> sport_sub;

  std::unique_ptr<UWBReceiver> uwb_rx;
  StateEstimator estimator;
  UWBConverter converter;
  PathTracker path;
  DataLogger logger;
  LowPassFilter lp_vy, lp_wz;

  std::atomic<bool>& running;
  std::mutex mtx;

  IMUData imu_latest{};
  std::array<double,4> foot_forces{};
  double body_height = 0.28;
  int gait_type = 0;

  Twist2D last_cmd{};
  std::string mode;
  std::string iface;

public:
  UWBPathController(const std::string& net_if,
                    const std::string& cfg_file,
                    const std::string& run_mode,
                    std::atomic<bool>& run_flag)
  : cfg(),
    estimator(cfg),
    converter(cfg),
    path(cfg),
    logger("uwb_path_log.csv"),
    lp_vy(cfg.LP_TAU),
    lp_wz(cfg.LP_TAU),
    running(run_flag),
    mode(run_mode),
    iface(net_if) {

    if (!cfg_file.empty()) cfg.loadFromFile(cfg_file);


    sport_client.SetTimeout(10.0f);
    sport_client.Init();
    std::cout << "[Init] Sport client OK\n";

    sport_sub = std::make_shared<ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>>("rt/sportmodestate");
    sport_sub->InitChannel([this](const void* msg){ if (msg) handleSportState(msg); });
    std::cout << "[Init] Sport state subscriber OK\n";

    uwb_rx = std::make_unique<UWBReceiver>(cfg.UWB_R_GATE, cfg.UWB_BETA_GATE, cfg.UWB_ALPHA_GATE, cfg.UWB_GAMMA_GATE);
    uwb_rx->updateSphericalGates(cfg);
    std::cout << "[Init] UWB receiver OK\n";

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    if (uwb_rx->hasData()) {
      std::cout << "[Init] UWB data available | Mode="<<uwb_rx->getModeName()<<" | "<<uwb_rx->getErrorDescription()<<"\n";
    } else {
      std::cout << "[Init] No UWB yet; check base station\n";
    }
    std::cout << "[Init] Mode: " << mode << "\n";
  }

  ~UWBPathController() {
    std::cout << "[Shutdown] Controller stopping...\n";
    running=false;
    try { sport_client.Move(0,0,0); } catch(...) {}
    if (sport_sub) sport_sub.reset();
    if (uwb_rx) { uwb_rx->shutdown(); uwb_rx.reset(); }
    std::cout << "[Shutdown] Done\n";
  }

  void handleSportState(const void* m) {
    if (!running) return;
    const auto& msg = *(const unitree_go::msg::dds_::SportModeState_*)m;
    std::lock_guard<std::mutex> lock(mtx);
    imu_latest.gyro_z = msg.imu_state().gyroscope()[2];
    imu_latest.roll   = msg.imu_state().rpy()[0];
    imu_latest.pitch  = msg.imu_state().rpy()[1];
    for (int i=0;i<4;i++) imu_latest.quaternion[i] = msg.imu_state().quaternion()[i];
    for (int i=0;i<4;i++) foot_forces[i] = msg.foot_force()[i];
    body_height = msg.body_height();
    gait_type = msg.gait_type();
  }

  bool checkZUPT() {
    double total = 0; for (auto f: foot_forces) total += f;
    bool low_motion = (std::fabs(last_cmd.vx) < cfg.ZUPT_VEL_THRESH) &&
                      (std::fabs(last_cmd.vy) < cfg.ZUPT_VEL_THRESH) &&
                      (std::fabs(imu_latest.gyro_z) < cfg.ZUPT_GYRO_THRESH);
    return (total > cfg.ZUPT_FOOT_THRESH) && low_motion;
  }

  Pose2D compensateLatency(const Pose2D& uwb_pose, double latency,
                           const Twist2D& cmd, double yaw_for_rotation) {
    if (latency <= 0 || latency > cfg.UWB_MAX_LATENCY) return uwb_pose;
    const double c = std::cos(yaw_for_rotation);
    const double s = std::sin(yaw_for_rotation);
    return {
      uwb_pose.x + (c*cmd.vx - s*cmd.vy) * latency,
      uwb_pose.y + (s*cmd.vx + c*cmd.vy) * latency,
      wrapAngle(uwb_pose.yaw + cmd.wz * latency)
    };
  }

  void run() {
    const double dt = 0.02; // 50 Hz
    auto next_time = std::chrono::steady_clock::now();
    int uwb_ok=0, uwb_bad=0, cycles=0;

    while (running) {
      next_time += std::chrono::milliseconds(20);
      cycles++;

      IMUData imu; std::array<double,4> feet{};
      { std::lock_guard<std::mutex> lock(mtx); imu = imu_latest; feet = foot_forces; }

      UWBMeasurement uwb_meas{};
      bool have_uwb = uwb_rx->getLatestMeasurement(uwb_meas, cfg.UWB_MAX_LATENCY);
      if (have_uwb && uwb_meas.quality < cfg.UWB_MIN_QUALITY) { have_uwb=false; uwb_bad++; }
      if (have_uwb) uwb_ok++;

      const double yaw_dot_used = imu.gyro_z - estimator.getYawBias();
      Pose2D pred = estimator.propagate(last_cmd, imu.gyro_z, dt);

      Pose2D uwb_body_pose{};
      Pose2D innov{}, shadow_innov{};
      bool innov_applied=false, innov_clipped=false, shadow_applied=false, shadow_clipped=false;
      double uwb_latency = 0;

      if (have_uwb) {
        auto now = std::chrono::system_clock::now();
        double now_s = std::chrono::duration<double>(now.time_since_epoch()).count();
        uwb_latency = now_s - uwb_meas.timestamp;
        if (uwb_latency < 0) uwb_latency = 0;

        uwb_body_pose = converter.uwbToBodyWorld(uwb_meas, imu, pred.yaw);
        uwb_body_pose = compensateLatency(uwb_body_pose, uwb_latency, last_cmd, pred.yaw);
      }

      Pose2D est = pred;
      if (mode=="SMOOTH_LOOP" && have_uwb) {
        auto [fused, ok, clipped] = estimator.fuseComplementary(pred, uwb_body_pose, dt);
        est = fused; innov_applied=ok; innov_clipped=clipped;
        innov = uwb_body_pose - pred;
      } else if (mode=="SHADOW" && have_uwb) {
        auto [fused, ok, clipped] = estimator.fuseComplementary(pred, uwb_body_pose, dt);
        (void)fused; // not applied
        shadow_innov = uwb_body_pose - pred;
        shadow_applied=false; shadow_clipped=clipped;
        innov = shadow_innov; innov_applied=false; innov_clipped=clipped;
      }

      estimator.updateState(est);

      Twist2D cmd_pre{};
      path.updatePhase(est, dt);
      switch (path.current_phase) {
        case PathTracker::FWD:
        case PathTracker::BACK: {
          const Pose2D tgt = path.target();
          const Pose2D start = (path.current_phase == PathTracker::FWD) ? Pose2D{0,0,0} : Pose2D{cfg.PATH_LENGTH,0,0};
          cmd_pre = path.computeStraightControl(est, start, tgt);
        } break;
        case PathTracker::TURN: cmd_pre = {0,0,0.3}; break;
        case PathTracker::SIT1:
        case PathTracker::SIT2: cmd_pre = {0,0,0}; break;
      }

      double vy_f = lp_vy.update(cmd_pre.vy, dt);
      double wz_f = lp_wz.update(cmd_pre.wz, dt);
      Twist2D cmd_post{ cmd_pre.vx, clamp(vy_f,-0.1,0.1), clamp(wz_f,-0.5,0.5) };

      sport_client.Move(cmd_post.vx, cmd_post.vy, cmd_post.wz);
      last_cmd = cmd_post;

      bool zupt=false;
      if ((path.current_phase==PathTracker::SIT1 || path.current_phase==PathTracker::SIT2) && checkZUPT()) {
        estimator.applyZUPT(imu.gyro_z);
        zupt = true;
      }

      logger.log(
        std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count(),
        mode, static_cast<int>(path.current_phase),
        est, pred,
        have_uwb ? uwb_meas : UWBMeasurement{}, uwb_latency,
        have_uwb ? uwb_body_pose : Pose2D{},
        innov, innov_applied, innov_clipped,
        shadow_innov, shadow_applied, shadow_clipped,
        cmd_pre, cmd_post,
        imu, estimator.getYawBias(), yaw_dot_used,
        feet, zupt, body_height, gait_type
      );

      if (cycles % 100 == 0) {
        double uwb_rate = (double)uwb_ok / cycles * 100.0;
        uint64_t tmsg=0, rej=0; uwb_rx->getStatistics(tmsg, rej);
        std::cout << "[Status] cyc="<<cycles
                  << " | UWB="<<std::fixed<<std::setprecision(1)<<uwb_rate<<"% (gate "<<rej<<"/"<<tmsg<<")"
                  << " | Pose=("<<est.x<<","<<est.y<<","<<est.yaw<<")"
                  << " | Bias="<<estimator.getYawBias()
                  << " | Lat(ms)="<<(uwb_latency*1000.0) << "\n";
      }

      std::this_thread::sleep_until(next_time);
    }

    sport_client.Move(0,0,0);
    std::cout << "[Controller] Stopped\n";
  }
};

int main(int argc, char* argv[]) {
  std::string iface, cfg_file, mode = "SMOOTH_LOOP";

  for (int i=1;i<argc;i++) {
    std::string a = argv[i];
    if (a=="--interface" && i+1<argc) iface = argv[++i];
    else if (a=="--config" && i+1<argc) cfg_file = argv[++i];
    else if (a=="--mode" && i+1<argc) mode = argv[++i];
    else if (a=="--help") {
      std::cout << "Usage: " << argv[0] << " --interface <iface> [--config file] [--mode SMOOTH_LOOP|OPEN_LOOP|SHADOW]\n";
      return 0;
    }
  }

  if (iface.empty()) {
    std::cerr << "Error: Network interface required\n";
    std::cerr << "Usage: " << argv[0] << " --interface eth0\n";
    return 2;
  }
  if (!interfaceExists(iface)) {
    std::cerr << "Error: Interface '"<<iface<<"' not found\n";
    return 2;
  }
  if (mode!="SMOOTH_LOOP" && mode!="OPEN_LOOP" && mode!="SHADOW") {
    std::cerr << "Error: Invalid mode\n";
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
