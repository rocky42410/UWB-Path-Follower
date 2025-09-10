// uwb_path_controller_updated.cpp
// Enhanced controller with point-to-point movement, discrete turns, and watchdog safety

#include <cmath>
#include <thread>
#include <chrono>
#include <atomic>
#include <mutex>
#include <fstream>
#include <iomanip>
#include <signal.h>
#include <csignal>
#include <memory>

// Unitree SDK includes
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/idl/go2/UwbState_.hpp>
#include <unitree/robot/go2/sport/sport_client.hpp>

using namespace unitree::robot;
using namespace unitree::common;

// ============= Enhanced Configuration =============
struct Config {
    // Base station
    struct {
        double x = 0.0, y = 0.0, z = 0.0;
        double yaw = 0.0;
    } base_station;
    
    // Tag mounting
    struct {
        double x = 0.0, y = 0.15, z = 0.0;
        double yaw_offset = 0.0;
    } tag_mount;
    
    // Point-to-point navigation
    double PATH_LENGTH = 1.0;        // m (walk distance)
    double WP_TOLERANCE = 0.08;      // m (position tolerance)
    double YAW_TOLERANCE = 0.06;     // rad (~3.4°)
    double SETTLE_TIME = 0.25;       // s (settling before confirm)
    double CMD_WATCHDOG = 2.0;       // s (safety timeout)
    double CRUISE_SPEED = 0.25;      // m/s (walking speed)
    double TURN_SPEED = 0.6;         // rad/s (turn rate)
    
    // Fusion parameters
    double K_POS = 0.1;
    double K_YAW = 0.15;
    double MAX_POS_INNOV = 0.5;
    double MAX_YAW_INNOV = 0.3;
    double MAX_POS_RATE = 0.2;
    double MAX_YAW_RATE = 0.5;
    
    // UWB quality
    double UWB_MIN_QUALITY = 0.7;
    double UWB_MAX_LATENCY = 0.3;
};

// ============= Basic Types =============
struct Pose2D {
    double x = 0, y = 0, yaw = 0;
    
    Pose2D operator-(const Pose2D& other) const {
        return {x - other.x, y - other.y, yaw - other.yaw};
    }
};

struct Twist2D {
    double vx = 0, vy = 0, wz = 0;
};

struct IMUData {
    double roll = 0, pitch = 0;
    double gyro_z = 0;
    std::array<double, 4> quaternion = {1,0,0,0};
};

struct UWBMeasurement {
    double r = 0;        // range
    double beta = 0;     // azimuth
    double alpha = 0;    // elevation
    double gamma = 0;    // tag yaw
    double timestamp = 0;
    double quality = 0;
};

// Utility functions
inline double wrapAngle(double angle) {
    while (angle > M_PI) angle -= 2*M_PI;
    while (angle < -M_PI) angle += 2*M_PI;
    return angle;
}

inline double angleDiff(double a, double b) {
    return wrapAngle(a - b);
}

inline double distance2D(double x1, double y1, double x2, double y2) {
    double dx = x2 - x1;
    double dy = y2 - y1;
    return std::sqrt(dx*dx + dy*dy);
}

// ============= Point-to-Point Path Controller =============
class PointToPointController {
public:
    enum Phase {
        INIT,           // Initial state
        CAPTURE_HOME,   // Record starting position
        WALK_FORWARD,   // Walk to target point
        SETTLE_FWD,     // Let body settle after walking
        TURN_180,       // Turn 180° in place
        SETTLE_TURN,    // Settle after turning
        WALK_BACK,      // Walk back to home
        SETTLE_BACK,    // Settle at home
        CONFIRM_HOME,   // Verify and correct position
        DONE,           // Mission complete
        FAULT           // Error state
    };

private:
    Config cfg;
    unitree::robot::go2::SportClient& sport_client;
    
    Phase current_phase = INIT;
    Pose2D home_pose;           // Starting position
    Pose2D target_pose;          // Forward target
    Pose2D current_uwb_pose;     // Latest UWB reading
    
    // Timing and watchdogs
    std::chrono::steady_clock::time_point phase_start_time;
    std::chrono::steady_clock::time_point last_cmd_time;
    std::chrono::steady_clock::time_point watchdog_deadline;
    
    // State flags
    bool uwb_valid = false;
    int confirmation_attempts = 0;
    const int MAX_CONFIRM_ATTEMPTS = 5;
    
    // Motion command rate limiting
    const double MIN_CMD_PERIOD = 0.05; // 20Hz max command rate
    
public:
    PointToPointController(unitree::robot::go2::SportClient& client, const Config& config) 
        : sport_client(client), cfg(config) {
        startRoutine();
    }
    
    void startRoutine() {
        current_phase = CAPTURE_HOME;
        phase_start_time = std::chrono::steady_clock::now();
        confirmation_attempts = 0;
    }
    
    void updateUWBPose(const Pose2D& pose, bool valid) {
        current_uwb_pose = pose;
        uwb_valid = valid;
        
        // Safety: halt if UWB lost during motion
        if (!valid && isMotionPhase()) {
            //emergencyStop();
        }
    }
    
    void tick() {
        auto now = std::chrono::steady_clock::now();
        
        // Check watchdog for motion phases
        if (isMotionPhase() && now > watchdog_deadline) {
            std::cout << "[WATCHDOG] Motion timeout! Emergency stop." << std::endl;
            emergencyStop();
            return;
        }
        
        // Rate limit commands
        if (shouldThrottleCommand()) return;
        
        switch (current_phase) {
            case INIT:
                startRoutine();
                break;
                
            case CAPTURE_HOME:
                handleCaptureHome();
                break;
                
            case WALK_FORWARD:
                handleWalkForward();
                break;
                
            case SETTLE_FWD:
                handleSettle(TURN_180);
                break;
                
            case TURN_180:
                handleTurn180();
                break;
                
            case SETTLE_TURN:
                handleSettle(WALK_BACK);
                break;
                
            case WALK_BACK:
                handleWalkBack();
                break;
                
            case SETTLE_BACK:
                handleSettle(CONFIRM_HOME);
                break;
                
            case CONFIRM_HOME:
                handleConfirmHome();
                break;
                
            case DONE:
                // Mission complete - could auto-restart here
                break;
                
            case FAULT:
                // Requires manual intervention
                break;
        }
    }
    
    Phase getPhase() const { return current_phase; }
    std::string getPhaseName() const {
        switch(current_phase) {
            case INIT: return "INIT";
            case CAPTURE_HOME: return "CAPTURE_HOME";
            case WALK_FORWARD: return "WALK_FORWARD";
            case SETTLE_FWD: return "SETTLE_FWD";
            case TURN_180: return "TURN_180";
            case SETTLE_TURN: return "SETTLE_TURN";
            case WALK_BACK: return "WALK_BACK";
            case SETTLE_BACK: return "SETTLE_BACK";
            case CONFIRM_HOME: return "CONFIRM_HOME";
            case DONE: return "DONE";
            case FAULT: return "FAULT";
            default: return "UNKNOWN";
        }
    }

private:
    void handleCaptureHome() {
        if (!uwb_valid) {
            std::cout << "[CAPTURE_HOME] Waiting for valid UWB..." << std::endl;
            return;
        }
        
        // Record home position
        home_pose = current_uwb_pose;
        
        // Calculate target 1m ahead along current heading
        target_pose.x = home_pose.x + std::cos(home_pose.yaw) * cfg.PATH_LENGTH;
        target_pose.y = home_pose.y + std::sin(home_pose.yaw) * cfg.PATH_LENGTH;
        target_pose.yaw = home_pose.yaw;
        
        std::cout << "[CAPTURE_HOME] Home: (" << home_pose.x << ", " << home_pose.y 
                  << ", " << home_pose.yaw << ")" << std::endl;
        std::cout << "[CAPTURE_HOME] Target: (" << target_pose.x << ", " << target_pose.y 
                  << ")" << std::endl;
        
        // Start walking forward
        sendWalkCommand(target_pose.x, target_pose.y);
        transitionTo(WALK_FORWARD);
    }
    
    void handleWalkForward() {
        double dist = distance2D(current_uwb_pose.x, current_uwb_pose.y, 
                                target_pose.x, target_pose.y);
        
        if (dist < cfg.WP_TOLERANCE) {
            std::cout << "[WALK_FORWARD] Reached target (dist=" << dist << "m)" << std::endl;
            sport_client.Move(0, 0, 0); // Stop
            transitionTo(SETTLE_FWD);
        } else {
            // Continue walking - resend command periodically
            if (getPhaseElapsed() > 0.5) { // Resend every 0.5s
                sendWalkCommand(target_pose.x, target_pose.y);
            }
        }
    }
    
    void handleTurn180() {
        // Calculate target heading (180° from initial)
        double target_yaw = wrapAngle(home_pose.yaw + M_PI);
        double yaw_error = angleDiff(target_yaw, current_uwb_pose.yaw);
        
        if (std::abs(yaw_error) < cfg.YAW_TOLERANCE) {
            std::cout << "[TURN_180] Turn complete (error=" << yaw_error << "rad)" << std::endl;
            sport_client.Move(0, 0, 0); // Stop
            transitionTo(SETTLE_TURN);
        } else {
            // Send turn command
            double turn_rate = (yaw_error > 0) ? cfg.TURN_SPEED : -cfg.TURN_SPEED;
            sport_client.Move(0, 0, turn_rate);
            armWatchdog(std::abs(yaw_error) / cfg.TURN_SPEED + 1.0);
        }
    }
    
    void handleWalkBack() {
        double dist = distance2D(current_uwb_pose.x, current_uwb_pose.y, 
                                home_pose.x, home_pose.y);
        
        if (dist < cfg.WP_TOLERANCE) {
            std::cout << "[WALK_BACK] Reached home (dist=" << dist << "m)" << std::endl;
            sport_client.Move(0, 0, 0); // Stop
            transitionTo(SETTLE_BACK);
        } else {
            // Continue walking back
            if (getPhaseElapsed() > 0.5) { // Resend every 0.5s
                sendWalkCommand(home_pose.x, home_pose.y);
            }
        }
    }
    
    void handleConfirmHome() {
        double pos_error = distance2D(current_uwb_pose.x, current_uwb_pose.y,
                                      home_pose.x, home_pose.y);
        double yaw_error = std::abs(angleDiff(home_pose.yaw, current_uwb_pose.yaw));
        
        std::cout << "[CONFIRM_HOME] Position error: " << pos_error 
                  << "m, Yaw error: " << yaw_error << "rad" << std::endl;
        
        if (pos_error < cfg.WP_TOLERANCE && yaw_error < cfg.YAW_TOLERANCE) {
            std::cout << "[CONFIRM_HOME] ✓ Landing confirmed!" << std::endl;
            sport_client.Move(0, 0, 0);
            transitionTo(DONE);
        } else if (confirmation_attempts >= MAX_CONFIRM_ATTEMPTS) {
            std::cout << "[CONFIRM_HOME] Max attempts reached. Accepting position." << std::endl;
            sport_client.Move(0, 0, 0);
            transitionTo(DONE);
        } else {
            confirmation_attempts++;
            
            // Correction nudges
            if (pos_error > cfg.WP_TOLERANCE) {
                // Small corrective walk
                sendWalkCommand(home_pose.x, home_pose.y, cfg.CRUISE_SPEED * 0.5);
                armWatchdog(2.0);
            } else if (yaw_error > cfg.YAW_TOLERANCE) {
                // Small corrective turn
                double delta = angleDiff(home_pose.yaw, current_uwb_pose.yaw);
                double turn_rate = (delta > 0) ? cfg.TURN_SPEED * 0.5 : -cfg.TURN_SPEED * 0.5;
                sport_client.Move(0, 0, turn_rate);
                armWatchdog(1.0);
            }
        }
    }
    
    void handleSettle(Phase next_phase) {
        if (getPhaseElapsed() > cfg.SETTLE_TIME) {
            std::cout << "[SETTLE] Complete, transitioning to " << getPhaseName(next_phase) << std::endl;
            transitionTo(next_phase);
        }
    }
    
    void sendWalkCommand(double target_x, double target_y, double speed = -1) {
        if (speed < 0) speed = cfg.CRUISE_SPEED;
        
        // Calculate direction to target
        double dx = target_x - current_uwb_pose.x;
        double dy = target_y - current_uwb_pose.y;
        double dist = std::sqrt(dx*dx + dy*dy);
        
        if (dist < 0.001) {
            sport_client.Move(0, 0, 0);
            return;
        }
        
        // Calculate velocity in robot frame
        double heading_to_target = std::atan2(dy, dx);
        double heading_error = angleDiff(heading_to_target, current_uwb_pose.yaw);
        
        // Forward velocity (slow down if heading is off)
        double vx = speed * std::cos(heading_error);
        if (std::abs(heading_error) > M_PI/4) {
            vx *= 0.3; // Slow down when turning
        }
        
        // Lateral correction
        double vy = -speed * std::sin(heading_error) * 0.3;
        
        // Heading correction
        double wz = heading_error * 1.0; // P-gain for heading
        wz = std::max(-cfg.TURN_SPEED, std::min(cfg.TURN_SPEED, wz));
        
        sport_client.Move(vx, vy, wz);
        
        // Set watchdog based on expected travel time
        double expected_time = dist / speed + 1.5;
        armWatchdog(expected_time);
        
        last_cmd_time = std::chrono::steady_clock::now();
    }
    
    void transitionTo(Phase new_phase) {
        current_phase = new_phase;
        phase_start_time = std::chrono::steady_clock::now();
        
        // Arm watchdog for motion phases
        if (isMotionPhase()) {
            armWatchdog(cfg.CMD_WATCHDOG);
        }
    }
    
    void armWatchdog(double seconds) {
        watchdog_deadline = std::chrono::steady_clock::now() + 
                           std::chrono::milliseconds(int(seconds * 1000));
    }
    
    bool isMotionPhase() const {
        return (current_phase == WALK_FORWARD || 
                current_phase == TURN_180 || 
                current_phase == WALK_BACK ||
                current_phase == CONFIRM_HOME);
    }
    
    bool shouldThrottleCommand() {
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - last_cmd_time).count();
        return elapsed < MIN_CMD_PERIOD;
    }
    
    double getPhaseElapsed() {
        auto now = std::chrono::steady_clock::now();
        return std::chrono::duration<double>(now - phase_start_time).count();
    }
    
    void emergencyStop() {
        sport_client.Move(0, 0, 0);
        current_phase = FAULT;
        std::cout << "[EMERGENCY] Stopped! Phase: " << getPhaseName() << std::endl;
    }
    
    std::string getPhaseName(Phase p) const {
        switch(p) {
            case INIT: return "INIT";
            case CAPTURE_HOME: return "CAPTURE_HOME";
            case WALK_FORWARD: return "WALK_FORWARD";
            case SETTLE_FWD: return "SETTLE_FWD";
            case TURN_180: return "TURN_180";
            case SETTLE_TURN: return "SETTLE_TURN";
            case WALK_BACK: return "WALK_BACK";
            case SETTLE_BACK: return "SETTLE_BACK";
            case CONFIRM_HOME: return "CONFIRM_HOME";
            case DONE: return "DONE";
            case FAULT: return "FAULT";
            default: return "UNKNOWN";
        }
    }
};

// ============= Simple UWB Receiver =============
class UWBReceiver {
private:
    ChannelSubscriberPtr<unitree_go::msg::dds_::UwbState_> uwb_sub;
    std::mutex data_mutex;
    UWBMeasurement latest;
    bool has_data = false;
    
public:
    UWBReceiver() {
        uwb_sub = std::make_shared<ChannelSubscriber<unitree_go::msg::dds_::UwbState_>>("rt/uwbstate");
        uwb_sub->InitChannel([this](const void* msg) {
            handleUWB(msg);
        });
    }
    
    void handleUWB(const void* message) {
        const auto& msg = *(const unitree_go::msg::dds_::UwbState_*)message;
        std::lock_guard<std::mutex> lock(data_mutex);
        
        latest.r = msg.distance_est();
        latest.beta = msg.orientation_est();
        latest.alpha = msg.pitch_est();
        latest.gamma = msg.yaw_est();
        latest.timestamp = std::chrono::duration<double>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        latest.quality = (msg.error_state() == 0) ? 0.9 : 0.5;
        has_data = true;
    }
    
    bool getMeasurement(UWBMeasurement& meas, double max_age = 0.3) {
        std::lock_guard<std::mutex> lock(data_mutex);
        if (!has_data) return false;
        
        double age = std::chrono::duration<double>(
            std::chrono::steady_clock::now().time_since_epoch()).count() - latest.timestamp;
        if (age > max_age) return false;
        
        meas = latest;
        return true;
    }
};

// ============= Main Controller =============
class UWBPathController {
private:
    Config cfg;
    unitree::robot::go2::SportClient sport_client;
    ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_> sport_subscriber;
    
    std::unique_ptr<UWBReceiver> uwb_receiver;
    std::unique_ptr<PointToPointController> point_controller;
    
    std::atomic<bool> running{true};
    std::mutex data_mutex;
    
    // Latest sensor data
    IMUData latest_imu;
    std::array<double, 4> latest_foot_forces{};
    
    std::string network_interface;
    
public:
    UWBPathController(const std::string& net_interface = "eth0") 
        : network_interface(net_interface) {
        
        // Initialize DDS
        //ChannelFactory::Instance()->Init(0, network_interface.c_str());
        //std::cout << "[Init] DDS initialized on interface: " << network_interface << std::endl;
        
        // Initialize sport client
        sport_client.SetTimeout(10.0f);
        sport_client.Init();
        std::cout << "[Init] Sport client initialized" << std::endl;
        
        // Setup sport state subscriber
        sport_subscriber = std::make_shared<ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>>(
            "rt/sportmodestate");
        sport_subscriber->InitChannel([this](const void* msg) { 
            handleSportState(msg); 
        });
        
        // Initialize UWB receiver
        uwb_receiver = std::make_unique<UWBReceiver>();
        std::cout << "[Init] UWB receiver initialized" << std::endl;
        
        // Initialize point-to-point controller
        point_controller = std::make_unique<PointToPointController>(sport_client, cfg);
        std::cout << "[Init] Point-to-point controller initialized" << std::endl;
        
        // Wait for initial data
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    
    void handleSportState(const void* message) {
        const auto& msg = *(const unitree_go::msg::dds_::SportModeState_*)message;
        
        std::lock_guard<std::mutex> lock(data_mutex);
        
        // Extract IMU data
        latest_imu.gyro_z = msg.imu_state().gyroscope()[2];
        latest_imu.roll = msg.imu_state().rpy()[0];
        latest_imu.pitch = msg.imu_state().rpy()[1];
        
        // Store quaternion
        for (int i = 0; i < 4; i++) {
            latest_imu.quaternion[i] = msg.imu_state().quaternion()[i];
        }
        
        // Extract foot forces
        for (int i = 0; i < 4; i++) {
            latest_foot_forces[i] = msg.foot_force()[i];
        }
    }
    
    Pose2D uwbToPose(const UWBMeasurement& uwb) {
        // Convert spherical UWB to 2D pose
        // Simplified conversion - adjust based on your setup
        double rho = uwb.r * std::cos(uwb.alpha);
        double x = rho * std::cos(uwb.beta);
        double y = rho * std::sin(uwb.beta);
        double yaw = uwb.gamma;
        
        return {x, y, yaw};
    }
    
    void run() {
        const double dt = 0.02;  // 50Hz
        auto next_time = std::chrono::steady_clock::now();
        
        int cycle_count = 0;
        
        while (running) {
            next_time += std::chrono::milliseconds(20);
            
            // Get UWB measurement
            UWBMeasurement uwb_meas;
            bool have_uwb = uwb_receiver->getMeasurement(uwb_meas, cfg.UWB_MAX_LATENCY);
            
            if (have_uwb && uwb_meas.quality >= cfg.UWB_MIN_QUALITY) {
                Pose2D uwb_pose = uwbToPose(uwb_meas);
                point_controller->updateUWBPose(uwb_pose, true);
            } else {
                point_controller->updateUWBPose(Pose2D{}, false);
            }
            
            // Run point-to-point controller
            point_controller->tick();
            
            // Status output
            if (++cycle_count % 50 == 0) {  // Every second
                std::cout << "[Status] Phase: " << point_controller->getPhaseName() 
                          << " | UWB: " << (have_uwb ? "OK" : "NO")
                          << " | Quality: " << uwb_meas.quality << std::endl;
            }
            
            // Check for completion or fault
            if (point_controller->getPhase() == PointToPointController::DONE) {
                std::cout << "[Controller] Mission complete! Press Ctrl+C to exit or wait for restart..." << std::endl;
                std::this_thread::sleep_for(std::chrono::seconds(3));
                // Optionally restart
                point_controller->startRoutine();
            }
            
            std::this_thread::sleep_until(next_time);
        }
    }
    
    void stop() {
        running = false;
        sport_client.Move(0, 0, 0);
    }
};

// ============= Main =============
int main(int argc, char* argv[]) {
    std::string network_interface = "eth0";
    
    // Parse arguments
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--interface" && i + 1 < argc) {
            network_interface = argv[++i];
        } else if (arg == "--help") {
            std::cout << "Usage: " << argv[0] << " [--interface <iface>]" << std::endl;
            return 0;
        }
    }
    
    std::cout << "=====================================" << std::endl;
    std::cout << "UWB Point-to-Point Path Controller" << std::endl;
    std::cout << "=====================================" << std::endl;
    std::cout << "Network Interface: " << network_interface << std::endl;
    std::cout << "Routine: Walk 1m forward → Turn 180° → Return home" << std::endl;
    std::cout << "Press Ctrl+C to stop" << std::endl;
    std::cout << "=====================================" << std::endl;
    try {
    unitree::robot::ChannelFactory::Instance()->Init(0, network_interface.c_str());
    std::cout << "[Init] DDS initialized on interface: " << network_interface << std::endl;
  } catch (const std::exception& e) {
    std::cerr << "[Error] Failed to init DDS: " << e.what() << std::endl;
    return 1;
  }


    
    
    try {
        UWBPathController controller(network_interface);
        
        std::signal(SIGINT, [](int) {
            std::cout << "\n[Signal] Shutting down..." << std::endl;
            std::exit(0);
        });
        
        controller.run();
    } catch (const std::exception& e) {
        std::cerr << "[Error] " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
