// src/uwb_path_controller.cpp
// Main controller with proper modular includes

#include <cmath>
#include <thread>
#include <chrono>
#include <atomic>
#include <mutex>
#include <fstream>
#include <iomanip>
#include <signal.h>
#include <memory>

// Unitree SDK includes
#include <unitree/robot/channel/channel_factory.hpp>
//#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/robot/client/sport_client.hpp>

// Our modular includes - all components
#include "uwb_path_follower/all_components.hpp"

using namespace unitree::robot;
using namespace unitree::common;
using namespace uwb_path;

// ============= Main Controller =============
class UWBPathController {
private:
    Config cfg;
    SportClient sport_client;
    ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_> sport_subscriber;
    
    // Components
    std::unique_ptr<UWBReceiver> uwb_receiver;
    StateEstimator estimator;
    UWBConverter uwb_converter;
    PathTracker path_tracker;
    DataLogger logger;
    
    LowPassFilter lp_vy;
    LowPassFilter lp_wz;
    
    std::atomic<bool> running{true};
    std::mutex data_mutex;
    
    // Latest sensor data
    IMUData latest_imu;
    std::array<double, 4> latest_foot_forces{};
    
    Twist2D last_cmd;
    std::string mode = "SMOOTH_LOOP";
    std::string network_interface;
    
public:
    UWBPathController(const std::string& net_interface = "eth0", 
                      const std::string& config_file = "") : 
        cfg(),
        estimator(cfg),
        uwb_converter(cfg),
        path_tracker(cfg),
        logger("uwb_path_log.csv"),
        lp_vy(cfg.LP_TAU),
        lp_wz(cfg.LP_TAU),
        network_interface(net_interface) {
        
        // Load config if provided
        if (!config_file.empty()) {
            cfg.loadFromFile(config_file);
        }
        
        // Initialize DDS channel factory
        ChannelFactory::Instance()->Init(0, network_interface.c_str());
        std::cout << "[Init] DDS initialized on interface: " << network_interface << std::endl;
        
        // Initialize sport client
        sport_client.Init();
        std::cout << "[Init] Sport client initialized" << std::endl;
        
        // Setup sport state subscriber
        sport_subscriber = std::make_shared<ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>>(
            "rt/sportmodestate");
        sport_subscriber->InitChannel([this](const void* msg) { 
            handleSportState(msg); 
        });
        std::cout << "[Init] Sport state subscriber initialized" << std::endl;
        
        // Initialize UWB receiver
        uwb_receiver = std::make_unique<UWBReceiver>();
        std::cout << "[Init] UWB receiver initialized on rt/uwbstate" << std::endl;
        
        // Wait for initial data
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        // Check UWB status
        if (uwb_receiver->hasData()) {
            std::cout << "[Init] ✅ UWB data available" << std::endl;
            std::cout << "[Init] UWB Mode: " << uwb_receiver->getModeName() << std::endl;
            std::cout << "[Init] UWB Status: " << uwb_receiver->getErrorDescription() << std::endl;
        } else {
            std::cout << "[Init] ⚠️ No UWB data yet - check base station" << std::endl;
        }
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
    
    bool checkZUPT() {
        double total_force = 0;
        for (auto f : latest_foot_forces) total_force += f;
        
        bool low_motion = (fabs(last_cmd.vx) < cfg.ZUPT_VEL_THRESH &&
                           fabs(last_cmd.vy) < cfg.ZUPT_VEL_THRESH &&
                           fabs(latest_imu.gyro_z) < cfg.ZUPT_GYRO_THRESH);
        
        return (total_force > cfg.ZUPT_FOOT_THRESH && low_motion);
    }
    
    void run() {
        const double dt = 0.02;  // 50Hz
        auto next_time = std::chrono::steady_clock::now();
        
        int uwb_success_count = 0;
        int uwb_reject_count = 0;
        int total_cycles = 0;
        
        while (running) {
            next_time += std::chrono::milliseconds(20);
            total_cycles++;
            
            // Get sensor snapshot
            IMUData imu;
            std::array<double, 4> foot_forces;
            {
                std::lock_guard<std::mutex> lock(data_mutex);
                imu = latest_imu;
                foot_forces = latest_foot_forces;
            }
            
            // Get UWB data
            UWBMeasurement uwb_meas;
            bool have_uwb = uwb_receiver->getLatestMeasurement(uwb_meas, cfg.UWB_MAX_LATENCY);
            
            if (have_uwb && uwb_meas.quality < cfg.UWB_MIN_QUALITY) {
                have_uwb = false;
                uwb_reject_count++;
            }
            
            if (have_uwb) {
                uwb_success_count++;
            }
            
            // State prediction
            Pose2D pred_pose = estimator.propagate(last_cmd, imu.gyro_z, dt);
            
            // UWB update
            Pose2D uwb_body_pose;
            Pose2D innov;
            bool innov_ok = false;
            bool innov_clipped = false;
            
            if (have_uwb) {
                auto now = std::chrono::steady_clock::now();
                double current_time = std::chrono::duration<double>(now.time_since_epoch()).count();
                double latency = current_time - uwb_meas.timestamp;
                
                uwb_body_pose = uwb_converter.uwbToBodyWorld(uwb_meas, imu, 
                                                             estimator.getState().yaw);
                
                // Latency compensation
                if (latency > 0 && latency < cfg.UWB_MAX_LATENCY) {
                    uwb_body_pose.x += last_cmd.vx * cos(uwb_body_pose.yaw) * latency;
                    uwb_body_pose.y += last_cmd.vx * sin(uwb_body_pose.yaw) * latency;
                    uwb_body_pose.yaw += last_cmd.wz * latency;
                }
            }
            
            // Fusion
            Pose2D est_pose = pred_pose;
            if (mode == "SMOOTH_LOOP" && have_uwb) {
                auto [fused, ok, clipped] = estimator.fuseComplementary(pred_pose, 
                                                                        uwb_body_pose, dt);
                est_pose = fused;
                innov_ok = ok;
                innov_clipped = clipped;
                innov = uwb_body_pose - pred_pose;
            }
            
            estimator.updateState(est_pose);
            
            // Path following
            Twist2D raw_cmd;
            path_tracker.updatePhase(est_pose, dt);
            
            switch (path_tracker.current_phase) {
                case PathTracker::FWD:
                case PathTracker::BACK: {
                    Pose2D target = path_tracker.getCurrentTarget();
                    Pose2D start = (path_tracker.current_phase == PathTracker::FWD) ? 
                                   Pose2D{0,0,0} : Pose2D{cfg.PATH_LENGTH,0,0};
                    raw_cmd = path_tracker.computeStraightControl(est_pose, start, target);
                    break;
                }
                case PathTracker::TURN:
                    raw_cmd = {0, 0, 0.3};
                    break;
                case PathTracker::SIT1:
                case PathTracker::SIT2:
                    raw_cmd = {0, 0, 0};
                    break;
            }
            
            // Low-pass filtering
            double vy_filtered = lp_vy.update(raw_cmd.vy, dt);
            double wz_filtered = lp_wz.update(raw_cmd.wz, dt);
            
            // Saturation
            Twist2D cmd = {
                raw_cmd.vx,
                clamp(vy_filtered, -0.1, 0.1),
                clamp(wz_filtered, -0.5, 0.5)
            };
            
            // Send command
            sport_client.Move(cmd.vx, cmd.vy, cmd.wz);
            last_cmd = cmd;
            
            // ZUPT check
            bool zupt_active = false;
            if ((path_tracker.current_phase == PathTracker::SIT1 || 
                 path_tracker.current_phase == PathTracker::SIT2) && checkZUPT()) {
                estimator.applyZUPT();
                zupt_active = true;
            }
            
            // Logging
            logger.log(std::chrono::duration<double>(
                      std::chrono::steady_clock::now().time_since_epoch()).count(),
                      mode, path_tracker.current_phase,
                      est_pose, pred_pose, 
                      have_uwb ? uwb_meas : UWBMeasurement{},
                      innov, innov_ok, innov_clipped, cmd, imu,
                      foot_forces, zupt_active);
            
            // Status output
            if (total_cycles % 100 == 0) {
                double uwb_rate = (double)uwb_success_count / total_cycles * 100;
                std::cout << "[Status] Cycle " << total_cycles 
                          << " | UWB rate: " << std::fixed << std::setprecision(1) 
                          << uwb_rate << "%"
                          << " | Pose: (" << est_pose.x << ", " << est_pose.y 
                          << ", " << est_pose.yaw << ")" << std::endl;
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
    std::string config_file = "";
    std::string mode = "SMOOTH_LOOP";
    
    // Parse arguments
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--interface" && i + 1 < argc) {
            network_interface = argv[++i];
        } else if (arg == "--config" && i + 1 < argc) {
            config_file = argv[++i];
        } else if (arg == "--mode" && i + 1 < argc) {
            mode = argv[++i];
        } else if (arg == "--help") {
            std::cout << "Usage: " << argv[0] << " [options]" << std::endl;
            std::cout << "Options:" << std::endl;
            std::cout << "  --interface <n>  Network interface (default: eth0)" << std::endl;
            std::cout << "  --config <file>  Config file (optional)" << std::endl;
            std::cout << "  --mode <mode>    SMOOTH_LOOP, OPEN_LOOP, or SHADOW" << std::endl;
            return 0;
        }
    }
    
    std::cout << "=====================================" << std::endl;
    std::cout << "UWB Path Follower for Unitree Go2" << std::endl;
    std::cout << "=====================================" << std::endl;
    std::cout << "Network Interface: " << network_interface << std::endl;
    std::cout << "Config: " << (config_file.empty() ? "default" : config_file) << std::endl;
    std::cout << "Mode: " << mode << std::endl;
    std::cout << "Press Ctrl+C to stop" << std::endl;
    std::cout << "=====================================" << std::endl;
    
    try {
        UWBPathController controller(network_interface, config_file);
        
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