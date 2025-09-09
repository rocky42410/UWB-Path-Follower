#!/bin/bash
# create_headers.sh - Create all required header files with correct content

echo "Creating all required header files..."

# Create directory structure
mkdir -p include/uwb_path_follower

# 1. Create types.hpp
cat > include/uwb_path_follower/types.hpp << 'EOF'
#pragma once
#include <array>
#include <cmath>

namespace uwb_path {

struct Pose2D {
    double x = 0, y = 0, yaw = 0;
    
    Pose2D operator+(const Pose2D& other) const {
        return {x + other.x, y + other.y, yaw + other.yaw};
    }
    
    Pose2D operator-(const Pose2D& other) const {
        return {x - other.x, y - other.y, yaw - other.yaw};
    }
    
    Pose2D operator*(double scalar) const {
        return {x * scalar, y * scalar, yaw * scalar};
    }
};

struct Twist2D {
    double vx = 0, vy = 0, wz = 0;
};

struct UWBMeasurement {
    double r = 0;
    double beta = 0;
    double alpha = 0;
    double gamma = 0;
    double timestamp = 0;
    double quality = 0;
};

struct IMUData {
    double roll = 0, pitch = 0;
    double gyro_z = 0;
    std::array<double, 4> quaternion = {1,0,0,0};
};

inline double wrapAngle(double angle) {
    while (angle > M_PI) angle -= 2*M_PI;
    while (angle < -M_PI) angle += 2*M_PI;
    return angle;
}

inline double angleDiff(double a, double b) {
    return wrapAngle(a - b);
}

inline double clamp(double val, double min, double max) {
    return std::max(min, std::min(max, val));
}

} // namespace uwb_path
EOF

echo "✓ Created types.hpp"

# 2. Create config.hpp
cat > include/uwb_path_follower/config.hpp << 'EOF'
#pragma once
#include <string>
#include <iostream>
#include <fstream>

namespace uwb_path {

struct Config {
    struct {
        double x = 0.0, y = 0.0, z = 0.0;
        double yaw = 0.0;
    } base_station;
    
    struct {
        double x = 0.0, y = 0.15, z = 0.0;
        double yaw_offset = 0.0;
    } tag_mount;
    
    double K_POS = 0.1;
    double K_YAW = 0.15;
    double MAX_POS_INNOV = 0.5;
    double MAX_YAW_INNOV = 0.3;
    double MAX_POS_RATE = 0.2;
    double MAX_YAW_RATE = 0.5;
    double V_NOMINAL = 0.3;
    double KP_CROSS = 1.0;
    double KP_HEADING = 0.8;
    double LP_TAU = 0.25;
    double PATH_LENGTH = 5.0;
    double WP_TOLERANCE = 0.2;
    double UWB_MIN_QUALITY = 0.7;
    double UWB_MAX_LATENCY = 0.1;
    double ZUPT_FOOT_THRESH = 10.0;
    double ZUPT_VEL_THRESH = 0.05;
    double ZUPT_GYRO_THRESH = 0.05;
    
    bool loadFromFile(const std::string& filename) {
        std::cout << "Config loading not implemented (YAML optional)" << std::endl;
        return true;
    }
};

} // namespace uwb_path
EOF

echo "✓ Created config.hpp"

# 3. Create uwb_receiver.hpp
cat > include/uwb_path_follower/uwb_receiver.hpp << 'EOF'
#pragma once

#include "uwb_path_follower/types.hpp"
#include <unitree/idl/go2/UwbState_.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <mutex>
#include <atomic>
#include <chrono>
#include <iostream>
#include <memory>
#include <string>

namespace uwb_path {

class UWBReceiver {
private:
    using UwbState = unitree_go::msg::dds_::UwbState_;
    using ChannelSubscriberPtr = std::shared_ptr<unitree::robot::ChannelSubscriber<UwbState>>;
    
    ChannelSubscriberPtr uwb_subscriber;
    std::mutex data_mutex;
    std::atomic<bool> data_available{false};
    UWBMeasurement latest_measurement;
    std::chrono::steady_clock::time_point last_update_time;
    
    struct {
        double roll = 0, pitch = 0, yaw = 0;
    } tag_imu, base_imu;
    
    double joystick_x = 0, joystick_y = 0;
    uint8_t joy_mode = 0;
    uint8_t error_state = 0;
    bool enabled_from_app = false;
    
public:
    UWBReceiver() {
        uwb_subscriber = std::make_shared<unitree::robot::ChannelSubscriber<UwbState>>("rt/uwbstate");
        uwb_subscriber->InitChannel([this](const void* message) {
            handleUWBMessage(message);
        });
        std::cout << "[UWB] Receiver initialized on topic: rt/uwbstate" << std::endl;
    }
    
    void handleUWBMessage(const void* message) {
        const UwbState& msg = *(const UwbState*)message;
        std::lock_guard<std::mutex> lock(data_mutex);
        
        latest_measurement.r = msg.distance_est();
        latest_measurement.beta = msg.orientation_est();
        latest_measurement.alpha = msg.pitch_est();
        latest_measurement.gamma = msg.yaw_est();
        
        auto now = std::chrono::steady_clock::now();
        latest_measurement.timestamp = std::chrono::duration<double>(now.time_since_epoch()).count();
        
        if (msg.error_state() == 0) {
            latest_measurement.quality = (msg.joy_mode() == 1 || msg.joy_mode() == 2) ? 0.95 : 0.85;
        } else {
            latest_measurement.quality = std::max(0.3, 0.7 - msg.error_state() * 0.1);
        }
        
        data_available = true;
        last_update_time = now;
    }
    
    bool getLatestMeasurement(UWBMeasurement& measurement, double max_age_seconds = 0.5) {
        std::lock_guard<std::mutex> lock(data_mutex);
        if (!data_available) return false;
        
        auto now = std::chrono::steady_clock::now();
        double age = std::chrono::duration<double>(now - last_update_time).count();
        if (age > max_age_seconds) return false;
        
        measurement = latest_measurement;
        return true;
    }
    
    bool hasData() const { return data_available; }
    
    std::string getModeName() {
        std::lock_guard<std::mutex> lock(data_mutex);
        switch(joy_mode) {
            case 0: return "Joystick";
            case 1: return "Walk Tracking";
            case 2: return "Run Tracking";
            default: return "Unknown";
        }
    }
    
    std::string getErrorDescription() {
        std::lock_guard<std::mutex> lock(data_mutex);
        return (error_state == 0) ? "No Error" : "Error Code: " + std::to_string(error_state);
    }
    
    bool isTrackingEnabled() {
        std::lock_guard<std::mutex> lock(data_mutex);
        return (joy_mode == 1 || joy_mode == 2) && enabled_from_app;
    }
};

} // namespace uwb_path
EOF

echo "✓ Created uwb_receiver.hpp"

# 4. Create all_components.hpp (with all inline implementations)
cat > include/uwb_path_follower/all_components.hpp << 'EOF'
#pragma once

#include <cmath>
#include <tuple>
#include <fstream>
#include <iomanip>
#include <array>
#include <string>

#include "uwb_path_follower/types.hpp"
#include "uwb_path_follower/config.hpp"
#include "uwb_path_follower/uwb_receiver.hpp"

namespace uwb_path {

// All component implementations inline (no separate files)

class UWBConverter {
private:
    const Config& cfg;
    
public:
    UWBConverter(const Config& config) : cfg(config) {}
    
    Pose2D sphericalToBase(const UWBMeasurement& uwb) {
        double rho = uwb.r * cos(uwb.alpha);
        double x_b = rho * cos(uwb.beta);
        double y_b = rho * sin(uwb.beta);
        return {x_b, y_b, 0};
    }
    
    Pose2D baseToWorld(const Pose2D& p_base) {
        double c = cos(cfg.base_station.yaw);
        double s = sin(cfg.base_station.yaw);
        return {
            c * p_base.x - s * p_base.y + cfg.base_station.x,
            s * p_base.x + c * p_base.y + cfg.base_station.y,
            p_base.yaw
        };
    }
    
    double tagYawWorld(const UWBMeasurement& uwb) {
        return wrapAngle(cfg.base_station.yaw + uwb.gamma);
    }
    
    Pose2D rotateLever(double roll, double pitch, double yaw_pred) {
        double c = cos(yaw_pred);
        double s = sin(yaw_pred);
        return {
            c * cfg.tag_mount.x - s * cfg.tag_mount.y,
            s * cfg.tag_mount.x + c * cfg.tag_mount.y,
            0
        };
    }
    
    Pose2D uwbToBodyWorld(const UWBMeasurement& uwb, const IMUData& imu, double yaw_pred) {
        Pose2D p_tag_base = sphericalToBase(uwb);
        Pose2D p_tag_world = baseToWorld(p_tag_base);
        double yaw_tag = tagYawWorld(uwb);
        double yaw_body = wrapAngle(yaw_tag + cfg.tag_mount.yaw_offset);
        Pose2D lever_world = rotateLever(imu.roll, imu.pitch, yaw_pred);
        
        return {
            p_tag_world.x - lever_world.x,
            p_tag_world.y - lever_world.y,
            yaw_body
        };
    }
};

class StateEstimator {
private:
    Pose2D state;
    double yaw_bias = 0;
    const Config& cfg;
    
    bool gateInnovation(const Pose2D& innov) {
        return (fabs(innov.x) < cfg.MAX_POS_INNOV &&
                fabs(innov.y) < cfg.MAX_POS_INNOV &&
                fabs(innov.yaw) < cfg.MAX_YAW_INNOV);
    }
    
    Pose2D rateLimitInnovation(const Pose2D& innov, double dt) {
        double max_pos_change = cfg.MAX_POS_RATE * dt;
        double max_yaw_change = cfg.MAX_YAW_RATE * dt;
        
        return {
            clamp(innov.x, -max_pos_change, max_pos_change),
            clamp(innov.y, -max_pos_change, max_pos_change),
            clamp(innov.yaw, -max_yaw_change, max_yaw_change)
        };
    }
    
public:
    StateEstimator(const Config& config) : cfg(config) {}
    
    Pose2D propagate(const Twist2D& cmd, double gyro_z, double dt) {
        double yaw_rate = gyro_z - yaw_bias;
        double c = cos(state.yaw);
        double s = sin(state.yaw);
        
        Pose2D pred = state;
        pred.x += (c * cmd.vx - s * cmd.vy) * dt;
        pred.y += (s * cmd.vx + c * cmd.vy) * dt;
        pred.yaw = wrapAngle(pred.yaw + yaw_rate * dt);
        
        return pred;
    }
    
    std::tuple<Pose2D, bool, bool> fuseComplementary(const Pose2D& pred, const Pose2D& uwb_pose, double dt) {
        Pose2D innov = {
            uwb_pose.x - pred.x,
            uwb_pose.y - pred.y,
            angleDiff(uwb_pose.yaw, pred.yaw)
        };
        
        if (!gateInnovation(innov)) {
            return {pred, false, false};
        }
        
        Pose2D bounded = rateLimitInnovation(innov, dt);
        bool clipped = (bounded.x != innov.x || bounded.y != innov.y || bounded.yaw != innov.yaw);
        
        Pose2D fused = {
            pred.x + cfg.K_POS * bounded.x,
            pred.y + cfg.K_POS * bounded.y,
            wrapAngle(pred.yaw + cfg.K_YAW * bounded.yaw)
        };
        
        return {fused, true, clipped};
    }
    
    void setState(const Pose2D& pose) { state = pose; }
    Pose2D getState() const { return state; }
    void updateState(const Pose2D& pose) { state = pose; }
    void applyZUPT() { /* TODO: Implement */ }
};

class PathTracker {
private:
    const Config& cfg;
    
public:
    enum Phase { FWD, SIT1, TURN, BACK, SIT2 };
    
    Phase current_phase = FWD;
    double phase_timer = 0;
    Pose2D waypoint_start;
    
    PathTracker(const Config& config) : cfg(config) {}
    
    Twist2D computeStraightControl(const Pose2D& pose, const Pose2D& wp_start, const Pose2D& wp_end) {
        double dx = wp_end.x - wp_start.x;
        double dy = wp_end.y - wp_start.y;
        double path_yaw = atan2(dy, dx);
        
        double to_robot_x = pose.x - wp_start.x;
        double to_robot_y = pose.y - wp_start.y;
        double cross = -to_robot_x * sin(path_yaw) + to_robot_y * cos(path_yaw);
        double heading_err = angleDiff(path_yaw, pose.yaw);
        
        double vy_raw = -cfg.KP_CROSS * cross;
        double wz_raw = cfg.KP_HEADING * heading_err;
        
        return {cfg.V_NOMINAL, vy_raw, wz_raw};
    }
    
    bool reachedWaypoint(const Pose2D& pose, const Pose2D& wp) {
        double dist = sqrt(pow(pose.x - wp.x, 2) + pow(pose.y - wp.y, 2));
        return dist < cfg.WP_TOLERANCE;
    }
    
    Pose2D getCurrentTarget() {
        switch (current_phase) {
            case FWD:  return {cfg.PATH_LENGTH, 0, 0};
            case BACK: return {0, 0, 0};
            default:   return waypoint_start;
        }
    }
    
    void updatePhase(const Pose2D& pose, double dt) {
        phase_timer += dt;
        
        switch (current_phase) {
            case FWD:
                if (reachedWaypoint(pose, {cfg.PATH_LENGTH, 0, 0})) {
                    current_phase = SIT1;
                    phase_timer = 0;
                    waypoint_start = pose;
                }
                break;
            case SIT1:
                if (phase_timer > 2.0) {
                    current_phase = TURN;
                    phase_timer = 0;
                }
                break;
            case TURN:
                if (phase_timer > 3.0) {
                    current_phase = BACK;
                    phase_timer = 0;
                }
                break;
            case BACK:
                if (reachedWaypoint(pose, {0, 0, 0})) {
                    current_phase = SIT2;
                    phase_timer = 0;
                    waypoint_start = pose;
                }
                break;
            case SIT2:
                if (phase_timer > 2.0) {
                    current_phase = FWD;
                    phase_timer = 0;
                }
                break;
        }
    }
};

class LowPassFilter {
private:
    double value = 0;
    double tau;
    
public:
    LowPassFilter(double time_constant) : tau(time_constant) {}
    
    double update(double input, double dt) {
        double alpha = dt / (tau + dt);
        value = alpha * input + (1 - alpha) * value;
        return value;
    }
    
    double get() const { return value; }
};

class DataLogger {
private:
    std::ofstream log_file;
    
public:
    DataLogger(const std::string& filename) {
        log_file.open(filename);
        log_file << "timestamp,mode,phase,x_est,y_est,yaw_est,x_pred,y_pred,yaw_pred,"
                 << "uwb_r,uwb_beta,uwb_alpha,uwb_gamma,uwb_quality,"
                 << "innov_x,innov_y,innov_yaw,innov_ok,innov_clipped,"
                 << "cmd_vx,cmd_vy,cmd_wz,gyro_z,roll,pitch,"
                 << "foot_force_0,foot_force_1,foot_force_2,foot_force_3,zupt_active\n";
    }
    
    void log(double timestamp, const std::string& mode, int phase,
             const Pose2D& est, const Pose2D& pred,
             const UWBMeasurement& uwb, const Pose2D& innov,
             bool innov_ok, bool clipped,
             const Twist2D& cmd, const IMUData& imu,
             const std::array<double, 4>& foot_forces, bool zupt) {
        
        log_file << std::fixed << std::setprecision(6)
                 << timestamp << "," << mode << "," << phase << ","
                 << est.x << "," << est.y << "," << est.yaw << ","
                 << pred.x << "," << pred.y << "," << pred.yaw << ","
                 << uwb.r << "," << uwb.beta << "," << uwb.alpha << "," 
                 << uwb.gamma << "," << uwb.quality << ","
                 << innov.x << "," << innov.y << "," << innov.yaw << ","
                 << innov_ok << "," << clipped << ","
                 << cmd.vx << "," << cmd.vy << "," << cmd.wz << ","
                 << imu.gyro_z << "," << imu.roll << "," << imu.pitch << ","
                 << foot_forces[0] << "," << foot_forces[1] << "," 
                 << foot_forces[2] << "," << foot_forces[3] << ","
                 << zupt << "\n";
    }
    
    ~DataLogger() {
        if (log_file.is_open()) log_file.close();
    }
};

} // namespace uwb_path
EOF

echo "✓ Created all_components.hpp"

echo ""
echo "✅ All required header files created successfully!"
echo ""
echo "Files created:"
echo "  - include/uwb_path_follower/types.hpp"
echo "  - include/uwb_path_follower/config.hpp"
echo "  - include/uwb_path_follower/uwb_receiver.hpp"
echo "  - include/uwb_path_follower/all_components.hpp"
echo ""
echo "Now you can compile the project:"
echo "  cd build"
echo "  cmake .."
echo "  make"