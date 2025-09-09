// include/uwb_path_follower/uwb_receiver.hpp
// Real UWB data acquisition from Unitree Go2 via DDS topic "rt/uwbstate"

#pragma once

#include "types.hpp"  // For UWBMeasurement struct

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
    
    // Latest measurement
    UWBMeasurement latest_measurement;
    
    // Tag IMU data (bonus - we get this for free!)
    struct {
        double roll = 0;
        double pitch = 0; 
        double yaw = 0;
    } tag_imu;
    
    // Base IMU data (also available)
    struct {
        double roll = 0;
        double pitch = 0;
        double yaw = 0;
    } base_imu;
    
    // Joystick values (if using manual control)
    double joystick_x = 0;
    double joystick_y = 0;
    
    // Tracking state
    uint8_t joy_mode = 0;
    uint8_t error_state = 0;
    bool enabled_from_app = false;
    
    // Timing
    std::chrono::steady_clock::time_point last_update_time;
    
public:
    UWBReceiver() {
        // Initialize DDS subscriber for UWB data
        uwb_subscriber = std::make_shared<unitree::robot::ChannelSubscriber<UwbState>>("rt/uwbstate");
        
        // Set up callback with lambda
        uwb_subscriber->InitChannel([this](const void* message) {
            handleUWBMessage(message);
        });
        
        std::cout << "[UWB] Receiver initialized on topic: rt/uwbstate" << std::endl;
    }
    
    void handleUWBMessage(const void* message) {
        const UwbState& msg = *(const UwbState*)message;
        
        std::lock_guard<std::mutex> lock(data_mutex);
        
        // Convert Unitree UWB format to our spherical format
        // Direct mapping based on documentation:
        latest_measurement.r = msg.distance_est();      // Distance in meters
        latest_measurement.beta = msg.orientation_est(); // Azimuth β in radians
        latest_measurement.alpha = msg.pitch_est();      // Elevation α in radians  
        latest_measurement.gamma = msg.yaw_est();        // Tag yaw γ in radians
        
        // Timestamp (use system time since message doesn't include timestamp)
        auto now = std::chrono::steady_clock::now();
        latest_measurement.timestamp = std::chrono::duration<double>(now.time_since_epoch()).count();
        
        // Quality estimation based on error state and mode
        // Good quality if error_state is 0 and in tracking mode
        if (msg.error_state() == 0) {
            if (msg.joy_mode() == 1 || msg.joy_mode() == 2) {  // Walk or run tracking
                latest_measurement.quality = 0.95;
            } else if (msg.joy_mode() == 0) {  // Joystick control
                latest_measurement.quality = 0.85;
            } else {
                latest_measurement.quality = 0.7;  // Other modes
            }
        } else {
            // Degraded quality based on error severity
            latest_measurement.quality = std::max(0.3, 0.7 - msg.error_state() * 0.1);
        }
        
        // Store additional IMU data (useful for cross-validation)
        tag_imu.roll = msg.tag_roll();
        tag_imu.pitch = msg.tag_pitch();
        tag_imu.yaw = msg.tag_yaw();
        
        base_imu.roll = msg.base_roll();
        base_imu.pitch = msg.base_pitch();
        base_imu.yaw = msg.base_yaw();
        
        // Store joystick values
        joystick_x = msg.joystick()[0];  // X-axis (forward positive)
        joystick_y = msg.joystick()[1];  // Y-axis (left positive)
        
        // Store state
        joy_mode = msg.joy_mode();
        error_state = msg.error_state();
        enabled_from_app = (msg.enabled_from_app() == 1);
        
        // Mark as available
        data_available = true;
        last_update_time = now;
        
        // Debug output (can be disabled in production)
        #ifdef DEBUG_UWB
        std::cout << "[UWB] r=" << latest_measurement.r 
                  << " β=" << latest_measurement.beta
                  << " α=" << latest_measurement.alpha
                  << " γ=" << latest_measurement.gamma
                  << " q=" << latest_measurement.quality
                  << " mode=" << (int)joy_mode
                  << " err=" << (int)error_state << std::endl;
        #endif
    }
    
    // Get latest measurement with freshness check
    bool getLatestMeasurement(UWBMeasurement& measurement, double max_age_seconds = 0.5) {
        std::lock_guard<std::mutex> lock(data_mutex);
        
        if (!data_available) {
            return false;
        }
        
        // Check age
        auto now = std::chrono::steady_clock::now();
        double age = std::chrono::duration<double>(now - last_update_time).count();
        
        if (age > max_age_seconds) {
            return false;  // Data too old
        }
        
        measurement = latest_measurement;
        return true;
    }
    
    // Get tag IMU data (bonus feature)
    void getTagIMU(double& roll, double& pitch, double& yaw) {
        std::lock_guard<std::mutex> lock(data_mutex);
        roll = tag_imu.roll;
        pitch = tag_imu.pitch;
        yaw = tag_imu.yaw;
    }
    
    // Get base IMU data (for cross-validation)
    void getBaseIMU(double& roll, double& pitch, double& yaw) {
        std::lock_guard<std::mutex> lock(data_mutex);
        roll = base_imu.roll;
        pitch = base_imu.pitch;
        yaw = base_imu.yaw;
    }
    
    // Get joystick values (if in manual mode)
    void getJoystick(double& x, double& y) {
        std::lock_guard<std::mutex> lock(data_mutex);
        x = joystick_x;
        y = joystick_y;
    }
    
    // Check if UWB is in tracking mode
    bool isTrackingEnabled() {
        std::lock_guard<std::mutex> lock(data_mutex);
        return (joy_mode == 1 || joy_mode == 2) && enabled_from_app;
    }
    
    // Get current mode
    std::string getModeName() {
        std::lock_guard<std::mutex> lock(data_mutex);
        switch(joy_mode) {
            case 0: return "Joystick";
            case 1: return "Walk Tracking";
            case 2: return "Run Tracking";
            case 3: return "Stand Up";
            case 4: return "Lie Down";
            case 5: return "Damping";
            case 6: return "Roll";
            case 7: return "Trigger Motion";
            case 8: return "Stand";
            default: return "Unknown";
        }
    }
    
    // Get error description
    std::string getErrorDescription() {
        std::lock_guard<std::mutex> lock(data_mutex);
        if (error_state == 0) return "No Error";
        return "Error Code: " + std::to_string(error_state);
    }
    
    bool hasData() const { return data_available; }
};

} // namespace uwb_path