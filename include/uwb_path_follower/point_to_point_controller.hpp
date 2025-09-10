// point_to_point_controller.hpp
// Modular header for point-to-point navigation with UWB feedback

#pragma once

#include <cmath>
#include <chrono>
#include <mutex>
#include <atomic>
#include <iostream>
#include <array>

namespace uwb_nav {

// ============= Configuration =============
struct PointToPointConfig {
    // Navigation parameters
    double path_length = 1.0;        // Distance to walk forward (m)
    double wp_tolerance = 0.08;      // Position tolerance (m)
    double yaw_tolerance = 0.06;     // Heading tolerance (rad)
    
    // Timing parameters
    double settle_time = 0.25;       // Settling duration (s)
    double cmd_watchdog = 2.0;       // Motion timeout (s)
    double cmd_period = 0.05;        // Min time between commands (s)
    
    // Motion parameters
    double cruise_speed = 0.25;      // Normal walking speed (m/s)
    double turn_speed = 0.6;         // Yaw rate for turns (rad/s)
    double correction_factor = 0.5;  // Speed reduction for corrections
    
    // Safety parameters
    int max_corrections = 5;         // Max position correction attempts
    double uwb_max_age = 0.3;        // Max age for UWB data (s)
    double uwb_min_quality = 0.7;    // Min acceptable UWB quality
};

// ============= Basic Types =============
struct Pose2D {
    double x = 0;
    double y = 0; 
    double yaw = 0;
    
    double distanceTo(const Pose2D& other) const {
        double dx = other.x - x;
        double dy = other.y - y;
        return std::sqrt(dx*dx + dy*dy);
    }
    
    double headingTo(const Pose2D& other) const {
        double dx = other.x - x;
        double dy = other.y - y;
        return std::atan2(dy, dx);
    }
};

struct Velocity2D {
    double vx = 0;    // Forward velocity (m/s)
    double vy = 0;    // Lateral velocity (m/s)
    double wz = 0;    // Yaw rate (rad/s)
};

// ============= Motion Interface =============
class MotionInterface {
public:
    virtual ~MotionInterface() = default;
    
    // Send velocity command to robot
    virtual void sendVelocity(const Velocity2D& vel) = 0;
    
    // Emergency stop
    virtual void stop() = 0;
    
    // Check if robot is ready
    virtual bool isReady() = 0;
};

// ============= UWB Interface =============  
class UWBInterface {
public:
    virtual ~UWBInterface() = default;
    
    // Get current pose from UWB (returns true if valid)
    virtual bool getCurrentPose(Pose2D& pose, double& quality) = 0;
    
    // Check if UWB data is fresh
    virtual bool isDataFresh(double max_age_sec) = 0;
};

// ============= Point-to-Point Controller =============
class PointToPointController {
public:
    enum class State {
        INIT,
        CAPTURE_HOME,
        WALK_FORWARD,
        SETTLE_FORWARD,
        TURN_180,
        SETTLE_TURN,
        WALK_BACK,
        SETTLE_BACK,
        CONFIRM_HOME,
        DONE,
        FAULT
    };
    
    PointToPointController(MotionInterface* motion, UWBInterface* uwb, 
                          const PointToPointConfig& config = {})
        : motion_(motion), uwb_(uwb), config_(config) {
        reset();
    }
    
    // Main update function (call at 20-50Hz)
    void update() {
        updateTiming();
        updateUWB();
        checkWatchdog();
        
        switch(state_) {
            case State::INIT:
                handleInit();
                break;
            case State::CAPTURE_HOME:
                handleCaptureHome();
                break;
            case State::WALK_FORWARD:
                handleWalkForward();
                break;
            case State::SETTLE_FORWARD:
                handleSettle(State::TURN_180);
                break;
            case State::TURN_180:
                handleTurn180();
                break;
            case State::SETTLE_TURN:
                handleSettle(State::WALK_BACK);
                break;
            case State::WALK_BACK:
                handleWalkBack();
                break;
            case State::SETTLE_BACK:
                handleSettle(State::CONFIRM_HOME);
                break;
            case State::CONFIRM_HOME:
                handleConfirmHome();
                break;
            case State::DONE:
                handleDone();
                break;
            case State::FAULT:
                handleFault();
                break;
        }
    }
    
    // Reset to start new routine
    void reset() {
        state_ = State::INIT;
        correction_attempts_ = 0;
        uwb_valid_ = false;
        state_start_time_ = std::chrono::steady_clock::now();
    }
    
    // Get current state
    State getState() const { return state_; }
    
    // Get state name for debugging
    std::string getStateName() const {
        switch(state_) {
            case State::INIT: return "INIT";
            case State::CAPTURE_HOME: return "CAPTURE_HOME";
            case State::WALK_FORWARD: return "WALK_FORWARD";
            case State::SETTLE_FORWARD: return "SETTLE_FORWARD";
            case State::TURN_180: return "TURN_180";
            case State::SETTLE_TURN: return "SETTLE_TURN";
            case State::WALK_BACK: return "WALK_BACK";
            case State::SETTLE_BACK: return "SETTLE_BACK";
            case State::CONFIRM_HOME: return "CONFIRM_HOME";
            case State::DONE: return "DONE";
            case State::FAULT: return "FAULT";
            default: return "UNKNOWN";
        }
    }
    
    // Check if mission complete
    bool isDone() const { return state_ == State::DONE; }
    
    // Check if in fault state
    bool isFault() const { return state_ == State::FAULT; }

private:
    // Interfaces
    MotionInterface* motion_;
    UWBInterface* uwb_;
    
    // Configuration
    PointToPointConfig config_;
    
    // State machine
    State state_ = State::INIT;
    std::chrono::steady_clock::time_point state_start_time_;
    std::chrono::steady_clock::time_point last_cmd_time_;
    std::chrono::steady_clock::time_point watchdog_deadline_;
    
    // Navigation data
    Pose2D home_pose_;
    Pose2D target_pose_;
    Pose2D current_pose_;
    double uwb_quality_ = 0;
    bool uwb_valid_ = false;
    
    // Correction tracking
    int correction_attempts_ = 0;
    
    // Utility functions
    double wrapAngle(double angle) {
        while (angle > M_PI) angle -= 2*M_PI;
        while (angle < -M_PI) angle += 2*M_PI;
        return angle;
    }
    
    double angleDiff(double a, double b) {
        return wrapAngle(a - b);
    }
    
    void updateTiming() {
        // Rate limiting
        auto now = std::chrono::steady_clock::now();
        double since_last = std::chrono::duration<double>(now - last_cmd_time_).count();
        if (since_last < config_.cmd_period) {
            std::this_thread::sleep_for(
                std::chrono::milliseconds(int((config_.cmd_period - since_last) * 1000)));
        }
    }
    
    void updateUWB() {
        uwb_valid_ = uwb_->getCurrentPose(current_pose_, uwb_quality_) &&
                    uwb_quality_ >= config_.uwb_min_quality &&
                    uwb_->isDataFresh(config_.uwb_max_age);
                    
        // Safety: stop if UWB lost during motion
        if (!uwb_valid_ && isMotionState()) {
            enterFault("UWB signal lost during motion");
        }
    }
    
    void checkWatchdog() {
        if (isMotionState() && std::chrono::steady_clock::now() > watchdog_deadline_) {
            enterFault("Motion watchdog timeout");
        }
    }
    
    bool isMotionState() const {
        return state_ == State::WALK_FORWARD || 
               state_ == State::TURN_180 ||
               state_ == State::WALK_BACK ||
               state_ == State::CONFIRM_HOME;
    }
    
    void armWatchdog(double seconds) {
        watchdog_deadline_ = std::chrono::steady_clock::now() + 
                            std::chrono::milliseconds(int(seconds * 1000));
    }
    
    double getStateElapsed() const {
        auto now = std::chrono::steady_clock::now();
        return std::chrono::duration<double>(now - state_start_time_).count();
    }
    
    void transitionTo(State new_state) {
        std::cout << "[State] " << getStateName() << " -> ";
        state_ = new_state;
        state_start_time_ = std::chrono::steady_clock::now();
        std::cout << getStateName() << std::endl;
        
        // Arm watchdog for motion states
        if (isMotionState()) {
            armWatchdog(config_.cmd_watchdog);
        }
    }
    
    void enterFault(const std::string& reason) {
        std::cout << "[FAULT] " << reason << std::endl;
        motion_->stop();
        state_ = State::FAULT;
    }
    
    // State handlers
    void handleInit() {
        if (motion_->isReady()) {
            transitionTo(State::CAPTURE_HOME);
        }
    }
    
    void handleCaptureHome() {
        if (!uwb_valid_) {
            if (getStateElapsed() > 5.0) {
                enterFault("No UWB signal at startup");
            }
            return;
        }
        
        // Record home position
        home_pose_ = current_pose_;
        
        // Calculate target point ahead
        target_pose_.x = home_pose_.x + std::cos(home_pose_.yaw) * config_.path_length;
        target_pose_.y = home_pose_.y + std::sin(home_pose_.yaw) * config_.path_length;
        target_pose_.yaw = home_pose_.yaw;
        
        std::cout << "[Home] " << home_pose_.x << ", " << home_pose_.y 
                  << ", " << home_pose_.yaw << " rad" << std::endl;
        
        // Start walking
        sendWalkCommand(target_pose_);
        transitionTo(State::WALK_FORWARD);
    }
    
    void handleWalkForward() {
        double dist = current_pose_.distanceTo(target_pose_);
        
        if (dist < config_.wp_tolerance) {
            motion_->stop();
            transitionTo(State::SETTLE_FORWARD);
        } else if (getStateElapsed() > 0.5) {
            // Resend command periodically
            sendWalkCommand(target_pose_);
        }
    }
    
    void handleTurn180() {
        double target_yaw = wrapAngle(home_pose_.yaw + M_PI);
        double yaw_error = angleDiff(target_yaw, current_pose_.yaw);
        
        if (std::abs(yaw_error) < config_.yaw_tolerance) {
            motion_->stop();
            transitionTo(State::SETTLE_TURN);
        } else {
            // Send turn command
            Velocity2D vel;
            vel.wz = (yaw_error > 0) ? config_.turn_speed : -config_.turn_speed;
            motion_->sendVelocity(vel);
            
            // Update watchdog based on expected duration
            armWatchdog(std::abs(yaw_error) / config_.turn_speed + 1.0);
        }
    }
    
    void handleWalkBack() {
        double dist = current_pose_.distanceTo(home_pose_);
        
        if (dist < config_.wp_tolerance) {
            motion_->stop();
            transitionTo(State::SETTLE_BACK);
        } else if (getStateElapsed() > 0.5) {
            sendWalkCommand(home_pose_);
        }
    }
    
    void handleConfirmHome() {
        double pos_error = current_pose_.distanceTo(home_pose_);
        double yaw_error = std::abs(angleDiff(home_pose_.yaw, current_pose_.yaw));
        
        if (pos_error < config_.wp_tolerance && yaw_error < config_.yaw_tolerance) {
            std::cout << "[Confirmed] Position error: " << pos_error * 100 << " cm" << std::endl;
            motion_->stop();
            transitionTo(State::DONE);
        } else if (correction_attempts_ >= config_.max_corrections) {
            std::cout << "[Accept] Final error: " << pos_error * 100 << " cm" << std::endl;
            motion_->stop();
            transitionTo(State::DONE);
        } else {
            correction_attempts_++;
            
            if (pos_error > config_.wp_tolerance) {
                // Position correction
                sendWalkCommand(home_pose_, config_.cruise_speed * config_.correction_factor);
            } else {
                // Heading correction
                Velocity2D vel;
                double delta = angleDiff(home_pose_.yaw, current_pose_.yaw);
                vel.wz = (delta > 0) ? config_.turn_speed * config_.correction_factor 
                                     : -config_.turn_speed * config_.correction_factor;
                motion_->sendVelocity(vel);
            }
            armWatchdog(2.0);
        }
    }
    
    void handleSettle(State next_state) {
        if (getStateElapsed() > config_.settle_time) {
            transitionTo(next_state);
        }
    }
    
    void handleDone() {
        // Mission complete - could auto-restart here
    }
    
    void handleFault() {
        // Requires manual intervention
    }
    
    void sendWalkCommand(const Pose2D& target, double speed = -1) {
        if (speed < 0) speed = config_.cruise_speed;
        
        double heading_to_target = current_pose_.headingTo(target);
        double heading_error = angleDiff(heading_to_target, current_pose_.yaw);
        
        Velocity2D vel;
        
        // Forward velocity (reduce when heading is off)
        vel.vx = speed;
        if (std::abs(heading_error) > M_PI/4) {
            vel.vx *= 0.3;
        }
        
        // Lateral correction
        vel.vy = -speed * std::sin(heading_error) * 0.3;
        
        // Heading correction
        vel.wz = heading_error * 1.0;  // P-gain
        vel.wz = std::max(-config_.turn_speed, std::min(config_.turn_speed, vel.wz));
        
        motion_->sendVelocity(vel);
        last_cmd_time_ = std::chrono::steady_clock::now();
        
        // Set watchdog based on distance
        double dist = current_pose_.distanceTo(target);
        armWatchdog(dist / speed + 1.5);
    }
};

} // namespace uwb_nav
