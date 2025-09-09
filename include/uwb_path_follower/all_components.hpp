// include/uwb_path_follower/all_components.hpp
// Convenience header that includes all components for the main controller

#pragma once

// Core types and utilities
#include "types.hpp"
#include "config.hpp"

// Components
#include "uwb_receiver.hpp"
#include "uwb_converter.hpp" 
#include "state_estimator.hpp"
#include "path_tracker.hpp"
#include "data_logger.hpp"
#include "low_pass_filter.hpp"

// Note: The actual implementation classes are below
// This is a temporary solution until they're properly separated into individual headers

namespace uwb_path {

// ============= UWB Converter =============
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
    
    Pose2D uwbToBodyWorld(const UWBMeasurement& uwb, const IMUData& imu, 
                          double yaw_pred) {
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

// ============= State Estimator =============
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
    
    std::tuple<Pose2D, bool, bool> fuseComplementary(const Pose2D& pred,
                                                      const Pose2D& uwb_pose,
                                                      double dt) {
        Pose2D innov = {
            uwb_pose.x - pred.x,
            uwb_pose.y - pred.y,
            angleDiff(uwb_pose.yaw, pred.yaw)
        };
        
        if (!gateInnovation(innov)) {
            return {pred, false, false};
        }
        
        Pose2D bounded = rateLimitInnovation(innov, dt);
        bool clipped = (bounded.x != innov.x || bounded.y != innov.y || 
                        bounded.yaw != innov.yaw);
        
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
    void applyZUPT() { /* Tighten bias estimate */ }
};

// ============= Path Tracker =============
class PathTracker {
private:
    const Config& cfg;
    
public:
    enum Phase {
        FWD, SIT1, TURN, BACK, SIT2
    };
    
    Phase current_phase = FWD;
    double phase_timer = 0;
    Pose2D waypoint_start;
    
    PathTracker(const Config& config) : cfg(config) {}
    
    Twist2D computeStraightControl(const Pose2D& pose, const Pose2D& wp_start,
                                    const Pose2D& wp_end) {
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

// ============= Low Pass Filter =============
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

// ============= Data Logger =============
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