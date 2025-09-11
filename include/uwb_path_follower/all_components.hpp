// File: include/uwb_path_follower/all_components.hpp
#pragma once
#include <cmath>
#include <tuple>
#include <fstream>
#include <iomanip>
#include <array>
#include <string>
#include <chrono>
#include <iostream>

#include "uwb_path_follower/types.hpp"
#include "uwb_path_follower/config.hpp"
#include "uwb_path_follower/math2d.hpp"

namespace uwb_path {

// -------- UWBConverter --------
class UWBConverter {
  const Config& cfg;
public:
  explicit UWBConverter(const Config& c): cfg(c) {}

  Pose2D sphericalToBase(const UWBMeasurement& uwb) {
    const double rho = uwb.r * std::cos(uwb.alpha); // ground range
    return { rho * std::cos(uwb.beta), rho * std::sin(uwb.beta), 0 };
  }

  Pose2D baseToWorld(const Pose2D& pb) {
    const double c = std::cos(cfg.base_station.yaw);
    const double s = std::sin(cfg.base_station.yaw);
    return {
      c*pb.x - s*pb.y + cfg.base_station.x,
      s*pb.x + c*pb.y + cfg.base_station.y,
      pb.yaw
    };
  }

  // Correct Rx(roll), then Ry(pitch), then Rz(yaw_pred)
  Pose2D rotateLeverTiltAware(double roll, double pitch, double yaw_pred) {
    const double cx = std::cos(roll),  sx = std::sin(roll);
    const double cy = std::cos(pitch), sy = std::sin(pitch);
    const double cz = std::cos(yaw_pred), sz = std::sin(yaw_pred);

    const double lx = cfg.tag_mount.x;
    const double ly = cfg.tag_mount.y;
    const double lz = cfg.tag_mount.z;

    // Rx(roll)
    const double x1 = lx;
    const double y1 = cx*ly - sx*lz;
    const double z1 = sx*ly + cx*lz;

    // Ry(pitch)
    const double x2 = cy*x1 + sy*z1;
    const double y2 = y1;
    const double z2 = -sy*x1 + cy*z1;

    // Rz(yaw_pred)
    const double xw = cz*x2 - sz*y2;
    const double yw = sz*x2 + cz*y2;

    return {xw, yw, 0};
  }

  Pose2D uwbToBodyWorld(const UWBMeasurement& uwb, const IMUData& imu, double yaw_pred) {
    const Pose2D p_tag_world = baseToWorld(sphericalToBase(uwb));
    const double yaw_tag_world = wrapAngle(cfg.base_station.yaw + uwb.gamma);
    const double yaw_body = wrapAngle(yaw_tag_world + cfg.tag_mount.yaw_offset);
    const Pose2D lever_w = rotateLeverTiltAware(imu.roll, imu.pitch, yaw_pred);
    return { p_tag_world.x - lever_w.x, p_tag_world.y - lever_w.y, yaw_body };
  }
};

// -------- StateEstimator --------
class StateEstimator {
  Pose2D state{};
  double yaw_bias = 0;
  const Config& cfg;

  bool gate(const Pose2D& e) const {
    return std::fabs(e.x) < cfg.MAX_POS_INNOV &&
           std::fabs(e.y) < cfg.MAX_POS_INNOV &&
           std::fabs(e.yaw) < cfg.MAX_YAW_INNOV;
  }
  
  Pose2D rateLimit(const Pose2D& e, double dt) const {
    const double mp = cfg.MAX_POS_RATE * dt;
    const double my = cfg.MAX_YAW_RATE * dt;
    return { clamp(e.x,-mp,mp), clamp(e.y,-mp,mp), clamp(e.yaw,-my,my) };
  }

public:
  explicit StateEstimator(const Config& c): cfg(c) {}

  Pose2D propagate(const Twist2D& cmd, double gyro_z, double dt) {
    const double yaw_rate = gyro_z - yaw_bias;
    const double c = std::cos(state.yaw), s = std::sin(state.yaw);
    Pose2D pred = state;
    pred.x   += (c*cmd.vx - s*cmd.vy) * dt;
    pred.y   += (s*cmd.vx + c*cmd.vy) * dt;
    pred.yaw  = wrapAngle(pred.yaw + yaw_rate * dt);
    return pred;
  }

  std::tuple<Pose2D,bool,bool> fuseComplementary(const Pose2D& pred,
                                                  const Pose2D& meas,
                                                  double dt) {
    Pose2D innov{ meas.x - pred.x, meas.y - pred.y, angleDiff(meas.yaw, pred.yaw) };
    if (!gate(innov)) return {pred,false,false};
    
    Pose2D bounded = rateLimit(innov, dt);
    bool clipped = (bounded.x!=innov.x || bounded.y!=innov.y || bounded.yaw!=innov.yaw);
    
    Pose2D fused{
      pred.x + cfg.K_POS * bounded.x,
      pred.y + cfg.K_POS * bounded.y,
      wrapAngle(pred.yaw + cfg.K_YAW * bounded.yaw)
    };
    return {fused,true,clipped};
  }

  void applyZUPT(double gyro_z) { 
    yaw_bias += cfg.ZUPT_BIAS_GAIN * (gyro_z - yaw_bias); 
  }

  void setState(const Pose2D& p){ state = p; }
  Pose2D getState() const { return state; }
  void updateState(const Pose2D& p){ state = p; }
  double getYawBias() const { return yaw_bias; }
  void setYawBias(double b){ yaw_bias = b; }
};

// -------- PathTracker --------
class PathTracker {
  const Config& cfg;
public:
  enum Phase { FWD, SIT1, TURN, BACK, SIT2 };
  Phase  current_phase = FWD;
  double phase_timer = 0;
  Pose2D waypoint_start{};

  explicit PathTracker(const Config& c): cfg(c) {}

  Twist2D computeStraightControl(const Pose2D& pose,
                                 const Pose2D& wp_start,
                                 const Pose2D& wp_end) {
    const double dx = wp_end.x - wp_start.x;
    const double dy = wp_end.y - wp_start.y;
    const double path_yaw = std::atan2(dy, dx);

    const double tx = pose.x - wp_start.x;
    const double ty = pose.y - wp_start.y;
    const double cross = -tx*std::sin(path_yaw) + ty*std::cos(path_yaw);
    const double heading_err = angleDiff(path_yaw, pose.yaw);

    const double vy = -cfg.KP_CROSS * cross;
    const double wz =  cfg.KP_HEADING * heading_err;
    return {cfg.V_NOM, vy, wz};
  }

  bool reached(const Pose2D& pose, const Pose2D& wp) const {
    const double dx = pose.x - wp.x;
    const double dy = pose.y - wp.y;
    return std::sqrt(dx*dx + dy*dy) < cfg.WP_TOLERANCE;
  }

  Pose2D target() const {
    switch(current_phase) {
      case FWD:  return {cfg.PATH_LENGTH, 0, 0};
      case BACK: return {0, 0, 0};
      default:   return waypoint_start;
    }
  }

  void updatePhase(const Pose2D& pose, double dt) {
    phase_timer += dt;
    switch (current_phase) {
      case FWD:
        if (reached(pose, {cfg.PATH_LENGTH,0,0})) { 
          current_phase=SIT1; phase_timer=0; waypoint_start=pose; 
        }
        break;
      case SIT1:
        if (phase_timer > 2.0) { 
          current_phase=TURN; phase_timer=0; 
        }
        break;
      case TURN:
        if (phase_timer > 3.0) { 
          current_phase=BACK; phase_timer=0; 
        }
        break;
      case BACK:
        if (reached(pose, {0,0,0})) { 
          current_phase=SIT2; phase_timer=0; waypoint_start=pose; 
        }
        break;
      case SIT2:
        if (phase_timer > 2.0) { 
          current_phase=FWD; phase_timer=0; 
        }
        break;
    }
  }
};

// -------- LowPassFilter --------
class LowPassFilter {
  double value = 0;
  double tau;
public:
  explicit LowPassFilter(double time_constant): tau(time_constant) {}
  
  double update(double input, double dt) {
    const double alpha = 1.0 - std::exp(-dt / tau);
    value = alpha*input + (1 - alpha)*value;
    return value;
  }
  
  double get() const { return value; }
  void reset() { value = 0; }
};

// -------- DataLogger --------
class DataLogger {
  std::ofstream log_file;
  int line_count = 0;
public:
  explicit DataLogger(const std::string& filename) {
    log_file.open(filename);
    log_file << "timestamp,mode,phase,"
             << "x_est,y_est,yaw_est,x_pred,y_pred,yaw_pred,"
             << "uwb_r,uwb_beta,uwb_alpha,uwb_gamma,uwb_quality,uwb_latency,"
             << "uwb_body_x,uwb_body_y,uwb_body_yaw,"
             << "innov_x,innov_y,innov_yaw,innov_applied,innov_clipped,"
             << "shadow_innov_x,shadow_innov_y,shadow_innov_yaw,shadow_innov_applied,shadow_innov_clipped,"
             << "cmd_pre_vx,cmd_pre_vy,cmd_pre_wz,"
             << "cmd_post_vx,cmd_post_vy,cmd_post_wz,"
             << "gyro_z,roll,pitch,yaw_bias,yaw_dot_used,"
             << "foot_force_0,foot_force_1,foot_force_2,foot_force_3,"
             << "zupt_active,body_height,gait_type\n";
  }

  void log(double timestamp, const std::string& mode, int phase,
           const Pose2D& est, const Pose2D& pred,
           const UWBMeasurement& uwb, double uwb_latency,
           const Pose2D& uwb_body_pose,
           const Pose2D& innov, bool innov_applied, bool innov_clipped,
           const Pose2D& shadow_innov, bool shadow_applied, bool shadow_clipped,
           const Twist2D& cmd_pre, const Twist2D& cmd_post,
           const IMUData& imu, double yaw_bias, double yaw_dot_used,
           const std::array<double,4>& foot_forces,
           bool zupt, double body_height, int gait_type) {

    log_file << std::fixed << std::setprecision(6)
             << timestamp << "," << mode << "," << phase << ","
             << est.x << "," << est.y << "," << est.yaw << ","
             << pred.x << "," << pred.y << "," << pred.yaw << ","
             << uwb.r << "," << uwb.beta << "," << uwb.alpha << ","
             << uwb.gamma << "," << uwb.quality << "," << uwb_latency << ","
             << uwb_body_pose.x << "," << uwb_body_pose.y << "," << uwb_body_pose.yaw << ","
             << innov.x << "," << innov.y << "," << innov.yaw << ","
             << (innov_applied?1:0) << "," << (innov_clipped?1:0) << ","
             << shadow_innov.x << "," << shadow_innov.y << "," << shadow_innov.yaw << ","
             << (shadow_applied?1:0) << "," << (shadow_clipped?1:0) << ","
             << cmd_pre.vx << "," << cmd_pre.vy << "," << cmd_pre.wz << ","
             << cmd_post.vx << "," << cmd_post.vy << "," << cmd_post.wz << ","
             << imu.gyro_z << "," << imu.roll << "," << imu.pitch << ","
             << yaw_bias << "," << yaw_dot_used << ","
             << foot_forces[0] << "," << foot_forces[1] << ","
             << foot_forces[2] << "," << foot_forces[3] << ","
             << (zupt?1:0) << "," << body_height << "," << gait_type << "\n";

    if (++line_count % 100 == 0) log_file.flush();
  }

  ~DataLogger() { 
    if (log_file.is_open()) log_file.close(); 
  }
};

} // namespace uwb_path