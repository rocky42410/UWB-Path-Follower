// File: include/uwb_path_follower/config.hpp
#pragma once
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>

namespace uwb_path {

struct Config {
  struct { double x=0, y=0, z=0, yaw=0; } base_station;
  struct { double x=0, y=0.15, z=0, yaw_offset=0; } tag_mount;

  // Fusion gains
  double K_POS = 0.1;
  double K_YAW = 0.15;

  // Innovation gates
  double MAX_POS_INNOV = 0.5;
  double MAX_YAW_INNOV = 0.3;
  double MAX_POS_RATE = 0.2;   // m/s equivalent
  double MAX_YAW_RATE = 0.5;   // rad/s equivalent

  // Spherical gates
  double UWB_R_GATE = 1.0;
  double UWB_BETA_GATE = 0.5;
  double UWB_ALPHA_GATE = 0.3;
  double UWB_GAMMA_GATE = 0.5;

  // Control
  double V_NOMINAL = 0.3;
  double KP_CROSS = 1.0;
  double KP_HEADING = 0.8;
  double LP_TAU = 0.25;

  // Path
  double PATH_LENGTH = 5.0;
  double WP_TOLERANCE = 0.2;

  // UWB quality & latency
  double UWB_MIN_QUALITY = 0.7;
  double UWB_MAX_LATENCY = 0.1; // s

  // ZUPT thresholds
  double ZUPT_FOOT_THRESH = 10.0;  // N total
  double ZUPT_VEL_THRESH = 0.05;   // m/s (commanded)
  double ZUPT_GYRO_THRESH = 0.05;  // rad/s
  double ZUPT_BIAS_GAIN = 0.02;

  bool loadFromFile(const std::string& filename) {
    std::ifstream f(filename);
    if (!f.is_open()) {
      std::cerr << "[Config] File not found: " << filename << ", using defaults\n";
      return false;
    }
    std::string line;
    while (std::getline(f, line)) {
      if (line.empty() || line[0]=='#' || line[0]==';') continue;
      auto eq = line.find('=');
      if (eq == std::string::npos) continue;
      std::string key = line.substr(0, eq);
      std::string val = line.substr(eq+1);
      auto trim = [](std::string& s) {
        s.erase(0, s.find_first_not_of(" \t"));
        s.erase(s.find_last_not_of(" \t")+1);
      };
      trim(key); trim(val);
      try {
        if (key=="base_x") base_station.x = std::stod(val);
        else if (key=="base_y") base_station.y = std::stod(val);
        else if (key=="base_z") base_station.z = std::stod(val);
        else if (key=="base_yaw") base_station.yaw = std::stod(val);
        else if (key=="lever_x") tag_mount.x = std::stod(val);
        else if (key=="lever_y") tag_mount.y = std::stod(val);
        else if (key=="lever_z") tag_mount.z = std::stod(val);
        else if (key=="tag_yaw_offset") tag_mount.yaw_offset = std::stod(val);
        else if (key=="k_pos") K_POS = std::stod(val);
        else if (key=="k_yaw") K_YAW = std::stod(val);
        else if (key=="max_pos_innov") MAX_POS_INNOV = std::stod(val);
        else if (key=="max_yaw_innov") MAX_YAW_INNOV = std::stod(val);
        else if (key=="max_pos_rate") MAX_POS_RATE = std::stod(val);
        else if (key=="max_yaw_rate") MAX_YAW_RATE = std::stod(val);
        else if (key=="uwb_r_gate") UWB_R_GATE = std::stod(val);
        else if (key=="uwb_beta_gate") UWB_BETA_GATE = std::stod(val);
        else if (key=="uwb_alpha_gate") UWB_ALPHA_GATE = std::stod(val);
        else if (key=="uwb_gamma_gate") UWB_GAMMA_GATE = std::stod(val);
        else if (key=="v_nominal") V_NOMINAL = std::stod(val);
        else if (key=="kp_cross") KP_CROSS = std::stod(val);
        else if (key=="kp_heading") KP_HEADING = std::stod(val);
        else if (key=="lp_tau") LP_TAU = std::stod(val);
        else if (key=="path_length") PATH_LENGTH = std::stod(val);
        else if (key=="wp_tolerance") WP_TOLERANCE = std::stod(val);
        else if (key=="uwb_min_quality") UWB_MIN_QUALITY = std::stod(val);
        else if (key=="uwb_max_latency") UWB_MAX_LATENCY = std::stod(val);
        else if (key=="zupt_foot_thresh") ZUPT_FOOT_THRESH = std::stod(val);
        else if (key=="zupt_vel_thresh") ZUPT_VEL_THRESH = std::stod(val);
        else if (key=="zupt_gyro_thresh") ZUPT_GYRO_THRESH = std::stod(val);
        else if (key=="zupt_bias_gain") ZUPT_BIAS_GAIN = std::stod(val);
      } catch (const std::exception& e) {
        std::cerr << "[Config] Error parsing " << key << ": " << e.what() << "\n";
      }
    }
    std::cout << "[Config] Loaded: " << filename << "\n";
    return true;
  }
};

} // namespace uwb_path
