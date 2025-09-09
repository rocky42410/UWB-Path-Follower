// include/uwb_path_follower/config.hpp
#pragma once

#include <string>
#include <iostream>
#include <fstream>
// YAML support is optional - uncomment if yaml-cpp is installed
// #include <yaml-cpp/yaml.h>

namespace uwb_path {

struct Config {
    // Base station extrinsics
    struct {
        double x = 0.0, y = 0.0, z = 0.0;  // BASE_POS_W
        double yaw = 0.0;                   // BASE_YAW_W (rad)
    } base_station;
    
    // Tag mounting on robot body
    struct {
        double x = 0.0, y = 0.15, z = 0.0; // LEVER_BODY (m)
        double yaw_offset = 0.0;            // YAW_TAG2BODY (rad)
    } tag_mount;
    
    // Fusion gains
    double K_POS = 0.1;    // Position correction gain
    double K_YAW = 0.15;   // Yaw correction gain
    
    // Innovation gates (m, rad)
    double MAX_POS_INNOV = 0.5;
    double MAX_YAW_INNOV = 0.3;
    
    // Rate limits (m/s, rad/s)
    double MAX_POS_RATE = 0.2;
    double MAX_YAW_RATE = 0.5;
    
    // Control params
    double V_NOMINAL = 0.3;      // m/s forward speed
    double KP_CROSS = 1.0;        // Cross-track P gain
    double KP_HEADING = 0.8;      // Heading P gain
    double LP_TAU = 0.25;         // Low-pass time constant
    
    // Path
    double PATH_LENGTH = 5.0;     // m
    double WP_TOLERANCE = 0.2;    // m
    
    // UWB quality
    double UWB_MIN_QUALITY = 0.7;
    double UWB_MAX_LATENCY = 0.1; // s
    
    // ZUPT thresholds
    double ZUPT_FOOT_THRESH = 10.0;  // N
    double ZUPT_VEL_THRESH = 0.05;   // m/s
    double ZUPT_GYRO_THRESH = 0.05;  // rad/s
    
    // Load from YAML file (optional implementation)
    bool loadFromFile(const std::string& filename) {
        try {
            // Basic file loading - implement YAML parsing if needed
            std::ifstream file(filename);
            if (!file.is_open()) {
                std::cerr << "Config file not found: " << filename << std::endl;
                return false;
            }
            // TODO: Parse YAML and update values
            std::cout << "Config loaded from: " << filename << std::endl;
            return true;
        } catch (...) {
            return false;
        }
    }
};

} // namespace uwb_path