// include/uwb_path_follower/types.hpp
#pragma once

#include <array>
#include <cmath>

namespace uwb_path {

// 2D pose representation
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

// 2D velocity command
struct Twist2D {
    double vx = 0, vy = 0, wz = 0;
};

// UWB spherical measurement
struct UWBMeasurement {
    double r = 0;        // range (m)
    double beta = 0;     // azimuth (rad)
    double alpha = 0;    // elevation (rad)
    double gamma = 0;    // tag yaw (rad)
    double timestamp = 0;
    double quality = 0;
};

// IMU data
struct IMUData {
    double roll = 0, pitch = 0;  // rad
    double gyro_z = 0;            // rad/s
    std::array<double, 4> quaternion = {1,0,0,0}; // w,x,y,z
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

inline double clamp(double val, double min, double max) {
    return std::max(min, std::min(max, val));
}

} // namespace uwb_path