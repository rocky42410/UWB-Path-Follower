// File: include/uwb_path_follower/types.hpp
#pragma once
#include <array>
#include <algorithm>
#include <cmath>

namespace uwb_path {

// Single-source portable constants
inline constexpr double PI = 3.141592653589793238462643383279502884;
inline constexpr double HALF_PI = PI * 0.5;

struct Pose2D {
  double x = 0, y = 0, yaw = 0;
  Pose2D() = default;
  Pose2D(double x_, double y_, double yaw_) : x(x_), y(y_), yaw(yaw_) {}
  
  Pose2D operator+(const Pose2D& o) const { return {x+o.x, y+o.y, yaw+o.yaw}; }
  Pose2D operator-(const Pose2D& o) const { return {x-o.x, y-o.y, yaw-o.yaw}; }
  Pose2D operator*(double s) const { return {x*s, y*s, yaw*s}; }
};

struct Twist2D {
  double vx = 0, vy = 0, wz = 0;
  Twist2D() = default;
  Twist2D(double vx_, double vy_, double wz_) : vx(vx_), vy(vy_), wz(wz_) {}
};

struct UWBMeasurement {
  double r = 0;        // range (m)
  double beta = 0;     // azimuth (rad)
  double alpha = 0;    // elevation (rad)
  double gamma = 0;    // tag yaw (rad)
  double timestamp = 0; // epoch seconds
  double quality = 0;   // 0..1
};

struct IMUData {
  double roll = 0, pitch = 0;   // rad
  double gyro_z = 0;            // rad/s
  std::array<double,4> quaternion{1,0,0,0}; // w,x,y,z
};

inline double wrapAngle(double a) {
  while (a > PI) a -= 2*PI;
  while (a < -PI) a += 2*PI;
  return a;
}

inline double angleDiff(double a, double b) { 
  return wrapAngle(a - b); 
}

inline double clamp(double v, double lo, double hi) {
  return std::max(lo, std::min(hi, v));
}

} // namespace uwb_path