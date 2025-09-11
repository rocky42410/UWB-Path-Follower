#pragma once
#include <cmath>

namespace uwb_path {

struct Vec2 {
    double x{}, y{};
    
    Vec2() = default;
    Vec2(double x_, double y_) : x(x_), y(y_) {}
    
    Vec2 operator+(Vec2 o) const { return {x+o.x, y+o.y}; }
    Vec2 operator-(Vec2 o) const { return {x-o.x, y-o.y}; }
    Vec2 operator*(double s) const { return {x*s, y*s}; }
    
    Vec2& operator+=(Vec2 o) { x+=o.x; y+=o.y; return *this; }
    Vec2& operator-=(Vec2 o) { x-=o.x; y-=o.y; return *this; }
    Vec2& operator*=(double s) { x*=s; y*=s; return *this; }
};

inline double norm(Vec2 v) { 
    return std::hypot(v.x, v.y); 
}

inline double atan2deg(double y, double x) { 
    return std::atan2(y, x) * 180.0 / M_PI; 
}

inline double rad(double d) { 
    return d * M_PI / 180.0; 
}

inline double deg(double r) { 
    return r * 180.0 / M_PI; 
}

inline double wrapPi(double a) {
    while(a >  M_PI) a -= 2*M_PI;
    while(a < -M_PI) a += 2*M_PI;
    return a;
}

// Additional helper for distance between two points
inline double distance(Vec2 a, Vec2 b) {
    return norm(b - a);
}

} // namespace uwb_path