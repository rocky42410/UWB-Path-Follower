// test/test_estimator.cpp
// Unit tests for StateEstimator component

#include <iostream>
#include <cmath>
#include <cassert>
#include <iomanip>

// Inline the necessary structures and classes for testing
struct Config {
    double K_POS = 0.1;
    double K_YAW = 0.15;
    double MAX_POS_INNOV = 0.5;
    double MAX_YAW_INNOV = 0.3;
    double MAX_POS_RATE = 0.2;
    double MAX_YAW_RATE = 0.5;
};

struct Pose2D {
    double x = 0, y = 0, yaw = 0;
    
    Pose2D operator-(const Pose2D& other) const {
        return {x - other.x, y - other.y, yaw - other.yaw};
    }
};

struct Twist2D {
    double vx = 0, vy = 0, wz = 0;
};

double wrapAngle(double angle) {
    while (angle > M_PI) angle -= 2*M_PI;
    while (angle < -M_PI) angle += 2*M_PI;
    return angle;
}

double angleDiff(double a, double b) {
    return wrapAngle(a - b);
}

double clamp(double val, double min, double max) {
    return std::max(min, std::min(max, val));
}

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
    void setYawBias(double bias) { yaw_bias = bias; }
};

// Test functions
void test_propagation() {
    std::cout << "Testing state propagation..." << std::endl;
    
    Config cfg;
    StateEstimator estimator(cfg);
    
    // Test 1: Forward motion
    estimator.setState({0, 0, 0});
    Twist2D cmd = {1.0, 0, 0};  // 1 m/s forward
    double gyro_z = 0;
    double dt = 0.1;
    
    Pose2D pred = estimator.propagate(cmd, gyro_z, dt);
    assert(fabs(pred.x - 0.1) < 1e-6);
    assert(fabs(pred.y) < 1e-6);
    assert(fabs(pred.yaw) < 1e-6);
    std::cout << "  ✓ Forward propagation" << std::endl;
    
    // Test 2: Rotation
    estimator.setState({0, 0, 0});
    cmd = {0, 0, 0};
    gyro_z = 1.0;  // 1 rad/s
    
    pred = estimator.propagate(cmd, gyro_z, dt);
    assert(fabs(pred.x) < 1e-6);
    assert(fabs(pred.y) < 1e-6);
    assert(fabs(pred.yaw - 0.1) < 1e-6);
    std::cout << "  ✓ Rotation propagation" << std::endl;
    
    // Test 3: Combined motion
    estimator.setState({0, 0, M_PI/2});  // Facing +Y
    cmd = {1.0, 0, 0};  // Forward in body frame
    gyro_z = 0;
    
    pred = estimator.propagate(cmd, gyro_z, dt);
    assert(fabs(pred.x) < 1e-6);
    assert(fabs(pred.y - 0.1) < 1e-6);  // Should move in +Y
    std::cout << "  ✓ Combined motion" << std::endl;
}

void test_fusion() {
    std::cout << "Testing UWB fusion..." << std::endl;
    
    Config cfg;
    StateEstimator estimator(cfg);
    
    // Test 1: Small innovation (should be accepted)
    Pose2D pred = {1.0, 1.0, 0.1};
    Pose2D uwb = {1.05, 1.03, 0.12};  // Small error
    double dt = 0.02;
    
    auto [fused, ok, clipped] = estimator.fuseComplementary(pred, uwb, dt);
    assert(ok == true);
    assert(clipped == false);
    assert(fabs(fused.x - 1.005) < 1e-3);  // Should move 10% toward UWB
    std::cout << "  ✓ Small innovation accepted" << std::endl;
    
    // Test 2: Large innovation (should be gated)
    uwb = {2.0, 2.0, 1.0};  // Large error
    auto [fused2, ok2, clipped2] = estimator.fuseComplementary(pred, uwb, dt);
    assert(ok2 == false);  // Should be rejected
    assert(fused2.x == pred.x);  // No change
    std::cout << "  ✓ Large innovation rejected" << std::endl;
    
    // Test 3: Rate limiting
    uwb = {1.3, 1.3, 0.2};  // Medium error
    auto [fused3, ok3, clipped3] = estimator.fuseComplementary(pred, uwb, dt);
    assert(ok3 == true);
    assert(clipped3 == true);  // Should be rate-limited
    
    // Check that change is limited to max rate * dt
    double max_change = cfg.MAX_POS_RATE * dt;
    assert(fabs(fused3.x - pred.x) <= cfg.K_POS * max_change * 1.01);
    std::cout << "  ✓ Rate limiting applied" << std::endl;
}

void test_wrap_angle() {
    std::cout << "Testing angle wrapping..." << std::endl;
    
    assert(fabs(wrapAngle(M_PI + 0.1) - (-M_PI + 0.1)) < 1e-6);
    assert(fabs(wrapAngle(-M_PI - 0.1) - (M_PI - 0.1)) < 1e-6);
    assert(fabs(wrapAngle(2 * M_PI) - 0) < 1e-6);
    
    assert(fabs(angleDiff(0.1, -0.1) - 0.2) < 1e-6);
    assert(fabs(angleDiff(-M_PI + 0.1, M_PI - 0.1) - 0.2) < 1e-6);
    
    std::cout << "  ✓ Angle wrapping correct" << std::endl;
}

void test_yaw_bias() {
    std::cout << "Testing yaw bias compensation..." << std::endl;
    
    Config cfg;
    StateEstimator estimator(cfg);
    
    // Set a bias
    estimator.setState({0, 0, 0});
    estimator.setYawBias(0.1);  // 0.1 rad/s bias
    
    Twist2D cmd = {0, 0, 0};
    double gyro_z = 0.5;  // Measured gyro
    double dt = 0.1;
    
    Pose2D pred = estimator.propagate(cmd, gyro_z, dt);
    // Should rotate by (0.5 - 0.1) * 0.1 = 0.04 rad
    assert(fabs(pred.yaw - 0.04) < 1e-6);
    
    std::cout << "  ✓ Yaw bias compensation" << std::endl;
}

void test_complete_cycle() {
    std::cout << "Testing complete estimation cycle..." << std::endl;
    
    Config cfg;
    StateEstimator estimator(cfg);
    estimator.setState({0, 0, 0});
    
    // Simulate 10 steps
    double dt = 0.02;
    for (int i = 0; i < 10; i++) {
        // Command
        Twist2D cmd = {0.3, 0.01 * sin(i * 0.1), 0.05};
        double gyro_z = 0.05 + 0.01 * sin(i * 0.2);
        
        // Propagate
        Pose2D pred = estimator.propagate(cmd, gyro_z, dt);
        
        // Simulate UWB (with some noise)
        Pose2D uwb_pose = pred;
        uwb_pose.x += 0.02 * sin(i * 0.3);
        uwb_pose.y += 0.01 * cos(i * 0.4);
        uwb_pose.yaw += 0.01 * sin(i * 0.5);
        
        // Fuse
        auto [fused, ok, clipped] = estimator.fuseComplementary(pred, uwb_pose, dt);
        
        // Update state
        estimator.updateState(fused);
    }
    
    Pose2D final_state = estimator.getState();
    std::cout << "  Final pose: x=" << final_state.x 
              << ", y=" << final_state.y 
              << ", yaw=" << final_state.yaw << std::endl;
    
    // Should have moved forward and turned slightly
    assert(final_state.x > 0);
    assert(fabs(final_state.yaw) > 0);
    
    std::cout << "  ✓ Complete cycle successful" << std::endl;
}

int main() {
    std::cout << "\n=== State Estimator Unit Tests ===" << std::endl;
    std::cout << std::fixed << std::setprecision(6);
    
    try {
        test_wrap_angle();
        test_propagation();
        test_fusion();
        test_yaw_bias();
        test_complete_cycle();
        
        std::cout << "\n✅ All tests passed!" << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "\n❌ Test failed: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}