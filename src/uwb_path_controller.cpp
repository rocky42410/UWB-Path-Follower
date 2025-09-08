// uwb_path_controller.cpp
// Main controller for UWB-fused path following on Unitree Go2
// Implements single-loop out-and-back with sits/turns
// Corrections applied only in state estimator, not commands

#include <cmath>
#include <thread>
#include <chrono>
#include <atomic>
#include <mutex>
#include <fstream>
#include <iomanip>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/robot/client/sport_client.hpp>

using namespace unitree::robot;
using namespace unitree::common;

// ============= Configuration =============
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
};

// ============= Data Structures =============
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

struct Twist2D {
    double vx = 0, vy = 0, wz = 0;
};

struct UWBMeasurement {
    double r = 0;        // range (m)
    double beta = 0;     // azimuth (rad)
    double alpha = 0;    // elevation (rad)
    double gamma = 0;    // tag yaw (rad)
    double timestamp = 0;
    double quality = 0;
};

struct IMUData {
    double roll = 0, pitch = 0;  // rad
    double gyro_z = 0;            // rad/s
    std::array<double, 4> quaternion = {1,0,0,0}; // w,x,y,z
};

// ============= Utility Functions =============
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

// ============= UWB Converter =============
class UWBConverter {
private:
    const Config& cfg;
    
public:
    UWBConverter(const Config& config) : cfg(config) {}
    
    // Convert spherical UWB to base frame XYZ
    Pose2D sphericalToBase(const UWBMeasurement& uwb) {
        double rho = uwb.r * cos(uwb.alpha);  // planar range
        double x_b = rho * cos(uwb.beta);
        double y_b = rho * sin(uwb.beta);
        double z_b = uwb.r * sin(uwb.alpha);
        return {x_b, y_b, 0};  // ignore z for 2D
    }
    
    // Convert base frame to world frame
    Pose2D baseToWorld(const Pose2D& p_base) {
        double c = cos(cfg.base_station.yaw);
        double s = sin(cfg.base_station.yaw);
        return {
            c * p_base.x - s * p_base.y + cfg.base_station.x,
            s * p_base.x + c * p_base.y + cfg.base_station.y,
            p_base.yaw
        };
    }
    
    // Get tag yaw in world frame
    double tagYawWorld(const UWBMeasurement& uwb) {
        return wrapAngle(cfg.base_station.yaw + uwb.gamma);
    }
    
    // Rotate lever arm by robot orientation
    Pose2D rotateLever(double roll, double pitch, double yaw_pred) {
        // Simplified for 2D case, ignoring roll/pitch for lever
        double c = cos(yaw_pred);
        double s = sin(yaw_pred);
        return {
            c * cfg.tag_mount.x - s * cfg.tag_mount.y,
            s * cfg.tag_mount.x + c * cfg.tag_mount.y,
            0
        };
    }
    
    // Full conversion: UWB spherical to body pose in world
    Pose2D uwbToBodyWorld(const UWBMeasurement& uwb, const IMUData& imu, 
                          double yaw_pred) {
        // Tag position in base frame
        Pose2D p_tag_base = sphericalToBase(uwb);
        
        // Tag position in world frame
        Pose2D p_tag_world = baseToWorld(p_tag_base);
        
        // Tag yaw to body yaw
        double yaw_tag = tagYawWorld(uwb);
        double yaw_body = wrapAngle(yaw_tag + cfg.tag_mount.yaw_offset);
        
        // Subtract rotated lever to get body position
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
    
    // Innovation gating
    bool gateInnovation(const Pose2D& innov) {
        return (fabs(innov.x) < cfg.MAX_POS_INNOV &&
                fabs(innov.y) < cfg.MAX_POS_INNOV &&
                fabs(innov.yaw) < cfg.MAX_YAW_INNOV);
    }
    
    // Rate limiting
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
    
    // Propagate state using commanded twist and gyro
    Pose2D propagate(const Twist2D& cmd, double gyro_z, double dt) {
        // Use gyro for yaw rate (with bias correction)
        double yaw_rate = gyro_z - yaw_bias;
        
        // Dead reckoning with body twist
        double c = cos(state.yaw);
        double s = sin(state.yaw);
        
        Pose2D pred = state;
        pred.x += (c * cmd.vx - s * cmd.vy) * dt;
        pred.y += (s * cmd.vx + c * cmd.vy) * dt;
        pred.yaw = wrapAngle(pred.yaw + yaw_rate * dt);
        
        return pred;
    }
    
    // Fuse UWB measurement
    std::tuple<Pose2D, bool, bool> fuseComplementary(const Pose2D& pred,
                                                      const Pose2D& uwb_pose,
                                                      double dt) {
        // Compute innovation
        Pose2D innov = {
            uwb_pose.x - pred.x,
            uwb_pose.y - pred.y,
            angleDiff(uwb_pose.yaw, pred.yaw)
        };
        
        // Gate check
        if (!gateInnovation(innov)) {
            return {pred, false, false};
        }
        
        // Rate limit
        Pose2D bounded = rateLimitInnovation(innov, dt);
        bool clipped = (bounded.x != innov.x || bounded.y != innov.y || 
                        bounded.yaw != innov.yaw);
        
        // Apply correction
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
    
    // ZUPT update
    void applyZUPT() {
        // During sits, tighten bias estimate if needed
        // For now, just maintain current bias
    }
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
    
    // Compute control for straight line segment
    Twist2D computeStraightControl(const Pose2D& pose, const Pose2D& wp_start,
                                    const Pose2D& wp_end) {
        // Path vector
        double dx = wp_end.x - wp_start.x;
        double dy = wp_end.y - wp_start.y;
        double path_yaw = atan2(dy, dx);
        
        // Cross-track error (perpendicular distance to line)
        double to_robot_x = pose.x - wp_start.x;
        double to_robot_y = pose.y - wp_start.y;
        double along = to_robot_x * cos(path_yaw) + to_robot_y * sin(path_yaw);
        double cross = -to_robot_x * sin(path_yaw) + to_robot_y * cos(path_yaw);
        
        // Heading error
        double heading_err = angleDiff(path_yaw, pose.yaw);
        
        // PD control
        double vy_raw = -cfg.KP_CROSS * cross;
        double wz_raw = cfg.KP_HEADING * heading_err;
        
        return {cfg.V_NOMINAL, vy_raw, wz_raw};
    }
    
    // Check if reached waypoint
    bool reachedWaypoint(const Pose2D& pose, const Pose2D& wp) {
        double dist = sqrt(pow(pose.x - wp.x, 2) + pow(pose.y - wp.y, 2));
        return dist < cfg.WP_TOLERANCE;
    }
    
    // Get current waypoint target
    Pose2D getCurrentTarget() {
        switch (current_phase) {
            case FWD:  return {cfg.PATH_LENGTH, 0, 0};
            case BACK: return {0, 0, 0};
            default:   return waypoint_start;
        }
    }
    
    // Update phase machine
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
        // Write header
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

// ============= Main Controller =============
class UWBPathController {
private:
    Config cfg;
    SportClient sport_client;
    ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_> sport_subscriber;
    
    StateEstimator estimator;
    UWBConverter uwb_converter;
    PathTracker path_tracker;
    DataLogger logger;
    
    LowPassFilter lp_vy;
    LowPassFilter lp_wz;
    
    std::atomic<bool> running{true};
    std::mutex data_mutex;
    
    // Latest sensor data
    IMUData latest_imu;
    std::array<double, 4> latest_foot_forces{};
    UWBMeasurement latest_uwb;
    bool uwb_available = false;
    
    Twist2D last_cmd;
    std::string mode = "SMOOTH_LOOP";
    
public:
    UWBPathController() : 
        cfg(),
        estimator(cfg),
        uwb_converter(cfg),
        path_tracker(cfg),
        logger("uwb_path_log.csv"),
        lp_vy(cfg.LP_TAU),
        lp_wz(cfg.LP_TAU) {
        
        // Initialize sport client
        sport_client.Init();
        
        // Setup DDS subscriber
        sport_subscriber = std::make_shared<ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>>(
            "rt/sportmodestate");
        sport_subscriber->InitChannel([this](const auto& msg) { handleSportState(msg); });
    }
    
    void handleSportState(const unitree_go::msg::dds_::SportModeState_& msg) {
        std::lock_guard<std::mutex> lock(data_mutex);
        
        // Extract IMU data
        latest_imu.gyro_z = msg.imu_state().gyroscope()[2];
        latest_imu.roll = msg.imu_state().rpy()[0];
        latest_imu.pitch = msg.imu_state().rpy()[1];
        
        // Extract foot forces
        for (int i = 0; i < 4; i++) {
            latest_foot_forces[i] = msg.foot_force()[i];
        }
    }
    
    void simulateUWB() {
        // Simulate UWB measurements for testing
        static double t = 0;
        t += 0.02;
        
        latest_uwb.r = 5.0 + 0.1 * sin(t);
        latest_uwb.beta = 0.1 * sin(0.5 * t);
        latest_uwb.alpha = 0.05 * sin(0.3 * t);
        latest_uwb.gamma = estimator.getState().yaw + 0.05 * sin(t);
        latest_uwb.quality = 0.9;
        latest_uwb.timestamp = t;
        uwb_available = true;
    }
    
    bool checkZUPT() {
        // Check foot forces
        double total_force = 0;
        for (auto f : latest_foot_forces) total_force += f;
        
        // Check velocity and gyro
        bool low_motion = (fabs(last_cmd.vx) < cfg.ZUPT_VEL_THRESH &&
                           fabs(last_cmd.vy) < cfg.ZUPT_VEL_THRESH &&
                           fabs(latest_imu.gyro_z) < cfg.ZUPT_GYRO_THRESH);
        
        return (total_force > cfg.ZUPT_FOOT_THRESH && low_motion);
    }
    
    void run() {
        const double dt = 0.02;  // 50Hz
        auto next_time = std::chrono::steady_clock::now();
        
        while (running) {
            next_time += std::chrono::milliseconds(20);
            
            // Get sensor snapshot
            IMUData imu;
            std::array<double, 4> foot_forces;
            {
                std::lock_guard<std::mutex> lock(data_mutex);
                imu = latest_imu;
                foot_forces = latest_foot_forces;
            }
            
            // Simulate UWB for testing
            simulateUWB();
            
            // === State Prediction ===
            Pose2D pred_pose = estimator.propagate(last_cmd, imu.gyro_z, dt);
            
            // === UWB Update ===
            Pose2D uwb_body_pose;
            bool have_uwb = false;
            Pose2D innov;
            bool innov_ok = false;
            bool innov_clipped = false;
            
            if (uwb_available && latest_uwb.quality >= cfg.UWB_MIN_QUALITY) {
                double latency = 0.01;  // Simulated latency
                
                // Convert UWB to body pose
                uwb_body_pose = uwb_converter.uwbToBodyWorld(latest_uwb, imu, 
                                                             estimator.getState().yaw);
                
                // Compensate for latency (simple forward prediction)
                uwb_body_pose.x += last_cmd.vx * latency;
                uwb_body_pose.y += last_cmd.vy * latency;
                
                have_uwb = true;
            }
            
            // === Fusion ===
            Pose2D est_pose = pred_pose;
            if (mode == "SMOOTH_LOOP" && have_uwb) {
                auto [fused, ok, clipped] = estimator.fuseComplementary(pred_pose, 
                                                                        uwb_body_pose, dt);
                est_pose = fused;
                innov_ok = ok;
                innov_clipped = clipped;
                innov = uwb_body_pose - pred_pose;
            }
            estimator.updateState(est_pose);
            
            // === Path Following ===
            Twist2D raw_cmd;
            
            // Update phase machine
            path_tracker.updatePhase(est_pose, dt);
            
            // Compute control based on phase
            switch (path_tracker.current_phase) {
                case PathTracker::FWD:
                case PathTracker::BACK: {
                    Pose2D target = path_tracker.getCurrentTarget();
                    Pose2D start = (path_tracker.current_phase == PathTracker::FWD) ? 
                                   Pose2D{0,0,0} : Pose2D{cfg.PATH_LENGTH,0,0};
                    raw_cmd = path_tracker.computeStraightControl(est_pose, start, target);
                    break;
                }
                
                case PathTracker::TURN:
                    raw_cmd = {0, 0, 0.3};  // Turn in place
                    break;
                    
                case PathTracker::SIT1:
                case PathTracker::SIT2:
                    raw_cmd = {0, 0, 0};  // Stop
                    break;
            }
            
            // === Low-pass filtering ===
            double vy_filtered = lp_vy.update(raw_cmd.vy, dt);
            double wz_filtered = lp_wz.update(raw_cmd.wz, dt);
            
            // === Saturation ===
            Twist2D cmd = {
                raw_cmd.vx,
                clamp(vy_filtered, -0.1, 0.1),
                clamp(wz_filtered, -0.5, 0.5)
            };
            
            // === Send command ===
            sport_client.Move(cmd.vx, cmd.vy, cmd.wz);
            last_cmd = cmd;
            
            // === ZUPT check ===
            bool zupt_active = false;
            if ((path_tracker.current_phase == PathTracker::SIT1 || 
                 path_tracker.current_phase == PathTracker::SIT2) && checkZUPT()) {
                estimator.applyZUPT();
                zupt_active = true;
            }
            
            // === Logging ===
            logger.log(std::chrono::duration<double>(
                      std::chrono::steady_clock::now().time_since_epoch()).count(),
                      mode, path_tracker.current_phase,
                      est_pose, pred_pose, latest_uwb, innov,
                      innov_ok, innov_clipped, cmd, imu,
                      foot_forces, zupt_active);
            
            // Sleep until next cycle
            std::this_thread::sleep_until(next_time);
        }
    }
    
    void stop() {
        running = false;
        sport_client.Move(0, 0, 0);
    }
};

// ============= Main =============
int main(int argc, char* argv[]) {
    std::cout << "Starting UWB Path Follower for Go2\n";
    std::cout << "Mode: SMOOTH_LOOP (continuous fusion)\n";
    std::cout << "Press Ctrl+C to stop\n";
    
    UWBPathController controller;
    
    // Setup signal handler
    std::signal(SIGINT, [](int) {
        std::cout << "\nShutting down...\n";
        std::exit(0);
    });
    
    // Run controller
    controller.run();
    
    return 0;
}