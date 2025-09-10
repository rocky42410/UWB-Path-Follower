// File: include/uwb_path_follower/uwb_receiver.hpp
#pragma once
#include "types.hpp"

#include <unitree/idl/go2/UwbState_.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include <mutex>
#include <atomic>
#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <type_traits>
#include <utility>
#include <limits>
#include <algorithm>
#include <vector>
#include <cmath>

namespace uwb_path {

// SFINAE helpers for timestamp extraction
namespace detail {

template <class T, class = void>
struct has_timestamp_us : std::false_type {};

template <class T>
struct has_timestamp_us<T,
  std::void_t<decltype(std::declval<T&>().timestamp_us())>
> : std::true_type {};

template <class T, class = void>
struct has_stamp_sec_nsec : std::false_type {};

template <class T>
struct has_stamp_sec_nsec<T,
  std::void_t<
    decltype(std::declval<T&>().stamp().sec()),
    decltype(std::declval<T&>().stamp().nsec())
  >
> : std::true_type {};

template <class Msg>
double extract_timestamp_seconds(const Msg& msg) {
  if constexpr (has_timestamp_us<Msg>::value) {
    const auto us = static_cast<double>(msg.timestamp_us());
    return (std::isfinite(us) && us > 0) ? 1e-6 * us : std::numeric_limits<double>::quiet_NaN();
  }
  if constexpr (has_stamp_sec_nsec<Msg>::value) {
    const double s  = static_cast<double>(msg.stamp().sec());
    const double ns = static_cast<double>(msg.stamp().nsec());
    const double t  = s + 1e-9 * ns;
    return (std::isfinite(t) && t > 0) ? t : std::numeric_limits<double>::quiet_NaN();
  }
  return std::numeric_limits<double>::quiet_NaN();
}

} // namespace detail

class UWBReceiver {
private:
  using UwbState = unitree_go::msg::dds_::UwbState_;
  using ChannelSubscriberT = unitree::robot::ChannelSubscriber<UwbState>;
  using ChannelSubscriberPtr = std::shared_ptr<ChannelSubscriberT>;

  ChannelSubscriberPtr uwb_subscriber;
  std::mutex data_mutex;
  std::atomic<bool> data_available{false};

  UWBMeasurement latest_measurement{};
  UWBMeasurement prev_measurement{};
  bool has_prev = false;

  double r_gate = 1.0, beta_gate = 0.5, alpha_gate = 0.3, gamma_gate = 0.5;

  std::atomic<uint64_t> msg_count{0};
  std::atomic<uint64_t> gate_reject_count{0};

  struct { double roll=0, pitch=0, yaw=0; } tag_imu;
  struct { double roll=0, pitch=0, yaw=0; } base_imu;

  double joystick_x=0, joystick_y=0;
  uint8_t joy_mode=0, error_state=0;
  bool enabled_from_app=false;

  // system_clock for epoch-based latency
  std::chrono::system_clock::time_point last_update_time{};

public:
  UWBReceiver(double rg, double bg, double ag, double gg)
  : r_gate(rg), beta_gate(bg), alpha_gate(ag), gamma_gate(gg) {
    uwb_subscriber = std::make_shared<ChannelSubscriberT>("rt/uwbstate");
    uwb_subscriber->InitChannel([this](const void* message){
      if (message) handleUWBMessage(message);
    });
    std::cout << "[UWB] Receiver gates r="<<r_gate<<" β="<<beta_gate<<" α="<<alpha_gate<<" γ="<<gamma_gate<<"\n";
  }

  void handleUWBMessage(const void* message) {
    const UwbState& msg = *(const UwbState*)message;
    std::lock_guard<std::mutex> lock(data_mutex);
    msg_count++;

    UWBMeasurement nm;
    nm.r     = msg.distance_est();
    nm.beta  = msg.orientation_est();
    nm.alpha = msg.pitch_est();
    nm.gamma = msg.yaw_est();

    auto now = std::chrono::system_clock::now();
    double now_sec = std::chrono::duration<double>(now.time_since_epoch()).count();
    double ts = detail::extract_timestamp_seconds(msg);
    nm.timestamp = std::isfinite(ts) ? ts : now_sec;

    double age = now_sec - nm.timestamp;
    if (!std::isfinite(nm.timestamp) || age < -1.0 || age > 10.0) nm.timestamp = now_sec;

    // validation
    if (!std::isfinite(nm.r) || nm.r <= 0 || nm.r > 100.0 ||
        !std::isfinite(nm.beta)  || std::abs(nm.beta)  > PI ||
        !std::isfinite(nm.alpha) || std::abs(nm.alpha) > HALF_PI ||
        !std::isfinite(nm.gamma) || std::abs(nm.gamma) > PI) return;
    if (std::abs(std::cos(nm.alpha)) < 1e-6) return; // near singular elevation

    // heuristic quality
    if (msg.error_state()==0) {
      if (msg.joy_mode()==1 || msg.joy_mode()==2) nm.quality = 0.95;
      else if (msg.joy_mode()==0) nm.quality = 0.85;
      else nm.quality = 0.7;
    } else {
      nm.quality = std::max(0.3, 0.7 - 0.1*msg.error_state());
    }

    bool accept = true;
    if (has_prev) {
      double dr     = std::abs(nm.r - prev_measurement.r);
      double dbeta  = std::abs(angleDiff(nm.beta,  prev_measurement.beta));
      double dalpha = std::abs(angleDiff(nm.alpha, prev_measurement.alpha));
      double dgamma = std::abs(angleDiff(nm.gamma, prev_measurement.gamma));
      if (dr > r_gate || dbeta > beta_gate || dalpha > alpha_gate || dgamma > gamma_gate) {
        accept = false; gate_reject_count++;
      }
    }

    if (accept) {
      latest_measurement = nm;
      prev_measurement = nm;
      has_prev = true;
      data_available = true;
      last_update_time = now;
    }

    // extra state
    tag_imu.roll = msg.tag_roll();
    tag_imu.pitch = msg.tag_pitch();
    tag_imu.yaw = msg.tag_yaw();

    base_imu.roll = msg.base_roll();
    base_imu.pitch = msg.base_pitch();
    base_imu.yaw = msg.base_yaw();

    joystick_x = msg.joystick()[0];
    joystick_y = msg.joystick()[1];
    joy_mode = msg.joy_mode();
    error_state = msg.error_state();
    enabled_from_app = (msg.enabled_from_app() == 1);
  }

  bool getLatestMeasurement(UWBMeasurement& m, double max_age_seconds = 0.5) {
    std::lock_guard<std::mutex> lock(data_mutex);
    if (!data_available) return false;
    auto now = std::chrono::system_clock::now();
    double age = std::chrono::duration<double>(now - last_update_time).count();
    if (age > max_age_seconds) return false;
    m = latest_measurement;
    return true;
  }

  void updateSphericalGates(const Config& cfg) {
    std::lock_guard<std::mutex> lock(data_mutex);
    r_gate = cfg.UWB_R_GATE;
    beta_gate = cfg.UWB_BETA_GATE;
    alpha_gate = cfg.UWB_ALPHA_GATE;
    gamma_gate = cfg.UWB_GAMMA_GATE;
  }

  void getStatistics(uint64_t& total, uint64_t& rejected) {
    total = msg_count.load();
    rejected = gate_reject_count.load();
  }

  void getTagIMU(double& r, double& p, double& y) {
    std::lock_guard<std::mutex> lock(data_mutex);
    r = tag_imu.roll; p = tag_imu.pitch; y = tag_imu.yaw;
  }

  bool isTrackingEnabled() {
    std::lock_guard<std::mutex> lock(data_mutex);
    return (joy_mode==1 || joy_mode==2) && enabled_from_app;
  }

  std::string getModeName() {
    std::lock_guard<std::mutex> lock(data_mutex);
    switch(joy_mode) {
      case 0: return "Joystick";
      case 1: return "Walk Tracking";
      case 2: return "Run Tracking";
      case 3: return "Stand Up";
      case 4: return "Lie Down";
      case 5: return "Damping";
      case 6: return "Roll";
      case 7: return "Trigger Motion";
      case 8: return "Stand";
      default: return "Unknown";
    }
  }

  std::string getErrorDescription() {
    std::lock_guard<std::mutex> lock(data_mutex);
    if (error_state == 0) return "No Error";
    return "Error Code: " + std::to_string(error_state);
  }

  bool hasData() const { return data_available.load(); }

  void shutdown() {
    std::lock_guard<std::mutex> lock(data_mutex);
    data_available = false;
    has_prev = false;
    latest_measurement = UWBMeasurement{};
    prev_measurement = UWBMeasurement{};
    if (uwb_subscriber) uwb_subscriber.reset();
  }
};

} // namespace uwb_path
