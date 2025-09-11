// diagnostics/heading_step_response.cpp
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/robot/go2/sport/sport_client.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

using unitree::robot::ChannelFactory;
using unitree::robot::ChannelSubscriber;
using unitree::robot::ChannelSubscriberPtr;
using unitree::robot::go2::SportClient;

static std::atomic<bool> g_run{true};
static void handle_sigint(int){ g_run = false; }

static inline double wrapToPi(double a){
  while(a >  M_PI) a -= 2*M_PI;
  while(a < -M_PI) a += 2*M_PI;
  return a;
}
static inline double rad2deg(double r){ return r * 180.0 / M_PI; }

struct Telemetry {
  double yaw = 0.0;      // IMU yaw (rad)
  double gyro_z = 0.0;   // IMU yaw rate (rad/s)
  double ts = 0.0;       // wall time (s)
};

int main(int argc, char** argv){
  // Args
  if (argc < 3){
    std::cerr << "Usage: " << argv[0]
              << " <networkInterface> <target_heading_deg> [enter_tol_deg=10] [exit_tol_deg=15]\n";
    return 2;
  }
  const std::string iface = argv[1];
  const double target_deg = std::stod(argv[2]);
  const double enter_tol_deg = (argc>3) ? std::stod(argv[3]) : 10.0;
  const double exit_tol_deg  = (argc>4) ? std::stod(argv[4]) : 15.0;

  std::signal(SIGINT, handle_sigint);

  // Init DDS / network
  ChannelFactory::Instance()->Init(0, iface.c_str()); // matches your examples. :contentReference[oaicite:4]{index=4}

  // Sport client and subscriber
  SportClient sc;
  sc.SetTimeout(10.0f);
  sc.Init(); // as in your sport example. :contentReference[oaicite:5]{index=5}

  std::mutex mtx;
  Telemetry tel;
  auto sub = std::make_shared<ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>>("rt/sportmodestate");
  sub->InitChannel([&](const void* msg){
    const auto& s = *(const unitree_go::msg::dds_::SportModeState_*)msg;
    std::lock_guard<std::mutex> lk(mtx);
    tel.yaw    = s.imu_state().rpy()[2];         // yaw (rad)
    tel.gyro_z = s.imu_state().gyroscope()[2];   // rad/s
    tel.ts     = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
  }, 1); // same topic as your example. :contentReference[oaicite:6]{index=6}

  // Parameters
  const int Hz = 100;                       // command + log rate
  const auto period = std::chrono::milliseconds(1000/Hz);
  const double target = target_deg * M_PI/180.0;

  const double Kp_turn = 1.0;               // ω = Kp*err (rad)
  const double w_min = 0.30;                // rad/s (deadband floor)
  const double w_max = 1.20;                // rad/s clamp
  const double v_min = 0.15;                // m/s (deadband floor)
  const double v_cmd = 0.35;                // forward speed when unlocked

  const double enter_tol = enter_tol_deg * M_PI/180.0;
  const double exit_tol  = exit_tol_deg  * M_PI/180.0;

  enum {TURN=0, FWD=1};
  int state = TURN;

  std::puts("t_s,yaw_deg,yawrate_rads,target_deg,err_deg,vx_cmd,wz_cmd,state");

  // Simple control loop with hysteresis and velocity floors
  auto next = std::chrono::steady_clock::now();
  while (g_run.load()){
    next += period;

    Telemetry t; { std::lock_guard<std::mutex> lk(mtx); t = tel; }
    const double err = wrapToPi(target - t.yaw);

    // hysteresis gate
    if (state==TURN && std::fabs(err) <= enter_tol) state = FWD;
    else if (state==FWD && std::fabs(err) >= exit_tol) state = TURN;

    double vx=0.0, wz=0.0;
    if (state==TURN){
      wz = std::clamp(Kp_turn * err, -w_max, w_max);
      if (std::fabs(wz) < w_min) wz = (wz >= 0.0 ? +w_min : -w_min);
    } else {
      vx = v_cmd;
      if (vx < v_min) vx = v_min;
    }

    // Send command (same API used in your code) :contentReference[oaicite:7]{index=7}
    sc.Move((float)vx, 0.0f, (float)wz);

    std::printf("%.3f,%.3f,%.4f,%.3f,%.3f,%.3f,%.3f,%d\n",
      t.ts, rad2deg(t.yaw), t.gyro_z, target_deg, rad2deg(err), vx, wz, state);

    std::this_thread::sleep_until(next);
  }

  // Stop
  sc.Move(0.0f,0.0f,0.0f);
  return 0;
}
