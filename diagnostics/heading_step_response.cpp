#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/robot/go2/sport/sport_client.hpp>

#include <algorithm>   // <-- needed for std::clamp
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

using unitree::robot::ChannelFactory;
using unitree::robot::ChannelSubscriber;
using unitree::robot::go2::SportClient;

using SportState = unitree_go::msg::dds_::SportModeState_;

static std::atomic<bool> g_run{true};
static void on_sigint(int) { g_run = false; }

static inline double wrapToPi(double a){
  constexpr double PI = 3.14159265358979323846;
  while (a >  PI) a -= 2*PI;
  while (a < -PI) a += 2*PI;
  return a;
}
static inline double rad2deg(double r){
  return r * 180.0 / 3.14159265358979323846;
}

struct Telemetry {
  double yaw = 0.0;      // IMU yaw (rad)
  double gyro_z = 0.0;   // IMU yaw-rate (rad/s)
  double t_s = 0.0;      // wall time seconds
};

int main(int argc, char** argv){
  if (argc < 3){
    std::cerr << "Usage: " << argv[0]
              << " <networkInterface> <target_heading_deg> [enter_tol_deg=10] [exit_tol_deg=15]\n";
    return 2;
  }
  const std::string iface = argv[1];
  const double target_deg = std::stod(argv[2]);
  const double enter_tol_deg = (argc>3) ? std::stod(argv[3]) : 10.0;
  const double exit_tol_deg  = (argc>4) ? std::stod(argv[4]) : 15.0;

  std::signal(SIGINT, on_sigint);

  ChannelFactory::Instance()->Init(0, iface.c_str());

  SportClient sc;
  sc.SetTimeout(10.0f);
  sc.Init();

  std::mutex mtx;
  Telemetry tel;

  auto sub = std::make_shared<ChannelSubscriber<SportState>>("rt/sportmodestate");
  sub->InitChannel([&](const void* msg){
    const auto& s = *static_cast<const SportState*>(msg);
    std::lock_guard<std::mutex> lk(mtx);
    tel.yaw    = s.imu_state().rpy()[2];
    tel.gyro_z = s.imu_state().gyroscope()[2];
    tel.t_s    = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
  }, 1);

  const int Hz = 100;
  const auto period = std::chrono::milliseconds(1000/Hz);
  const double target = target_deg * 3.14159265358979323846 / 180.0;

  const double Kp_turn = 1.0;   // ω = Kp * err
  const double w_min   = 0.30;  // rad/s deadband floor
  const double w_max   = 1.20;  // rad/s cap
  const double v_min   = 0.15;  // m/s floor
  const double v_cmd   = 0.35;  // forward when unlocked

  const double enter_tol = enter_tol_deg * 3.14159265358979323846 / 180.0;
  const double exit_tol  = exit_tol_deg  * 3.14159265358979323846 / 180.0;

  enum { TURN=0, FWD=1 };
  int state = TURN;

  std::puts("t_s,yaw_deg,yawrate_rads,target_deg,err_deg,vx_cmd,wz_cmd,state");

  auto next = std::chrono::steady_clock::now();
  while (g_run.load()){
    next += period;

    Telemetry t;
    { std::lock_guard<std::mutex> lk(mtx); t = tel; }

    const double err = wrapToPi(target - t.yaw);

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

    sc.Move((float)vx, 0.0f, (float)wz);

    std::printf("%.3f,%.3f,%.4f,%.3f,%.3f,%.3f,%.3f,%d\n",
                t.t_s, rad2deg(t.yaw), t.gyro_z, target_deg, rad2deg(err), vx, wz, state);

    std::this_thread::sleep_until(next);
  }

  sc.Move(0.0f, 0.0f, 0.0f);
  return 0;
}
