// diagnostics/turn_deadband_scan.cpp
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
#include <functional>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

using unitree::robot::ChannelFactory;
using unitree::robot::ChannelSubscriber;
using unitree::robot::ChannelSubscriberPtr;
using unitree::robot::go2::SportClient;

static std::atomic<bool> g_run{true};
static void handle_sigint(int){ g_run = false; }

struct Telemetry {
  double yaw = 0.0;      // rad
  double gyro_z = 0.0;   // rad/s
};

static inline double wrapToPi(double a){
  while(a >  M_PI) a -= 2*M_PI;
  while(a < -M_PI) a += 2*M_PI;
  return a;
}

int main(int argc, char** argv){
  if (argc < 2){
    std::cerr << "Usage: " << argv[0] << " <networkInterface>\n";
    return 2;
  }
  const std::string iface = argv[1];
  std::signal(SIGINT, handle_sigint);

  ChannelFactory::Instance()->Init(0, iface.c_str()); // same as your examples. :contentReference[oaicite:9]{index=9}

  SportClient sc;
  sc.SetTimeout(10.0f);
  sc.Init(); // :contentReference[oaicite:10]{index=10}

  std::mutex mtx;
  Telemetry tel;
  auto sub = std::make_shared<ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>>("rt/sportmodestate");
  sub->InitChannel([&](const void* msg){
    const auto& s = *(const unitree_go::msg::dds_::SportModeState_*)msg;
    std::lock_guard<std::mutex> lk(mtx);
    tel.yaw    = s.imu_state().rpy()[2];
    tel.gyro_z = s.imu_state().gyroscope()[2];
  }, 1); // :contentReference[oaicite:11]{index=11}

  auto sleep_s = [](double secs){
    std::this_thread::sleep_for(std::chrono::milliseconds((int)(secs*1000)));
  };

  std::puts("mode,command,observed");

  // TURN deadband sweep: command ω, measure observed average |gyro_z|
  for (double w=0.05; g_run && w<=1.0; w+=0.05){
    sc.Move(0.0f, 0.0f, (float)+w);
    sleep_s(0.5); // settle
    double sum=0; int n=0;
    for (int i=0;i<200 && g_run;i++){
      { std::lock_guard<std::mutex> lk(mtx); sum += std::fabs(tel.gyro_z); }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      n++;
    }
    sc.Move(0.0f,0.0f,0.0f);
    sleep_s(0.2);
    std::printf("turn,+%.2f,%.5f\n", w, (n?sum/n:0.0));

    sc.Move(0.0f, 0.0f, (float)-w);
    sleep_s(0.5);
    sum=0; n=0;
    for (int i=0;i<200 && g_run;i++){
      { std::lock_guard<std::mutex> lk(mtx); sum += std::fabs(tel.gyro_z); }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      n++;
    }
    sc.Move(0.0f,0.0f,0.0f);
    sleep_s(0.2);
    std::printf("turn,-%.2f,%.5f\n", w, (n?sum/n:0.0));
  }

  // FORWARD deadband sweep: command v, estimate speed by yaw integration proxy not ideal; better:
  // if you have odom, swap in odom speed. Here we use crude |gyro_z| veto: we expect low yaw during straight.
  // You can replace with UWB delta-x over interval if stable (your UWB controller already converts and logs). :contentReference[oaicite:12]{index=12}
  for (double v=0.05; g_run && v<=0.6; v+=0.05){
    sc.Move((float)v, 0.0f, 0.0f);
    sleep_s(1.5); // allow stepping to stabilize
    // No direct body speed in SportModeState; leave as "commanded v" marker.
    // If you have a position topic or your UWB logger available, collect that speed offline against this CSV.
    std::printf("forward,%.2f,%.2f\n", v, v);
    sc.Move(0.0f,0.0f,0.0f);
    sleep_s(0.5);
  }

  sc.Move(0.0f,0.0f,0.0f);
  return 0;
}
