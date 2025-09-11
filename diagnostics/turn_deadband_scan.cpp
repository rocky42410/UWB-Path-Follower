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
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

using unitree::robot::ChannelFactory;
using unitree::robot::ChannelSubscriber;
using unitree::robot::go2::SportClient;

using SportState = unitree_go::msg::dds_::SportModeState_;

static std::atomic<bool> g_run{true};
static void on_sigint(int){ g_run=false; }

struct Telemetry {
  double gyro_z = 0.0; // rad/s
};

int main(int argc, char** argv){
  if (argc < 2){
    std::cerr << "Usage: " << argv[0] << " <networkInterface>\n";
    return 2;
  }
  const std::string iface = argv[1];
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
    tel.gyro_z = s.imu_state().gyroscope()[2];
  }, 1);

  auto sleep_s = [](double secs){
    std::this_thread::sleep_for(std::chrono::milliseconds((int)(secs*1000)));
  };

  std::puts("mode,command,observed");

  for (double w=0.05; g_run && w<=1.00; w+=0.05){
    sc.Move(0.0f, 0.0f, (float)+w);
    sleep_s(0.5);
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

  for (double v=0.05; g_run && v<=0.60; v+=0.05){
    sc.Move((float)v, 0.0f, 0.0f);
    sleep_s(1.5);
    std::printf("forward,%.2f,%.2f\n", v, v);
    sc.Move(0.0f,0.0f,0.0f);
    sleep_s(0.5);
  }

  sc.Move(0,0,0);
  return 0;
}
