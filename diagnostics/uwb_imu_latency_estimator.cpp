#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/idl/go2/UwbState_.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <deque>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

using unitree::robot::ChannelFactory;
using unitree::robot::ChannelSubscriber;

using SportState = unitree_go::msg::dds_::SportModeState_;
using UwbState   = unitree_go::msg::dds_::UwbState_;

static std::atomic<bool> g_run{true};
static void on_sigint(int){ g_run=false; }

static inline double wrapToPi(double a){
  constexpr double PI = 3.14159265358979323846;
  while (a >  PI) a -= 2*PI;
  while (a < -PI) a += 2*PI;
  return a;
}

struct Buff {
  std::deque<double> imu_yaw;  // rad
  std::deque<double> uwb_yaw;  // rad (from yaw_est)
  int N = 2000;                // ~20 s @ 100 Hz
  void push(double iy, double uy){
    imu_yaw.push_back(iy);
    uwb_yaw.push_back(uy);
    if ((int)imu_yaw.size() > N){ imu_yaw.pop_front(); uwb_yaw.pop_front(); }
  }
  bool ready() const { return (int)imu_yaw.size()==N; }
};

int main(int argc, char** argv){
  if (argc < 2){
    std::cerr << "Usage: " << argv[0] << " <networkInterface>\n";
    return 2;
  }
  const std::string iface = argv[1];
  std::signal(SIGINT, on_sigint);

  ChannelFactory::Instance()->Init(0, iface.c_str());

  std::mutex mtx;
  Buff buff;
  double imu_yaw=0.0, uwb_yaw=0.0;

  auto sub_imu = std::make_shared<ChannelSubscriber<SportState>>("rt/sportmodestate");
  sub_imu->InitChannel([&](const void* msg){
    const auto& s = *static_cast<const SportState*>(msg);
    std::lock_guard<std::mutex> lk(mtx);
    imu_yaw = s.imu_state().rpy()[2];
  }, 1);

  auto sub_uwb = std::make_shared<ChannelSubscriber<UwbState>>("rt/uwbstate");
  sub_uwb->InitChannel([&](const void* msg){
    const auto& u = *static_cast<const UwbState*>(msg);
    std::lock_guard<std::mutex> lk(mtx);
    uwb_yaw = wrapToPi(u.yaw_est());
  }, 1);

  const int Hz = 100;
  const auto period = std::chrono::milliseconds(1000/Hz);

  while (g_run.load()){
    const auto next = std::chrono::steady_clock::now() + period;
    {
      std::lock_guard<std::mutex> lk(mtx);
      buff.push(imu_yaw, uwb_yaw);
    }

    if (buff.ready()){
      const int maxLag = (int)(0.5 * Hz);
      double bestR = -1e9; int bestL = 0;
      for (int L=-maxLag; L<=maxLag; ++L){
        double c=0, sii=0, suu=0;
        for (int i=0; i<buff.N; ++i){
          int j = i+L;
          if (j<0 || j>=buff.N) continue;
          double a = buff.imu_yaw[i];
          double b = buff.uwb_yaw[j];
          c   += a*b;
          sii += a*a;
          suu += b*b;
        }
        double denom = std::sqrt(sii*suu) + 1e-12;
        double r = c/denom;
        if (r > bestR){ bestR=r; bestL=L; }
      }
      double lag_ms = 1000.0 * (double)bestL / (double)Hz;
      std::fprintf(stderr, "Estimated UWB lag vs IMU: %.1f ms (rho=%.3f)\n", lag_ms, bestR);
    }

    std::this_thread::sleep_until(next);
  }
  return 0;
}
