// diagnostics/uwb_imu_latency_estimator.cpp
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/idl/go2/UwbState_.hpp>   // if available on your system
#include <cmath>
#include <cstdio>
#include <deque>
#include <mutex>
#include <string>
#include <atomic>
#include <thread>
#include <chrono>
#include <csignal>
#include <iostream>

using unitree::robot::ChannelFactory;
using unitree::robot::ChannelSubscriber;
using unitree::robot::ChannelSubscriberPtr;

static std::atomic<bool> g_run{true};
static void handle_sigint(int){ g_run=false; }

struct Buff {
  std::deque<double> imu_yaw;   // rad
  std::deque<double> uwb_yaw;   // rad
  int N = 2000;                 // ~20s @ 100 Hz
  void push(double iy, double uy){
    imu_yaw.push_back(iy);
    uwb_yaw.push_back(uy);
    if ((int)imu_yaw.size() > N){ imu_yaw.pop_front(); uwb_yaw.pop_front(); }
  }
  bool ready() const { return (int)imu_yaw.size()==N; }
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

  ChannelFactory::Instance()->Init(0, iface.c_str()); // as in your examples. :contentReference[oaicite:15]{index=15}

  std::mutex mtx;
  Buff buff;
  double last_imu_yaw=0.0, last_uwb_yaw=0.0;

  // IMU yaw from sport state (same topic used in your examples). :contentReference[oaicite:16]{index=16}
  auto sub_imu = std::make_shared<ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>>("rt/sportmodestate");
  sub_imu->InitChannel([&](const void* msg){
    const auto& s = *(const unitree_go::msg::dds_::SportModeState_*)msg;
    std::lock_guard<std::mutex> lk(mtx);
    last_imu_yaw = s.imu_state().rpy()[2];
  }, 1);

  // If you have UWB heading on DDS, subscribe here; otherwise comment this section and stream yaw via stdin.
  // Many UWB systems give position, not yaw; if so, compute bearing change externally and feed in.
  bool uwb_dds_ok = true;
  ChannelSubscriberPtr<unitree_go::msg::dds_::UwbState_> sub_uwb(
    new ChannelSubscriber<unitree_go::msg::dds_::UwbState_>("rt/uwbstate"));
  try {
    sub_uwb->InitChannel([&](const void* msg){
      const auto& u = *(const unitree_go::msg::dds_::UwbState_*)msg;
      // If no heading in message, derive your own (example uses yaw field name 'yaw' if present)
      double uwb_yaw = 0.0;
      // Replace this if your UWB message encodes orientation differently:
      if (u.__is_yaw_present()){ uwb_yaw = u.yaw(); } // if generated code provides this accessor
      std::lock_guard<std::mutex> lk(mtx);
      last_uwb_yaw = wrapToPi(uwb_yaw);
    }, 1);
  } catch (...) {
    uwb_dds_ok = false;
    std::cerr << "[Info] UWB DDS subscription failed or message type unavailable; "
              << "you can pipe uwb yaw (rad) via stdin at ~100 Hz as a fallback.\n";
  }

  // Cross-correlation loop @ ~100 Hz
  const int Hz = 100;
  auto period = std::chrono::milliseconds(1000/Hz);

  while (g_run.load()){
    auto next = std::chrono::steady_clock::now() + period;

    // Optional stdin fallback for UWB yaw
    if (!uwb_dds_ok){
      if (std::cin.good()){
        double uy=0.0;
        if (std::cin >> uy){ last_uwb_yaw = wrapToPi(uy); }
      }
    }

    {
      std::lock_guard<std::mutex> lk(mtx);
      buff.push(last_imu_yaw, last_uwb_yaw);
    }

    if (buff.ready()){
      // compute correlation for lags in +/-500 ms
      int maxLag = (int)(0.5 * Hz);
      double bestR=-1e9; int bestL=0;
      for (int L=-maxLag; L<=maxLag; ++L){
        double c=0, sii=0, suu=0;
        for (int i=0;i<buff.N;i++){
          int j=i+L;
          if (j<0 || j>=buff.N) continue;
          double a = buff.imu_yaw[i];
          double b = buff.uwb_yaw[j];
          c   += a*b;
          sii += a*a;
          suu += b*b;
        }
        double denom = std::sqrt(sii*suu)+1e-12;
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
