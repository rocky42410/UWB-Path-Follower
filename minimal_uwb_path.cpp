// minimal_uwb_path.cpp - Minimal example with correct includes
#include <iostream>
#include <thread>
#include <chrono>
#include <signal.h>

// Correct Unitree SDK includes
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/idl/go2/UwbState_.hpp>
#include <unitree/robot/go2/sport/sport_client.hpp>  // CORRECT PATH

using namespace unitree::robot;

class SimpleController {
private:
    unitree::robot::go2::SportClient sport_client;  // CORRECT NAMESPACE
    ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_> sport_sub;
    ChannelSubscriberPtr<unitree_go::msg::dds_::UwbState_> uwb_sub;
    
    bool running = true;
    
public:
    SimpleController(const std::string& network_interface) {
        // Initialize DDS
        ChannelFactory::Instance()->Init(0, network_interface.c_str());
        std::cout << "[Init] DDS initialized on: " << network_interface << std::endl;
        
        // Initialize sport client with correct pattern
        sport_client.SetTimeout(10.0f);
        sport_client.Init();
        std::cout << "[Init] Sport client initialized" << std::endl;
        
        // Subscribe to sport state
        sport_sub = std::make_shared<ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>>(
            "rt/sportmodestate");
        sport_sub->InitChannel([this](const void* msg) {
            handleSportState(msg);
        });
        
        // Subscribe to UWB state
        uwb_sub = std::make_shared<ChannelSubscriber<unitree_go::msg::dds_::UwbState_>>(
            "rt/uwbstate");
        uwb_sub->InitChannel([this](const void* msg) {
            handleUWBState(msg);
        });
        
        std::cout << "[Init] All subscribers initialized" << std::endl;
    }
    
    void handleSportState(const void* message) {
        const auto& msg = *(const unitree_go::msg::dds_::SportModeState_*)message;
        // Process sport state
        static int count = 0;
        if (++count % 50 == 0) {  // Every second at 50Hz
            std::cout << "[Sport] IMU Roll: " << msg.imu_state().rpy()[0] << std::endl;
        }
    }
    
    void handleUWBState(const void* message) {
        const auto& msg = *(const unitree_go::msg::dds_::UwbState_*)message;
        // Process UWB data
        std::cout << "[UWB] Distance: " << msg.distance_est() 
                  << " m, Quality: " << (int)msg.error_state() << std::endl;
    }
    
    void run() {
        // Simple movement pattern
        std::cout << "[Run] Starting control loop..." << std::endl;
        
        while (running) {
            // Forward for 2 seconds
            sport_client.Move(0.3, 0, 0);
            std::this_thread::sleep_for(std::chrono::seconds(2));
            
            // Stop for 1 second
            sport_client.Move(0, 0, 0);
            std::this_thread::sleep_for(std::chrono::seconds(1));
            
            // Turn for 1 second
            sport_client.Move(0, 0, 0.3);
            std::this_thread::sleep_for(std::chrono::seconds(1));
            
            // Stop
            sport_client.Move(0, 0, 0);
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        
        // Stop on exit
        sport_client.Move(0, 0, 0);
    }
    
    void stop() { running = false; }
};

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " <network_interface>" << std::endl;
        std::cout << "Example: " << argv[0] << " eth0" << std::endl;
        return 1;
    }
    
    try {
        SimpleController controller(argv[1]);
        
        // Setup signal handler
        std::signal(SIGINT, [](int) {
            std::cout << "\n[Signal] Shutting down..." << std::endl;
            std::exit(0);
        });
        
        controller.run();
    } catch (const std::exception& e) {
        std::cerr << "[Error] " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}