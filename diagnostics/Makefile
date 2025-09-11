# diagnostics/Makefile
CXX = g++
CXXFLAGS = -std=c++17 -O2 -Wall

UNITREE_SDK ?= /root/unitree_sdk2-main
INCLUDES = -I$(UNITREE_SDK)/include
LDFLAGS = -L$(UNITREE_SDK)/lib -lunitree_sdk2 -lddsc -lddscxx -lpthread -lrt -lm -ldl

# Add DDS includes if available
ifneq (,$(wildcard $(UNITREE_SDK)/thirdparty/include/ddscxx))
    INCLUDES += -I$(UNITREE_SDK)/thirdparty/include/ddscxx
endif

TARGETS = heading_step_response turn_deadband_scan uwb_imu_latency_estimator

all: $(TARGETS)

%: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) $< -o $@ $(LDFLAGS)

clean:
	rm -f $(TARGETS)

.PHONY: all clean