# Makefile for testing minimal example
# Usage: make UNITREE_SDK=/path/to/unitree_sdk2-main

CXX = g++
CXXFLAGS = -std=c++17 -Wall -O2

# Default SDK path (update as needed)
UNITREE_SDK ?= /root/unitree_sdk2-main

# Include paths
INCLUDES = -I$(UNITREE_SDK)/include

# Check if DDS headers are in thirdparty
ifneq (,$(wildcard $(UNITREE_SDK)/thirdparty/include/ddscxx))
    INCLUDES += -I$(UNITREE_SDK)/thirdparty/include/ddscxx
endif

# Library paths
LDFLAGS = -L$(UNITREE_SDK)/lib \
          -L$(UNITREE_SDK)/lib/x86_64 \
          -L$(UNITREE_SDK)/lib/aarch64 \
          -L$(UNITREE_SDK)/build/lib \
          -Wl,-rpath,$(UNITREE_SDK)/lib

# Libraries
LIBS = -lunitree_sdk2 -lddsc -lddscxx -lpthread -lrt

# Targets
all: test_includes minimal_uwb_path

test_includes: test_includes.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) $< -o $@ $(LDFLAGS) $(LIBS)
	@echo "✓ Test includes compiled successfully!"

minimal_uwb_path: minimal_uwb_path.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) $< -o $@ $(LDFLAGS) $(LIBS)
	@echo "✓ Minimal example compiled successfully!"

clean:
	rm -f test_includes minimal_uwb_path

test: test_includes
	./test_includes

.PHONY: all clean test