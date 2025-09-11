cmake_minimum_required(VERSION 3.10)
project(uwb_path_follower)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

# --- NEW: make sure we prefer real pthread flags -----------------------------
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# Unitree SDK detection (unchanged)
if(DEFINED ENV{UNITREE_SDK_PATH})
  set(UNITREE_SDK_PATH $ENV{UNITREE_SDK_PATH})
elseif(NOT DEFINED UNITREE_SDK_PATH)
  set(UNITREE_SDK_PATH "/root/unitree_sdk2-main")
endif()

if(NOT EXISTS ${UNITREE_SDK_PATH}/include/unitree)
  message(FATAL_ERROR "Unitree SDK not found at ${UNITREE_SDK_PATH}")
endif()

message(STATUS "Using Unitree SDK: ${UNITREE_SDK_PATH}")

include_directories(
  ${UNITREE_SDK_PATH}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

if(EXISTS ${UNITREE_SDK_PATH}/thirdparty/include/ddscxx)
  include_directories(${UNITREE_SDK_PATH}/thirdparty/include/ddscxx)
endif()

link_directories(
  ${UNITREE_SDK_PATH}/lib
  ${UNITREE_SDK_PATH}/lib/aarch64
  ${UNITREE_SDK_PATH}/lib/x86_64
  ${UNITREE_SDK_PATH}/build/lib
)

# If you also build the diagnostic tools here:
add_executable(heading_step_response diagnostics/heading_step_response.cpp)
add_executable(turn_deadband_scan diagnostics/turn_deadband_scan.cpp)
add_executable(uwb_imu_latency_estimator diagnostics/uwb_imu_latency_estimator.cpp)

# Main controller as before
add_executable(uwb_path_controller
  src/uwb_path_controller.cpp
)

# --- NEW: convenience function to enforce link order everywhere --------------
function(link_go2 tgt)
  # Ensure the linker keeps these even if only transitively referenced
  target_link_options(${tgt} PRIVATE -Wl,--no-as-needed)

  # ORDER MATTERS with static libs. Put dependents AFTER the archives that need them.
  # unitree_sdk2 (static) pulls:
  #   - ddscxx (C++ API)
  #   - ddsc   (C core)
  #   - ddsrt  (CycloneDDS runtime)
  #   - pthread (via Threads::Threads)
  target_link_libraries(${tgt}
    unitree_sdk2
    ddscxx
    ddsc
    ddsrt
    m
    dl
    rt
    Threads::Threads   # keep pthreads LAST
  )

  set_target_properties(${tgt} PROPERTIES
    INSTALL_RPATH "${UNITREE_SDK_PATH}/lib:${UNITREE_SDK_PATH}/lib/aarch64:${UNITREE_SDK_PATH}/lib/x86_64:/usr/local/lib"
    BUILD_RPATH   "${UNITREE_SDK_PATH}/lib:${UNITREE_SDK_PATH}/lib/aarch64:${UNITREE_SDK_PATH}/lib/x86_64:/usr/local/lib"
  )
endfunction()

# Apply to all binaries you build in this repo
link_go2(uwb_path_controller)
link_go2(heading_step_response)
link_go2(turn_deadband_scan)
link_go2(uwb_imu_latency_estimator)

install(TARGETS uwb_path_controller DESTINATION bin)
