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

find_package(Threads REQUIRED)

# Unitree SDK detection
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

# Main controller
add_executable(uwb_path_controller
  src/uwb_path_controller.cpp
)

target_link_libraries(uwb_path_controller
  unitree_sdk2
  ddsc
  ddscxx
  Threads::Threads
  rt
  m
  dl
)

set_target_properties(uwb_path_controller PROPERTIES
  INSTALL_RPATH "${UNITREE_SDK_PATH}/lib:/usr/local/lib"
  BUILD_RPATH "${UNITREE_SDK_PATH}/lib:/usr/local/lib"
)

# Optional: Build diagnostics separately with their own Makefile
option(BUILD_DIAGNOSTICS "Build diagnostic tools" OFF)
if(BUILD_DIAGNOSTICS)
  add_executable(heading_step_response diagnostics/heading_step_response.cpp)
  add_executable(turn_deadband_scan diagnostics/turn_deadband_scan.cpp)
  add_executable(uwb_imu_latency_estimator diagnostics/uwb_imu_latency_estimator.cpp)
  
  foreach(diag_target heading_step_response turn_deadband_scan uwb_imu_latency_estimator)
    target_link_libraries(${diag_target}
      unitree_sdk2
      ddsc
      ddscxx
      Threads::Threads
      rt
      m
      dl
    )
    set_target_properties(${diag_target} PROPERTIES
      INSTALL_RPATH "${UNITREE_SDK_PATH}/lib:/usr/local/lib"
      BUILD_RPATH "${UNITREE_SDK_PATH}/lib:/usr/local/lib"
    )
  endforeach()
endif()

install(TARGETS uwb_path_controller DESTINATION bin)