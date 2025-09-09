cmake_minimum_required(VERSION 3.10)
project(uwb_path_follower)

# C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -O2")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

# Find packages
find_package(Threads REQUIRED)

# ============= UNITREE SDK PATH DETECTION =============
# Try multiple methods to find the SDK

# Method 1: Environment variable
if(DEFINED ENV{UNITREE_SDK_PATH})
    set(UNITREE_SDK_PATH $ENV{UNITREE_SDK_PATH})
    message(STATUS "Found UNITREE_SDK_PATH from environment: ${UNITREE_SDK_PATH}")
endif()

# Method 2: Common installation paths
if(NOT DEFINED UNITREE_SDK_PATH OR NOT EXISTS ${UNITREE_SDK_PATH})
    set(SDK_SEARCH_PATHS
        /opt/unitree_sdk2
        /usr/local/unitree_sdk2
        /opt/unitree/unitree_sdk2
        ~/unitree_sdk2
        ~/software/unitree_sdk2
        ~/ws/unitree_sdk2
        ${CMAKE_CURRENT_SOURCE_DIR}/../unitree_sdk2
        ${CMAKE_CURRENT_SOURCE_DIR}/../../unitree_sdk2
    )
    
    foreach(PATH ${SDK_SEARCH_PATHS})
        if(EXISTS ${PATH}/include/unitree)
            set(UNITREE_SDK_PATH ${PATH})
            message(STATUS "Found Unitree SDK at: ${UNITREE_SDK_PATH}")
            break()
        endif()
    endforeach()
endif()

# Method 3: User specified via cmake
if(NOT DEFINED UNITREE_SDK_PATH OR NOT EXISTS ${UNITREE_SDK_PATH})
    set(UNITREE_SDK_PATH "/path/to/unitree_sdk2" CACHE PATH "Path to Unitree SDK")
endif()

# Verify the SDK path is valid
if(NOT EXISTS ${UNITREE_SDK_PATH}/include/unitree)
    message(FATAL_ERROR 
        "Unitree SDK not found!\n"
        "The directory ${UNITREE_SDK_PATH}/include/unitree does not exist.\n"
        "Please set UNITREE_SDK_PATH using one of these methods:\n"
        "  1. Environment: export UNITREE_SDK_PATH=/path/to/unitree_sdk2\n"
        "  2. CMake: cmake .. -DUNITREE_SDK_PATH=/path/to/unitree_sdk2\n"
        "  3. Install SDK to standard location: /opt/unitree_sdk2\n"
        "\n"
        "The SDK directory should contain:\n"
        "  include/unitree/  (with robot/, common/, idl/ subdirs)\n"
        "  lib/             (with libunitree_sdk2.so)\n"
    )
endif()

# Verify critical headers exist
set(REQUIRED_HEADERS
    ${UNITREE_SDK_PATH}/include/unitree/robot/client/sport_client.hpp
    ${UNITREE_SDK_PATH}/include/unitree/robot/channel/channel_publisher.hpp
    ${UNITREE_SDK_PATH}/include/unitree/idl/go2/SportModeState_.hpp
    ${UNITREE_SDK_PATH}/include/unitree/idl/go2/UwbState_.hpp
)

foreach(HEADER ${REQUIRED_HEADERS})
    if(NOT EXISTS ${HEADER})
        message(WARNING "Required header not found: ${HEADER}")
    endif()
endforeach()

# Verify library exists
if(NOT EXISTS ${UNITREE_SDK_PATH}/lib/libunitree_sdk2.so)
    message(WARNING "Unitree SDK library not found: ${UNITREE_SDK_PATH}/lib/libunitree_sdk2.so")
endif()

message(STATUS "Using Unitree SDK from: ${UNITREE_SDK_PATH}")
message(STATUS "  Include: ${UNITREE_SDK_PATH}/include")
message(STATUS "  Library: ${UNITREE_SDK_PATH}/lib")

# Include directories
include_directories(
    ${UNITREE_SDK_PATH}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Link directories
link_directories(
    ${UNITREE_SDK_PATH}/lib
)

# Find CycloneDDS (optional, may be bundled with Unitree SDK)
find_library(DDSC_LIB ddsc
    HINTS ${UNITREE_SDK_PATH}/lib
    /usr/local/lib
    /usr/lib
)

if(NOT DDSC_LIB)
    message(WARNING "CycloneDDS library (ddsc) not found. Build may fail.")
else()
    message(STATUS "Found CycloneDDS: ${DDSC_LIB}")
endif()

# Main executable
add_executable(uwb_path_controller
    src/uwb_path_controller.cpp
)

# Link libraries
target_link_libraries(uwb_path_controller
    unitree_sdk2
    ddsc
    Threads::Threads
    m
)

# Set RPATH for runtime library loading
set_target_properties(uwb_path_controller PROPERTIES
    INSTALL_RPATH "${UNITREE_SDK_PATH}/lib"
    BUILD_RPATH "${UNITREE_SDK_PATH}/lib"
)

# Test executable (if exists)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/test_estimator.cpp")
    add_executable(test_estimator
        test/test_estimator.cpp
    )
    
    target_link_libraries(test_estimator
        unitree_sdk2
        ddsc
        Threads::Threads
        m
    )
    
    set_target_properties(test_estimator PROPERTIES
        INSTALL_RPATH "${UNITREE_SDK_PATH}/lib"
        BUILD_RPATH "${UNITREE_SDK_PATH}/lib"
    )
endif()

# Installation
install(TARGETS uwb_path_controller
    DESTINATION bin
)

install(FILES config/uwb_config.yaml
    DESTINATION etc/uwb_path_follower
)

# Print summary
message(STATUS "===========================================")
message(STATUS "UWB Path Follower Configuration Summary:")
message(STATUS "  CMake Version: ${CMAKE_VERSION}")
message(STATUS "  Compiler: ${CMAKE_CXX_COMPILER}")
message(STATUS "  Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "  Unitree SDK: ${UNITREE_SDK_PATH}")
message(STATUS "  CycloneDDS: ${DDSC_LIB}")
message(STATUS "===========================================")
message(STATUS "To build: make -j$(nproc)")
message(STATUS "===========================================")