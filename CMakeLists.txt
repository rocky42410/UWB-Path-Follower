cmake_minimum_required(VERSION 3.10)
project(uwb_path_follower)

# C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

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
# Method 1: Environment variable
if(DEFINED ENV{UNITREE_SDK_PATH})
    set(UNITREE_SDK_PATH $ENV{UNITREE_SDK_PATH})
    message(STATUS "Found UNITREE_SDK_PATH from environment: ${UNITREE_SDK_PATH}")
endif()

# Method 2: Try common paths
if(NOT DEFINED UNITREE_SDK_PATH OR NOT EXISTS ${UNITREE_SDK_PATH})
    set(SDK_SEARCH_PATHS
        /root/unitree_sdk2-main          # Your actual path from the error
        /root/unitree_sdk2
        /opt/unitree_sdk2
        /usr/local/unitree_sdk2
        ~/unitree_sdk2-main
        ~/unitree_sdk2
    )
    
    foreach(PATH ${SDK_SEARCH_PATHS})
        if(EXISTS ${PATH}/include/unitree)
            set(UNITREE_SDK_PATH ${PATH})
            message(STATUS "Found Unitree SDK at: ${UNITREE_SDK_PATH}")
            break()
        endif()
    endforeach()
endif()

# Default fallback
if(NOT DEFINED UNITREE_SDK_PATH)
    set(UNITREE_SDK_PATH "/root/unitree_sdk2-main" CACHE PATH "Path to Unitree SDK")
endif()

# Verify SDK exists
if(NOT EXISTS ${UNITREE_SDK_PATH}/include/unitree)
    message(FATAL_ERROR "Unitree SDK not found at ${UNITREE_SDK_PATH}")
endif()

message(STATUS "Using Unitree SDK from: ${UNITREE_SDK_PATH}")

# ============= CYCLONE DDS DETECTION =============
# The Unitree SDK requires CycloneDDS headers and libraries

# Method 1: Check if it's in the Unitree SDK's thirdparty
set(DDS_INCLUDE_DIRS "")
set(DDS_LIBRARIES "")

# Check for thirdparty directory in Unitree SDK
if(EXISTS ${UNITREE_SDK_PATH}/thirdparty)
    message(STATUS "Found thirdparty directory in Unitree SDK")
    
    # Look for CycloneDDS in thirdparty
    if(EXISTS ${UNITREE_SDK_PATH}/thirdparty/include/ddscxx)
        list(APPEND DDS_INCLUDE_DIRS ${UNITREE_SDK_PATH}/thirdparty/include)
        list(APPEND DDS_INCLUDE_DIRS ${UNITREE_SDK_PATH}/thirdparty/include/ddscxx)
        message(STATUS "Found DDS headers in thirdparty")
    endif()
    
    # Look for built libraries
    if(EXISTS ${UNITREE_SDK_PATH}/build/lib)
        link_directories(${UNITREE_SDK_PATH}/build/lib)
        set(DDS_LIB_PATH ${UNITREE_SDK_PATH}/build/lib)
    elseif(EXISTS ${UNITREE_SDK_PATH}/thirdparty/lib)
        link_directories(${UNITREE_SDK_PATH}/thirdparty/lib)
        set(DDS_LIB_PATH ${UNITREE_SDK_PATH}/thirdparty/lib)
    endif()
endif()

# Method 2: Check system-wide installation
if(NOT DDS_INCLUDE_DIRS)
    # Common CycloneDDS installation paths
    set(DDS_SEARCH_PATHS
        /usr/local/include
        /usr/include
        /opt/cyclonedds/include
        ${CMAKE_PREFIX_PATH}/include
    )
    
    foreach(PATH ${DDS_SEARCH_PATHS})
        if(EXISTS ${PATH}/dds/dds.hpp OR EXISTS ${PATH}/ddscxx/dds/dds.hpp)
            list(APPEND DDS_INCLUDE_DIRS ${PATH})
            if(EXISTS ${PATH}/ddscxx)
                list(APPEND DDS_INCLUDE_DIRS ${PATH}/ddscxx)
            endif()
            message(STATUS "Found DDS headers at: ${PATH}")
            break()
        endif()
    endforeach()
endif()

# Method 3: Try pkg-config
find_package(PkgConfig QUIET)
if(PKG_CONFIG_FOUND)
    pkg_check_modules(CYCLONEDDS cyclonedds-cxx)
    if(CYCLONEDDS_FOUND)
        list(APPEND DDS_INCLUDE_DIRS ${CYCLONEDDS_INCLUDE_DIRS})
        message(STATUS "Found CycloneDDS via pkg-config")
    endif()
endif()

if(NOT DDS_INCLUDE_DIRS)
    message(WARNING "CycloneDDS headers not found. You may need to:")
    message(WARNING "  1. Build Unitree SDK's thirdparty directory:")
    message(WARNING "     cd ${UNITREE_SDK_PATH} && mkdir build && cd build")
    message(WARNING "     cmake .. && make")
    message(WARNING "  2. Or install CycloneDDS separately:")
    message(WARNING "     git clone https://github.com/eclipse-cyclonedds/cyclonedds-cxx.git")
    message(WARNING "     cd cyclonedds-cxx && mkdir build && cd build")
    message(WARNING "     cmake .. && make && sudo make install")
else()
    message(STATUS "DDS Include dirs: ${DDS_INCLUDE_DIRS}")
endif()

# ============= INCLUDE DIRECTORIES =============
include_directories(
    ${UNITREE_SDK_PATH}/include
    ${DDS_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# ============= LINK DIRECTORIES =============
link_directories(
    ${UNITREE_SDK_PATH}/lib
    ${UNITREE_SDK_PATH}/lib/x86_64     # For x86_64 architecture
    ${UNITREE_SDK_PATH}/lib/aarch64    # For ARM64 architecture
    ${DDS_LIB_PATH}
    /usr/local/lib                     # Common library path
)

# ============= FIND LIBRARIES =============
# Find unitree_sdk2 library
find_library(UNITREE_SDK2_LIB 
    NAMES unitree_sdk2
    PATHS 
        ${UNITREE_SDK_PATH}/lib/${CMAKE_SYSTEM_PROCESSOR}
        ${UNITREE_SDK_PATH}/lib/x86_64
        ${UNITREE_SDK_PATH}/lib/aarch64
        ${UNITREE_SDK_PATH}/lib
    NO_DEFAULT_PATH
)

if(NOT UNITREE_SDK2_LIB)
    message(WARNING "unitree_sdk2 library not found, will try to link anyway")
    set(UNITREE_SDK2_LIB "unitree_sdk2")
else()
    message(STATUS "Found unitree_sdk2: ${UNITREE_SDK2_LIB}")
endif()

# Find DDS libraries
find_library(DDSC_LIB 
    NAMES ddsc
    PATHS 
        ${DDS_LIB_PATH}
        ${UNITREE_SDK_PATH}/lib
        /usr/local/lib
        /usr/lib
)

find_library(DDSCXX_LIB 
    NAMES ddscxx cycloneddsidlcxx
    PATHS 
        ${DDS_LIB_PATH}
        ${UNITREE_SDK_PATH}/lib
        /usr/local/lib
        /usr/lib
)

if(NOT DDSC_LIB)
    message(WARNING "ddsc library not found, will try to link anyway")
    set(DDSC_LIB "ddsc")
else()
    message(STATUS "Found ddsc: ${DDSC_LIB}")
endif()

if(NOT DDSCXX_LIB)
    message(WARNING "ddscxx library not found, will try to link anyway")
    set(DDSCXX_LIB "ddscxx")
else()
    message(STATUS "Found ddscxx: ${DDSCXX_LIB}")
endif()

# ============= MAIN EXECUTABLE =============
add_executable(uwb_path_controller
    src/uwb_path_controller.cpp
)

# Link libraries (order matters!)
target_link_libraries(uwb_path_controller
    ${UNITREE_SDK2_LIB}
    ${DDSC_LIB}
    ${DDSCXX_LIB}
    Threads::Threads
    rt
    m
    dl
)

# Set RPATH for runtime
set_target_properties(uwb_path_controller PROPERTIES
    INSTALL_RPATH "${UNITREE_SDK_PATH}/lib:${DDS_LIB_PATH}:/usr/local/lib"
    BUILD_RPATH "${UNITREE_SDK_PATH}/lib:${DDS_LIB_PATH}:/usr/local/lib"
)

# ============= TEST EXECUTABLE =============
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/test_estimator.cpp")
    add_executable(test_estimator
        test/test_estimator.cpp
    )
    
    target_link_libraries(test_estimator
        ${UNITREE_SDK2_LIB}
        ${DDSC_LIB}
        ${DDSCXX_LIB}
        Threads::Threads
        rt
        m
    )
endif()

# ============= INSTALLATION =============
install(TARGETS uwb_path_controller
    DESTINATION bin
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config/uwb_config.yaml")
    install(FILES config/uwb_config.yaml
        DESTINATION etc/uwb_path_follower
    )
endif()

# ============= STATUS OUTPUT =============
message(STATUS "===========================================")
message(STATUS "UWB Path Follower Configuration Summary:")
message(STATUS "  CMake Version: ${CMAKE_VERSION}")
message(STATUS "  Compiler: ${CMAKE_CXX_COMPILER}")
message(STATUS "  Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "  System: ${CMAKE_SYSTEM_NAME} ${CMAKE_SYSTEM_PROCESSOR}")
message(STATUS "  Unitree SDK: ${UNITREE_SDK_PATH}")
message(STATUS "  DDS Headers: ${DDS_INCLUDE_DIRS}")
message(STATUS "  Libraries:")
message(STATUS "    unitree_sdk2: ${UNITREE_SDK2_LIB}")
message(STATUS "    ddsc: ${DDSC_LIB}")
message(STATUS "    ddscxx: ${DDSCXX_LIB}")
message(STATUS "===========================================")
