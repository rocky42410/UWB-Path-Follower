cmake_minimum_required(VERSION 3.10)
project(uwb_path_follower)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -O2")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

find_package(Threads REQUIRED)

set(UNITREE_SDK_PATH "~/unitree_sdk-main" CACHE PATH "Path to Unitree SDK")
if(NOT EXISTS ${UNITREE_SDK_PATH})
    message(WARNING "UNITREE_SDK_PATH not found. Set with -DUNITREE_SDK_PATH=/your/path")
endif()

include_directories(
    ${UNITREE_SDK_PATH}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

link_directories(
    ${UNITREE_SDK_PATH}/lib
)

add_executable(uwb_path_controller
    src/uwb_path_controller.cpp
)

target_link_libraries(uwb_path_controller
    unitree_sdk2
    ddsc
    Threads::Threads
    m
)

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
endif()

install(TARGETS uwb_path_controller
    DESTINATION bin
)

install(FILES config/uwb_config.yaml
    DESTINATION etc/uwb_path_follower
)
