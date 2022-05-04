cmake_minimum_required(VERSION 3.16)

project(rtk-sender-example CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(MAVSDK REQUIRED)

add_executable(rtk-sender-example
    rtk-sender-example.cpp
    serial-comms.h
    serial-comms.cpp
    definitions.h
    PX4-GPSDrivers/src/gps_helper.h
    PX4-GPSDrivers/src/gps_helper.cpp
    PX4-GPSDrivers/src/ubx.h
    PX4-GPSDrivers/src/ubx.cpp
    PX4-GPSDrivers/src/rtcm.h
    PX4-GPSDrivers/src/rtcm.cpp
)

target_compile_options(rtk-sender-example
    PRIVATE
    -Wall
    -Wextra
)

target_link_libraries(rtk-sender-example
    MAVSDK::mavsdk
)

target_include_directories(rtk-sender-example
    PRIVATE
    PX4-GPSDrivers/src/
)
