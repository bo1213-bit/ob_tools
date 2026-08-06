cmake_minimum_required(VERSION 3.10)
project(ob_tools)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Orbbec SDK — relative to ~/ros2_ws/ob_tools_src/
set(OrbbecSDK_DIR "${CMAKE_CURRENT_LIST_DIR}/../OrbbecSDK_v2.8.7_202606161335_ab8672c_linux_arm64/lib")
find_package(OrbbecSDK REQUIRED)

# OpenCV — 用于把采集的帧保存为 PNG 图像
find_package(OpenCV REQUIRED)

add_executable(timestamp_sync_check
    main.cpp
    data_collector.cpp
    sync_analyzer.cpp
)
target_link_libraries(timestamp_sync_check ob::OrbbecSDK ${OpenCV_LIBS} pthread)
target_include_directories(timestamp_sync_check PRIVATE .)