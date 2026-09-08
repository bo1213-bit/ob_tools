cmake_minimum_required(VERSION 3.10)
project(ob_tools)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Orbbec SDK — 指向板子家目录下解压的 SDK（~/OrbbecSDK_v2.9.3_..._linux_arm64/lib）
# 允许命令行 -DOrbbecSDK_DIR=/其他路径/lib 覆盖
if(NOT OrbbecSDK_DIR)
    set(OrbbecSDK_DIR "$ENV{HOME}/OrbbecSDK_v2.9.3_202607151523_2f6561c_linux_arm64/lib")
endif()
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