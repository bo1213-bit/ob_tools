cmake_minimum_required(VERSION 3.10)
project(ob_tools)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---- Locate the Orbbec SDK ----
if(NOT DEFINED OrbbecSDK_DIR)
    set(OrbbecSDK_DIR "${CMAKE_CURRENT_LIST_DIR}/../OrbbecSDK_v2.8.7_202606161335_ab8672c_linux_arm64/lib")
endif()
find_package(OrbbecSDK REQUIRED)

# ---- Tool 1: Timestamp Sync Check ----
add_executable(timestamp_sync_check
    src/main.cpp
    src/data_collector.cpp
    src/sync_analyzer.cpp
)
target_link_libraries(timestamp_sync_check ${OrbbecSDK_LIBRARIES} pthread)
target_include_directories(timestamp_sync_check PRIVATE ${OrbbecSDK_INCLUDE_DIRS} src)

# ---- Tool 2: FPS Quality Test ----
add_executable(fps_quality_test fps_quality_test.cpp)
target_link_libraries(fps_quality_test ${OrbbecSDK_LIBRARIES} pthread)
target_include_directories(fps_quality_test PRIVATE ${OrbbecSDK_INCLUDE_DIRS})
