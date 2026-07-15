cmake_minimum_required(VERSION 3.10)
project(ros2_sync_tool)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# --- sync_analyzer_core library (pure math, no ROS dependency) ---
add_library(sync_analyzer_core
    src/sync_analyzer_core.cpp
)
target_include_directories(sync_analyzer_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# --- sync_analyzer_node executable ---
add_executable(sync_analyzer_node
    src/ros2_sync_analyzer_node.cpp
)
target_link_libraries(sync_analyzer_node sync_analyzer_core)
ament_target_dependencies(sync_analyzer_node rclcpp sensor_msgs)
target_include_directories(sync_analyzer_node PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

install(TARGETS sync_analyzer_node sync_analyzer_core
    DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()