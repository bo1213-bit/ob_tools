cmake_minimum_required(VERSION 3.8)
project(ob_ros2_timestamp_collector)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Threads REQUIRED)

add_library(collector_core src/collector_core.cpp)
target_compile_features(collector_core PUBLIC cxx_std_17)
target_link_libraries(collector_core PUBLIC Threads::Threads)
target_include_directories(collector_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(timestamp_collector_node src/timestamp_collector_node.cpp)
target_link_libraries(timestamp_collector_node collector_core)
target_include_directories(timestamp_collector_node PRIVATE include)
ament_target_dependencies(timestamp_collector_node rclcpp sensor_msgs)

install(TARGETS collector_core timestamp_collector_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config scripts
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_capture_record test/test_capture_record.cpp)
  target_include_directories(test_capture_record PRIVATE include)

  ament_add_gtest(test_collector_core test/test_collector_core.cpp)
  target_link_libraries(test_collector_core collector_core)
  target_include_directories(test_collector_core PRIVATE include)

  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(
    NAME test_three_gmsl_external_trigger
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_three_gmsl_external_trigger.py
  )

  add_test(
    NAME test_analyze_capture_sync
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_analyze_capture_sync.py
  )
endif()

ament_package()
