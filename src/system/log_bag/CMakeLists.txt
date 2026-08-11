cmake_minimum_required(VERSION 3.5)
project(log_bag)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)

add_executable(robot_log_recorder
  src/robot_log_recorder.cpp
)
ament_target_dependencies(robot_log_recorder
  rclcpp
  rcl_interfaces
)

install(TARGETS robot_log_recorder
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/repair_rosbag2.py
  DESTINATION share/${PROJECT_NAME}/scripts)

ament_package()
