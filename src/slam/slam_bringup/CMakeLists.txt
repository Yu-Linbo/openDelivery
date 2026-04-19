cmake_minimum_required(VERSION 3.8)
project(slam_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

add_executable(managed_slam_lifecycle src/managed_slam_lifecycle.cpp)
ament_target_dependencies(managed_slam_lifecycle
  ament_index_cpp
  rclcpp
  rclcpp_lifecycle
)

install(TARGETS managed_slam_lifecycle
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
