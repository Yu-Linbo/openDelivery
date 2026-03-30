cmake_minimum_required(VERSION 3.5)
project(heartbeat)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(custom_msgs_srvs REQUIRED)

add_executable(heartbeat_node
  src/main.cpp
  src/heartbeat_node.cpp
)
target_include_directories(heartbeat_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
ament_target_dependencies(heartbeat_node
  rclcpp
  custom_msgs_srvs
)

install(TARGETS heartbeat_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
