cmake_minimum_required(VERSION 3.5)
project(chassis_state_machine)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(custom_msgs_srvs REQUIRED)

add_executable(chassis_state_machine_node src/chassis_state_machine_node.cpp)
ament_target_dependencies(chassis_state_machine_node rclcpp geometry_msgs custom_msgs_srvs)

install(TARGETS chassis_state_machine_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
ament_package()
