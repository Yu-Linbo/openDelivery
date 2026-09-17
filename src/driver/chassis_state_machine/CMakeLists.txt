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
set(_PARAMS_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../params")
install(FILES "${_PARAMS_ROOT}/launch/${PROJECT_NAME}/chassis_state_machine.launch.py"
  DESTINATION share/${PROJECT_NAME}/launch)
ament_package()
