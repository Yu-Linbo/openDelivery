cmake_minimum_required(VERSION 3.5)
project(manager)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(custom_msgs_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(health_monitor_node
  src/health_monitor_node.cpp
)
target_include_directories(health_monitor_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
ament_target_dependencies(health_monitor_node
  rclcpp
  custom_msgs_srvs
  geometry_msgs
)

add_executable(task_manager_node
  src/task_manager_node.cpp
)
target_include_directories(task_manager_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
ament_target_dependencies(task_manager_node
  rclcpp
  custom_msgs_srvs
)

install(TARGETS health_monitor_node task_manager_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
