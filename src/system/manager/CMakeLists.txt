cmake_minimum_required(VERSION 3.5)
project(manager)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(custom_msgs_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ament_index_cpp REQUIRED)

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

add_executable(semantic_location_query_node
  src/semantic_location_query_node.cpp
)
target_include_directories(semantic_location_query_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
ament_target_dependencies(semantic_location_query_node
  rclcpp
  custom_msgs_srvs
  geometry_msgs
)

add_executable(semantic_location_query_example
  examples/semantic_location_query_example.cpp
)
target_include_directories(semantic_location_query_example PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
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
  geometry_msgs
)

add_executable(stack_lifecycle_manager_node
  src/stack_lifecycle_manager_node.cpp
)

target_include_directories(stack_lifecycle_manager_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
ament_target_dependencies(stack_lifecycle_manager_node
  rclcpp
  custom_msgs_srvs
  lifecycle_msgs
  std_srvs
  ament_index_cpp
)

install(TARGETS health_monitor_node semantic_location_query_node semantic_location_query_example task_manager_node stack_lifecycle_manager_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
