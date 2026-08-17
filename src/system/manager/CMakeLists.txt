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
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs)
find_package(jsoncpp REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_task_manager_elevator_plan
    test/test_task_manager_elevator_plan.cpp
    src/task_manager_node.cpp
  )
  target_compile_definitions(test_task_manager_elevator_plan PRIVATE
    MANAGER_TASK_MANAGER_NO_MAIN
  )
  target_include_directories(test_task_manager_elevator_plan PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  ament_target_dependencies(test_task_manager_elevator_plan
    rclcpp
    custom_msgs_srvs
    geometry_msgs
  )
  target_link_libraries(test_task_manager_elevator_plan jsoncpp_lib)
endif()

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
  tf2
  tf2_ros
)
target_include_directories(semantic_location_query_node PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(semantic_location_query_node
  ${OpenCV_LIBRARIES}
  jsoncpp_lib)

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
target_link_libraries(task_manager_node jsoncpp_lib)

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
