cmake_minimum_required(VERSION 3.5)
project(log_bag)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(custom_msgs_srvs REQUIRED)
find_package(SQLite3 REQUIRED)

add_executable(robot_log_recorder
  src/robot_log_recorder.cpp
)
target_include_directories(robot_log_recorder PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
ament_target_dependencies(robot_log_recorder
  rclcpp
  rcl_interfaces
  custom_msgs_srvs
)
target_link_libraries(robot_log_recorder SQLite::SQLite3)

if(BUILD_TESTING)
  enable_testing()
  add_executable(test_log_bag_retention test/test_retention_policy.cpp)
  target_include_directories(test_log_bag_retention PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  add_test(NAME test_log_bag_retention COMMAND test_log_bag_retention)
endif()

install(TARGETS robot_log_recorder
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/repair_rosbag2.py
  DESTINATION share/${PROJECT_NAME}/scripts)

ament_package()
