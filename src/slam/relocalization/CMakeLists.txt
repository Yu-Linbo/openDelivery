cmake_minimum_required(VERSION 3.5)
project(relocalization)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(custom_msgs_srvs REQUIRED)
find_package(jsoncpp REQUIRED)

add_library(relocalization_core src/relocalization_core.cpp)
target_include_directories(relocalization_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(relocalization_core jsoncpp_lib)

add_executable(relocalization_node src/relocalization_node.cpp)
target_include_directories(relocalization_node PRIVATE include)
target_link_libraries(relocalization_node relocalization_core)
ament_target_dependencies(relocalization_node
  rclcpp sensor_msgs nav_msgs geometry_msgs tf2 tf2_ros custom_msgs_srvs)

install(TARGETS relocalization_core relocalization_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_relocalization_core test/test_relocalization_core.cpp)
  target_include_directories(test_relocalization_core PRIVATE include)
  target_link_libraries(test_relocalization_core relocalization_core)
endif()

ament_export_include_directories(include)
ament_export_libraries(relocalization_core)
ament_package()
