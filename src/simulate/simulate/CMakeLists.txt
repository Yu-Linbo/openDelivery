cmake_minimum_required(VERSION 3.8)
project(simulate)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_fake_elevator test/test_fake_elevator.py)
endif()

find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_library(ray_collision_filter_plugin SHARED
  src/ray_collision_filter_plugin.cpp
)
target_include_directories(ray_collision_filter_plugin PRIVATE ${GAZEBO_INCLUDE_DIRS})
target_link_libraries(ray_collision_filter_plugin ${GAZEBO_LIBRARIES})
target_compile_options(ray_collision_filter_plugin PRIVATE ${GAZEBO_CXX_FLAGS})

add_library(topdown_camera_control_plugin SHARED
  src/topdown_camera_control_plugin.cpp
)
target_include_directories(topdown_camera_control_plugin PRIVATE ${GAZEBO_INCLUDE_DIRS})
ament_target_dependencies(topdown_camera_control_plugin
  gazebo_ros
  geometry_msgs
  rclcpp
)
target_link_libraries(topdown_camera_control_plugin ${GAZEBO_LIBRARIES})
target_compile_options(topdown_camera_control_plugin PRIVATE ${GAZEBO_CXX_FLAGS})

install(TARGETS ray_collision_filter_plugin topdown_camera_control_plugin
  LIBRARY DESTINATION lib
)

install(PROGRAMS scripts/fake_elevator.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME fake_elevator
)
set(_PARAMS_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../params")
install(FILES "${_PARAMS_ROOT}/launch/${PROJECT_NAME}/simulate.launch.py"
  DESTINATION share/${PROJECT_NAME}/launch)
# Package-local launch files are tests only.
install(DIRECTORY launch urdf worlds model
  DESTINATION share/${PROJECT_NAME}
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/scripts/start_gzweb.sh")
  install(PROGRAMS scripts/start_gzweb.sh
    DESTINATION share/${PROJECT_NAME}/scripts
  )
endif()

# Vendored GzWeb source (downloaded under src/simulate/gzweb) is optional.
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../gzweb/package.json")
  install(DIRECTORY ../gzweb
    DESTINATION share/${PROJECT_NAME}/third_party
  )
endif()

ament_package()
