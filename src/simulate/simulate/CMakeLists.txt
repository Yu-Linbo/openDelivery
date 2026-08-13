cmake_minimum_required(VERSION 3.8)
project(simulate)

find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)

add_library(ray_collision_filter_plugin SHARED
  src/ray_collision_filter_plugin.cpp
)
target_include_directories(ray_collision_filter_plugin PRIVATE ${GAZEBO_INCLUDE_DIRS})
target_link_libraries(ray_collision_filter_plugin ${GAZEBO_LIBRARIES})
target_compile_options(ray_collision_filter_plugin PRIVATE ${GAZEBO_CXX_FLAGS})

install(TARGETS ray_collision_filter_plugin
  LIBRARY DESTINATION lib
)

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
