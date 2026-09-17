cmake_minimum_required(VERSION 3.5)
project(nav_bringup)

find_package(ament_cmake REQUIRED)

set(_PARAMS_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../params")
install(DIRECTORY "${_PARAMS_ROOT}/launch/${PROJECT_NAME}/"
  DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
