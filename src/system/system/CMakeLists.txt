cmake_minimum_required(VERSION 3.5)
project(system)

find_package(ament_cmake REQUIRED)

# Production launch sources live only under repo-root params/launch/.
set(_PARAMS_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../params")
install(
  FILES "${_PARAMS_ROOT}/launch/system/startup.launch.py"
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY "${_PARAMS_ROOT}/launch/fake"
  DESTINATION share/${PROJECT_NAME}/bringup_launch
)

# fake_pub simulation script (source lives under src/fake/scripts)
set(_FAKE_PUB "${CMAKE_CURRENT_SOURCE_DIR}/../../fake/scripts/fake_pub.py")
install(
  PROGRAMS "${_FAKE_PUB}"
  DESTINATION lib/${PROJECT_NAME}
  RENAME fake_pub_node
)

install(
  PROGRAMS
  scripts/start_open_delivery.sh
  scripts/sim_bringup.sh
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
