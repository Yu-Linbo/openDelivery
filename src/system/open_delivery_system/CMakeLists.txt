cmake_minimum_required(VERSION 3.5)
project(open_delivery_system)

find_package(ament_cmake REQUIRED)

# Repo-root params/ — install so `ros2 launch open_delivery_system startup.launch.py` works on Foxy.
set(_PARAMS_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../params")
install(
  FILES "${_PARAMS_ROOT}/startup.launch.py"
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY
    "${_PARAMS_ROOT}/launch/system"
    "${_PARAMS_ROOT}/launch/fake"
    "${_PARAMS_ROOT}/launch/simulate"
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
  PROGRAMS scripts/start_open_delivery.sh
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
