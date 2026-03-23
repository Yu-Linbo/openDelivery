cmake_minimum_required(VERSION 3.5)
project(open_delivery_rtabmap)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/map_open_delivery.sh
  scripts/save_map_open_delivery.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
