cmake_minimum_required(VERSION 3.8)
project(simulate)

find_package(ament_cmake REQUIRED)

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
