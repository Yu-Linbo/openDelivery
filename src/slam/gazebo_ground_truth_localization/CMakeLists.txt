cmake_minimum_required(VERSION 3.8)
project(gazebo_ground_truth_localization)

find_package(ament_cmake REQUIRED)

install(PROGRAMS scripts/gazebo_ground_truth_localizer.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME gazebo_ground_truth_localizer
)
install(PROGRAMS scripts/ground_truth_math.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_ground_truth_math test/test_ground_truth_math.py)
  ament_add_pytest_test(test_initial_pose_sync test/test_initial_pose_sync.py)
endif()

ament_package()
