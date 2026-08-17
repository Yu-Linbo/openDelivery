cmake_minimum_required(VERSION 3.5)
project(custom_msgs_srvs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotStatus.msg"
  "msg/LocalizeNavCommand.msg"
  "msg/StackComponentState.msg"
  "msg/StackLifecycle.msg"
  "msg/TaskInfo.msg"
  "msg/TaskStatus.msg"
  "msg/TaskCommand.msg"
  "msg/ElevatorInfo.msg"
  "msg/ElevatorStatus.msg"
  "msg/ElevatorCommand.msg"
  "srv/SetHeartbeatParams.srv"
  "srv/SetRobotTask.srv"
  "srv/QuerySemanticLocation.srv"
  "srv/SetStackLifecycleTransition.srv"
  "srv/RecordRelocalization.srv"
  "srv/Relocalize.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
