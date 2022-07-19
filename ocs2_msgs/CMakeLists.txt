cmake_minimum_required(VERSION 3.10.2)
project(ocs2_msgs)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MPCState.msg"
  "msg/MPCInput.msg"
  "msg/ModeSchedule.msg"
  "msg/MPCObservation.msg"
  "msg/MPCPerformanceIndices.msg"
  "msg/MPCTargetTrajectories.msg"
  "msg/ControllerData.msg"
  "msg/MPCFlattenedController.msg"
  "srv/Reset.srv"
  DEPENDENCIES builtin_interfaces std_msgs
)

ament_export_dependencies(std_msgs)
ament_export_dependencies(rosidl_default_runtime)

ament_package()
