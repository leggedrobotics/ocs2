cmake_minimum_required(VERSION 3.5)
project(ocs2_switched_model_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ocs2_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Gait.msg"
  "msg/GaitSequence.msg"
  "msg/ScheduledGaitSequence.msg"
  "srv/TrajectoryRequest.srv"
  DEPENDENCIES
    ocs2_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
