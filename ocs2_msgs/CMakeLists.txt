cmake_minimum_required(VERSION 3.14.4)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
project(ocs2_msgs)

find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/MpcState.msg"
        "msg/MpcInput.msg"
        "msg/ModeSchedule.msg"
        "msg/MpcObservation.msg"
        "msg/MpcPerformanceIndices.msg"
        "msg/MpcTargetTrajectories.msg"
        "msg/ControllerData.msg"
        "msg/MpcFlattenedController.msg"
        "msg/LagrangianMetrics.msg"
        "msg/Multiplier.msg"
        "msg/Constraint.msg"
        "srv/Reset.srv"
        DEPENDENCIES
            std_msgs
            builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
