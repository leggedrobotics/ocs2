cmake_minimum_required(VERSION 3.5)
project(ocs2_legged_robot_ros)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(robot_state_publisher REQUIRED)

find_package(ocs2_core REQUIRED)
find_package(ocs2_oc REQUIRED)
find_package(ocs2_ddp REQUIRED)
find_package(ocs2_mpc REQUIRED)
find_package(ocs2_sqp REQUIRED)
find_package(ocs2_ipm REQUIRED)
find_package(ocs2_robotic_tools REQUIRED)
find_package(ocs2_pinocchio_interface REQUIRED)
find_package(ocs2_centroidal_model REQUIRED)
find_package(ocs2_robotic_assets REQUIRED)
find_package(ocs2_msgs REQUIRED)
find_package(ocs2_ros_interfaces REQUIRED)
find_package(ocs2_legged_robot REQUIRED)

find_package(pinocchio REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Boost REQUIRED COMPONENTS system filesystem log_setup log)

add_library(${PROJECT_NAME}
  src/gait/GaitKeyboardPublisher.cpp
  src/gait/GaitReceiver.cpp
  src/visualization/LeggedRobotVisualizer.cpp
)
target_link_libraries(${PROJECT_NAME}
  pinocchio::pinocchio
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIRS}
    ${pinocchio_INCLUDE_DIRS}
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  tf2_ros
  sensor_msgs
  geometry_msgs
  visualization_msgs
  urdf
  kdl_parser
  robot_state_publisher
  ocs2_core
  ocs2_oc
  ocs2_ddp
  ocs2_mpc
  ocs2_sqp
  ocs2_ipm
  ocs2_robotic_tools
  ocs2_pinocchio_interface
  ocs2_centroidal_model
  ocs2_robotic_assets
  ocs2_msgs
  ocs2_ros_interfaces
  ocs2_legged_robot
)
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_PINOCCHIO_FLAGS})

add_executable(legged_robot_ddp_mpc
  src/LeggedRobotDdpMpcNode.cpp
)
target_link_libraries(legged_robot_ddp_mpc ${PROJECT_NAME})
target_compile_options(legged_robot_ddp_mpc PRIVATE ${OCS2_PINOCCHIO_FLAGS})

add_executable(legged_robot_sqp_mpc
  src/LeggedRobotSqpMpcNode.cpp
)
target_link_libraries(legged_robot_sqp_mpc ${PROJECT_NAME})
target_compile_options(legged_robot_sqp_mpc PRIVATE ${OCS2_PINOCCHIO_FLAGS})

add_executable(legged_robot_ipm_mpc
  src/LeggedRobotIpmMpcNode.cpp
)
target_link_libraries(legged_robot_ipm_mpc ${PROJECT_NAME})
target_compile_options(legged_robot_ipm_mpc PRIVATE ${OCS2_PINOCCHIO_FLAGS})

add_executable(legged_robot_dummy
  src/LeggedRobotDummyNode.cpp
)
target_link_libraries(legged_robot_dummy ${PROJECT_NAME})
target_compile_options(legged_robot_dummy PRIVATE ${OCS2_PINOCCHIO_FLAGS})

add_executable(legged_robot_target
  src/LeggedRobotPoseCommandNode.cpp
)
target_link_libraries(legged_robot_target ${PROJECT_NAME})
target_compile_options(legged_robot_target PRIVATE ${OCS2_PINOCCHIO_FLAGS})

add_executable(legged_robot_gait_command
  src/LeggedRobotGaitCommandNode.cpp
)
target_link_libraries(legged_robot_gait_command ${PROJECT_NAME})
target_compile_options(legged_robot_gait_command PRIVATE ${OCS2_PINOCCHIO_FLAGS})

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS
  legged_robot_ddp_mpc
  legged_robot_sqp_mpc
  legged_robot_ipm_mpc
  legged_robot_dummy
  legged_robot_target
  legged_robot_gait_command
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)
install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  EXPORT export_${PROJECT_NAME}
  DESTINATION share/${PROJECT_NAME}/cmake
)

ament_export_include_directories(
  include
  ${pinocchio_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  tf2_ros
  sensor_msgs
  geometry_msgs
  visualization_msgs
  urdf
  kdl_parser
  robot_state_publisher
  ocs2_core
  ocs2_oc
  ocs2_ddp
  ocs2_mpc
  ocs2_sqp
  ocs2_ipm
  ocs2_robotic_tools
  ocs2_pinocchio_interface
  ocs2_centroidal_model
  ocs2_robotic_assets
  ocs2_msgs
  ocs2_ros_interfaces
  ocs2_legged_robot
  pinocchio
  Eigen3
  Boost
)

ament_package()
