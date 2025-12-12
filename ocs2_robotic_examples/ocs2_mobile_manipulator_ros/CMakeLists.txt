cmake_minimum_required(VERSION 3.5)
project(ocs2_mobile_manipulator_ros)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(robot_state_publisher REQUIRED)

find_package(ocs2_ros_interfaces REQUIRED)
find_package(ocs2_core REQUIRED)
find_package(ocs2_ddp REQUIRED)
find_package(ocs2_mpc REQUIRED)
find_package(ocs2_robotic_tools REQUIRED)
find_package(ocs2_robotic_assets REQUIRED)
find_package(ocs2_pinocchio_interface REQUIRED)
find_package(ocs2_self_collision REQUIRED)
find_package(ocs2_self_collision_visualization REQUIRED)
find_package(ocs2_mobile_manipulator REQUIRED)

find_package(pinocchio REQUIRED)
find_package(hpp-fcl REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Boost REQUIRED COMPONENTS system filesystem log_setup log)

set(dependencies
  rclcpp
  tf2_ros
  sensor_msgs
  geometry_msgs
  visualization_msgs
  urdf
  kdl_parser
  robot_state_publisher
  ocs2_ros_interfaces
  ocs2_core
  ocs2_ddp
  ocs2_mpc
  ocs2_robotic_tools
  ocs2_robotic_assets
  ocs2_pinocchio_interface
  ocs2_self_collision
  ocs2_self_collision_visualization
  ocs2_mobile_manipulator
)

add_executable(mobile_manipulator_mpc_node
  src/MobileManipulatorMpcNode.cpp
)
target_include_directories(mobile_manipulator_mpc_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(mobile_manipulator_mpc_node ${dependencies})
target_compile_options(mobile_manipulator_mpc_node PRIVATE ${OCS2_PINOCCHIO_FLAGS})

add_executable(mobile_manipulator_distance_visualization
  src/MobileManipulatorDistanceVisualization.cpp
)
target_include_directories(mobile_manipulator_distance_visualization PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(mobile_manipulator_distance_visualization ${dependencies})
target_compile_options(mobile_manipulator_distance_visualization PRIVATE ${OCS2_PINOCCHIO_FLAGS})

add_executable(mobile_manipulator_dummy_mrt_node
  src/MobileManipulatorDummyMRT.cpp
  src/MobileManipulatorDummyVisualization.cpp
)
target_include_directories(mobile_manipulator_dummy_mrt_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(mobile_manipulator_dummy_mrt_node ${dependencies})
target_compile_options(mobile_manipulator_dummy_mrt_node PRIVATE ${OCS2_PINOCCHIO_FLAGS})

add_executable(mobile_manipulator_target
  src/MobileManipulatorTarget.cpp
)
target_include_directories(mobile_manipulator_target PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(mobile_manipulator_target ${dependencies})
target_compile_options(mobile_manipulator_target PRIVATE ${OCS2_PINOCCHIO_FLAGS})

#############
## Install ##
#############

install(TARGETS
  mobile_manipulator_mpc_node
  mobile_manipulator_distance_visualization
  mobile_manipulator_dummy_mrt_node
  mobile_manipulator_target
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
