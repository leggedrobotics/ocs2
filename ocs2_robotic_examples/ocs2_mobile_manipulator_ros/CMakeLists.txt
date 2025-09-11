cmake_minimum_required(VERSION 3.14.4)
project(ocs2_mobile_manipulator_ros LANGUAGES CXX)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter -Wno-unused-variable -Wno-return-type -Wno-sign-compare -Wno-deprecated-declarations -Wno-reorder)
endif()

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(dependencies
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  urdf
  kdl_parser
  robot_state_publisher
  visualization_msgs
  geometry_msgs
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
## Find ament macros and libraries
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
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


find_package(PkgConfig REQUIRED)
pkg_check_modules(pinocchio REQUIRED pinocchio)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(hpp-fcl REQUIRED)
find_package(pinocchio REQUIRED)
find_package(urdf REQUIRED)
find_package(urdfdom REQUIRED)

###########
## Build ##
###########

set(FLAGS
  ${OCS2_CXX_FLAGS}
  ${pinocchio_CFLAGS_OTHER}
  -Wno-ignored-attributes
  -Wno-invalid-partial-specialization   # to silence warning with unsupported Eigen Tensor
  -DPINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR
  -DPINOCCHIO_URDFDOM_USE_STD_SHARED_PTR
)


link_directories(
  ${pinocchio_LIBRARY_DIRS}
)

# MPC node
add_executable(mobile_manipulator_mpc_node
  src/MobileManipulatorMpcNode.cpp
)

target_include_directories(mobile_manipulator_mpc_node
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

target_compile_options(mobile_manipulator_mpc_node PUBLIC ${FLAGS})

# Use ament for dependency management
ament_target_dependencies(mobile_manipulator_mpc_node ${dependencies})

# DistanceVisualization node
add_executable(mobile_manipulator_distance_visualization
  src/MobileManipulatorDistanceVisualization.cpp
)

target_compile_options(mobile_manipulator_distance_visualization PUBLIC ${FLAGS})

# Use ament for dependency management
ament_target_dependencies(mobile_manipulator_distance_visualization ${dependencies})


# Dummy node
add_executable(mobile_manipulator_dummy_mrt_node
  src/MobileManipulatorDummyMRT.cpp
  src/MobileManipulatorDummyVisualization.cpp
)

target_include_directories(mobile_manipulator_dummy_mrt_node
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

target_compile_options(mobile_manipulator_dummy_mrt_node PUBLIC ${FLAGS})

# Use ament for dependency management
ament_target_dependencies(mobile_manipulator_dummy_mrt_node ${dependencies})

# Target node
add_executable(mobile_manipulator_target
  src/MobileManipulatorTarget.cpp
)
target_compile_options(mobile_manipulator_target PUBLIC ${FLAGS})

# Use ament for dependency management
ament_target_dependencies(mobile_manipulator_target ${dependencies})

####################
## Clang tooling ###
####################

find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
  message(STATUS "Run clang tooling")
  add_clang_tooling(
    TARGETS mobile_manipulator_mpc_node mobile_manipulator_dummy_mrt_node
    SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
    CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
    CF_WERROR
  )
endif (cmake_clang_tools_FOUND)

#############
## Install ##
#############

# Install headers
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# Install executables
install(
  TARGETS 
    mobile_manipulator_mpc_node 
    mobile_manipulator_distance_visualization
    mobile_manipulator_dummy_mrt_node 
    mobile_manipulator_target
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files and other resources
install(
  DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
