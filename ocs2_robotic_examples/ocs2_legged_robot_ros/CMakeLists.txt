cmake_minimum_required(VERSION 3.0.2)
project(ocs2_legged_robot_ros)

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CATKIN_PACKAGE_DEPENDENCIES
  roslib
  cmake_modules
  tf
  urdf
  kdl_parser
  robot_state_publisher
  ocs2_core
  ocs2_oc
  ocs2_ddp
  ocs2_mpc
  ocs2_robotic_tools
  ocs2_pinocchio_interface
  ocs2_centroidal_model
  ocs2_robotic_assets
  ocs2_msgs
  ocs2_ros_interfaces
  ocs2_legged_robot
)

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_PACKAGE_DEPENDENCIES}
)

find_package(Boost REQUIRED COMPONENTS
  system
  filesystem
)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

find_package(PkgConfig REQUIRED)
pkg_check_modules(pinocchio REQUIRED pinocchio)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS
    include
    ${EIGEN3_INCLUDE_DIRS}
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
  DEPENDS
    Boost
    pinocchio
)

###########
## Build ##
###########

include_directories(
  include
  ${pinocchio_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  ${pinocchio_LIBRARY_DIRS}
)

# Mpc node
add_executable(legged_robot_mpc
  src/gait/GaitReceiver.cpp
  src/LeggedRobotMpcNode.cpp
)
add_dependencies(legged_robot_mpc
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(legged_robot_mpc
  ${catkin_LIBRARIES}
)
target_compile_options(legged_robot_mpc PRIVATE ${OCS2_CXX_FLAGS})

# Dummy node
add_executable(legged_robot_dummy
  src/visualization/LeggedRobotVisualizer.cpp
  src/LeggedRobotDummyNode.cpp
)
add_dependencies(legged_robot_dummy
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(legged_robot_dummy
  ${catkin_LIBRARIES}
)
target_compile_options(legged_robot_dummy PRIVATE ${OCS2_CXX_FLAGS})

# Keyboard command node for target trajectories
add_executable(legged_robot_target
  src/LeggedRobotPoseCommandNode.cpp
)
add_dependencies(legged_robot_target
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(legged_robot_target
  ${catkin_LIBRARIES}
)
target_compile_options(legged_robot_target PRIVATE ${OCS2_CXX_FLAGS})

# Keyboard command node for gait
add_executable(legged_robot_gait_command
  src/gait/GaitKeyboardPublisher.cpp
  src/LeggedRobotGaitCommandNode.cpp
)
add_dependencies(legged_robot_gait_command
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(legged_robot_gait_command
  ${catkin_LIBRARIES}
)
target_compile_options(legged_robot_gait_command PRIVATE ${OCS2_CXX_FLAGS})

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
   message(STATUS "Run clang tooling for target ocs2_legged_robot_ros")
   add_clang_tooling(
     TARGETS
       legged_robot_mpc
       legged_robot_dummy
       legged_robot_target
       legged_robot_gait_command
     SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
     CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
     CF_FIX
     CF_WERROR
)
endif(cmake_clang_tools_FOUND)

#############
## Install ##
#############

	install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(
  TARGETS
    legged_robot_mpc
    legged_robot_dummy
    legged_robot_target
    legged_robot_gait_command
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############
