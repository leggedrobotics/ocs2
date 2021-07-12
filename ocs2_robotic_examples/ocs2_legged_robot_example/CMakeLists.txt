cmake_minimum_required(VERSION 3.0.2)
project(ocs2_legged_robot_example)

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CATKIN_PACKAGE_DEPENDENCIES
  roslib
  cmake_modules
  ocs2_core
  ocs2_mpc
  ocs2_robotic_tools
  ocs2_ddp
  ocs2_pinocchio_interface
  ocs2_centroidal_model
  ocs2_ros_interfaces
  tf
  urdf
  kdl_parser
  robot_state_publisher
  anymal_c_simple_description
)

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_PACKAGE_DEPENDENCIES}
)

find_package(Boost REQUIRED COMPONENTS
  system
  filesystem
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(pinocchio REQUIRED pinocchio)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fopenmp -std=c++11 -Wfatal-errors -Wl,--no-as-needed")

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
    pinocchio
    Boost
)
###########
## Build ##
###########

include_directories(
  include
  test/include
  ${pinocchio_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
link_directories(${pinocchio_LIBRARY_DIRS})

# Legged robot interface library
add_library(ocs2_legged_robot_example
  src/common/ModelSettings.cpp
  src/dynamics/LeggedRobotDynamicsAD.cpp
  src/constraint/EndEffectorLinearConstraint.cpp
  src/constraint/FrictionConeConstraint.cpp
  src/constraint/ZeroForceConstraint.cpp
  src/constraint/NormalVelocityConstraintCppAd.cpp
  src/constraint/ZeroVelocityConstraintCppAd.cpp
  src/cost/LeggedRobotStateInputQuadraticCost.cpp
  src/initialization/LeggedRobotInitializer.cpp
  src/synchronized_module/SwitchedModelReferenceManager.cpp
  src/foot_planner/SplineCpg.cpp
  src/foot_planner/CubicSpline.cpp
  src/foot_planner/SwingTrajectoryPlanner.cpp
  src/gait/Gait.cpp
  src/gait/GaitReceiver.cpp
  src/gait/GaitSchedule.cpp
  src/gait/ModeSequenceTemplate.cpp
  src/LeggedRobotInterface.cpp
  src/LeggedRobotPreComputation.cpp
  src/visualization/LeggedRobotVisualizer.cpp
)

add_dependencies(ocs2_legged_robot_example
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(ocs2_legged_robot_example
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${pinocchio_LIBRARIES}
  dl
)
target_compile_options(ocs2_legged_robot_example PUBLIC ${pinocchio_CFLAGS_OTHER} -DPINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR -DPINOCCHIO_URDFDOM_USE_STD_SHARED_PTR)

# Mpc node
add_executable(legged_robot_mpc
  src/LeggedRobotMpcNode.cpp
)
add_dependencies(legged_robot_mpc
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(legged_robot_mpc
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)
target_compile_options(legged_robot_mpc PUBLIC ${pinocchio_CFLAGS_OTHER})

# Dummy node
add_executable(legged_robot_dummy
  src/LeggedRobotDummyNode.cpp
)
add_dependencies(legged_robot_dummy
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(legged_robot_dummy
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
target_compile_options(legged_robot_dummy PUBLIC ${pinocchio_CFLAGS_OTHER})

# Keyboard Command node for legged robot walking (gait selection and target selection)
add_executable(legged_robot_target
  src/LeggedRobotPoseCommandNode.cpp
)
add_dependencies(legged_robot_target
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(legged_robot_target
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
target_compile_options(legged_robot_target PUBLIC ${pinocchio_CFLAGS_OTHER})

add_executable(legged_robot_gait_command
  src/gait/Gait.cpp
  src/gait/ModeSequenceTemplate.cpp
  src/command/LeggedRobotModeSequenceKeyboard.cpp
  src/LeggedRobotGaitCommandNode.cpp
)
add_dependencies(legged_robot_gait_command
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(legged_robot_gait_command
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
target_compile_options(legged_robot_gait_command PUBLIC ${pinocchio_CFLAGS_OTHER})

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
   message(STATUS "Run clang tooling for target ocs2_legged_robot_example")
   add_clang_tooling(
     TARGETS
     legged_robot_mpc
     legged_robot_dummy
     legged_robot_target
     legged_robot_gait_command
     SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/test
     CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
     CF_WERROR
)
endif(cmake_clang_tools_FOUND)

#############
## Install ##
#############
install(TARGETS ocs2_legged_robot_example
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(TARGETS
  legged_robot_mpc
  legged_robot_dummy
  legged_robot_target
  legged_robot_gait_command
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch config rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############
catkin_add_gtest(${PROJECT_NAME}_test
  test/AnymalFactoryFunctions.cpp
  test/constraint/testEndEffectorLinearConstraint.cpp
  test/constraint/testFrictionConeConstraint.cpp
  test/constraint/testZeroForceConstraint.cpp
)
target_link_libraries(${PROJECT_NAME}_test
  gtest_main
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)
