cmake_minimum_required(VERSION 3.10.2)
project(ocs2_legged_robot)

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ocs2_core REQUIRED)
find_package(ocs2_mpc REQUIRED)
find_package(ocs2_robotic_tools REQUIRED)
find_package(ocs2_oc REQUIRED)
find_package(ocs2_sqp REQUIRED)
find_package(ocs2_thirdparty REQUIRED)
find_package(ocs2_ddp REQUIRED)
find_package(ocs2_msgs REQUIRED)
find_package(ocs2_pinocchio_interface REQUIRED)
find_package(ocs2_centroidal_model REQUIRED)
find_package(ocs2_ros_interfaces REQUIRED)
find_package(ocs2_robotic_assets REQUIRED)
find_package(OpenMP REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)

set(AMENT_PACKAGE_DEPENDENCIES
  ocs2_core
  ocs2_oc
  ocs2_ddp
  ocs2_mpc
  ocs2_sqp
  ocs2_robotic_tools
  ocs2_pinocchio_interface
  ocs2_centroidal_model
  ocs2_robotic_assets
  ocs2_thirdparty
  ocs2_msgs
  ocs2_ros_interfaces
  rclcpp
  kdl_parser
  tf2
  urdf
  robot_state_publisher
  tf2_ros
  tf2_geometry_msgs
  std_srvs
  # trajectory_msgs
  visualization_msgs
  pinocchio
)


find_package(Boost REQUIRED COMPONENTS
  system
  filesystem
  log
  log_setup
)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

if ( EXISTS "/opt/openrobots/lib/libpinocchio.so")
else()
    message(FATAL_ERROR "Install Pinocchio first. Instructions: https://stack-of-tasks.github.io/pinocchio/download.html")
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fopenmp -std=c++11 -Wfatal-errors")

list(APPEND CMAKE_PREFIX_PATH "/opt/openrobots/")
find_package(pinocchio REQUIRED)

###########
## Build ##
###########

# Resolve for the package path at compile time.
configure_file (
  "${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/package_path.h.in"
  "${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME}/package_path.h" @ONLY
)

set(FLAGS
  ${OCS2_CXX_FLAGS}
  ${pinocchio_CFLAGS_OTHER}
  -Wno-ignored-attributes
  -Wno-invalid-partial-specialization   # to silence warning with unsupported Eigen Tensor
  -DPINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR
  -DPINOCCHIO_URDFDOM_USE_STD_SHARED_PTR
)

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

# Legged robot interface library
add_library(${PROJECT_NAME}
  src/common/ModelSettings.cpp
  src/dynamics/LeggedRobotDynamicsAD.cpp
  src/constraint/EndEffectorLinearConstraint.cpp
  src/constraint/FrictionConeConstraint.cpp
  src/constraint/ZeroForceConstraint.cpp
  src/constraint/NormalVelocityConstraintCppAd.cpp
  src/constraint/ZeroVelocityConstraintCppAd.cpp
  src/cost/LeggedRobotStateInputQuadraticCost.cpp
  src/initialization/LeggedRobotInitializer.cpp
  src/reference_manager/SwitchedModelReferenceManager.cpp
  src/foot_planner/CubicSpline.cpp
  src/foot_planner/SplineCpg.cpp
  src/foot_planner/SwingTrajectoryPlanner.cpp
  src/gait/Gait.cpp
  src/gait/GaitSchedule.cpp
  src/gait/ModeSequenceTemplate.cpp
  src/LeggedRobotInterface.cpp
  src/LeggedRobotPreComputation.cpp
)

set_target_properties(${PROJECT_NAME}
  PROPERTIES INSTALL_RPATH "/opt/openrobots/lib"
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
  ${Boost_LIBRARIES}
  ${pinocchio_LIBRARIES}
  dl
)

ament_target_dependencies(
  ocs2_legged_robot
  ${AMENT_PACKAGE_DEPENDENCIES}
)

target_compile_options(${PROJECT_NAME} PUBLIC ${FLAGS})
target_compile_definitions(${PROJECT_NAME} PRIVATE BOOST_LOG_DYN_LINK=true)

#########################
###   CLANG TOOLING   ###
#########################

find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
   message(STATUS "Run clang tooling for target ocs2_legged_robot")
   add_clang_tooling(
     TARGETS ${PROJECT_NAME}
     SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/test
     CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
     CF_WERROR
)
endif(cmake_clang_tools_FOUND)

#############
## Install ##
#############

# install(TARGETS ${PROJECT_NAME}
#   EXPORT export_${PROJECT_NAME}
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)


install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)

ament_export_libraries(${PROJECT_NAME})

ament_export_targets(
  export_${PROJECT_NAME}
)

ament_export_dependencies(ocs2_core)
ament_export_dependencies(ocs2_mpc)
ament_export_dependencies(ocs2_robotic_tools)
ament_export_dependencies(ocs2_oc)
ament_export_dependencies(ocs2_thirdparty)
ament_export_dependencies(ocs2_ddp)
ament_export_dependencies(ocs2_msgs)
ament_export_dependencies(ocs2_pinocchio_interface)
ament_export_dependencies(ocs2_centroidal_model)
ament_export_dependencies(ocs2_ros_interfaces)
ament_export_dependencies(tf2)
ament_export_dependencies(urdf)
ament_export_dependencies(kdl_parser)
ament_export_dependencies(robot_state_publisher)
ament_export_dependencies(visualization_msgs)
ament_export_dependencies(nav_msgs)
ament_export_dependencies(std_srvs)

#############
## Testing ##
#############

# if(BUILD_TESTING)

# # Include linting tests
# find_package(ament_lint_auto REQUIRED)
# find_package(ament_cmake_gtest REQUIRED)
# ament_lint_auto_find_test_dependencies()

# # test_time_triggered_rollout
# ament_add_gtest(${PROJECT_NAME}_test
#   test/AnymalFactoryFunctions.cpp
#   test/constraint/testEndEffectorLinearConstraint.cpp
#   test/constraint/testFrictionConeConstraint.cpp
#   test/constraint/testZeroForceConstraint.cpp
# )

# target_link_libraries(${PROJECT_NAME}_test
#   ${PROJECT_NAME}
#   ${Boost_LIBRARIES}
#   gtest_main
# )

# target_compile_options(${PROJECT_NAME}_test PRIVATE ${FLAGS})

# target_include_directories(${PROJECT_NAME}_test PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test/include>
#   $<INSTALL_INTERFACE:include>
# )
# endif(BUILD_TESTING)

ament_package()

