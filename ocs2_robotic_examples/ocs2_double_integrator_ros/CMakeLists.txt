cmake_minimum_required(VERSION 3.0.2)
project(ocs2_double_integrator_ros)

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CATKIN_PACKAGE_DEPENDENCIES
  ocs2_core
  ocs2_ddp
  ocs2_mpc
  ocs2_ros_interfaces
  ocs2_double_integrator
  ocs2_robotic_tools
  ocs2_robotic_assets
)

find_package(catkin REQUIRED COMPONENTS
  roslib
  ${CATKIN_PACKAGE_DEPENDENCIES}
)

find_package(Boost REQUIRED COMPONENTS
  system
  filesystem
)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS
    include
    ${EIGEN3_INCLUDE_DIRS}
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
  DEPENDS
    Boost
)

###########
## Build ##
###########

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# Mpc node
add_executable(double_integrator_mpc
  src/DoubleIntegratorMpcNode.cpp
)
add_dependencies(double_integrator_mpc
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(double_integrator_mpc
  ${catkin_LIBRARIES}
)

# Dummy node
add_executable(double_integrator_dummy_test
  src/DummyDoubleIntegratorNode.cpp
  src/DoubleIntegratorDummyVisualization.cpp
)
add_dependencies(double_integrator_dummy_test
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(double_integrator_dummy_test
  ${catkin_LIBRARIES}
)

# Target node
add_executable(double_integrator_target
  src/DoubleIntegratorTargetPoseCommand.cpp
)
add_dependencies(double_integrator_target
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(double_integrator_target
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)
target_compile_options(double_integrator_target PRIVATE ${OCS2_CXX_FLAGS})


#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  message(STATUS "Run clang tooling.")
  add_clang_tooling(
    TARGETS
      double_integrator_mpc
      double_integrator_dummy_test
      double_integrator_target
    SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/test
    CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
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
    double_integrator_mpc
    double_integrator_dummy_test
    double_integrator_target
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
