cmake_minimum_required(VERSION 2.8.3)
project(ocs2_robotic_examples)

find_package(catkin REQUIRED COMPONENTS 
  cmake_modules
  tf
  urdf
  kdl_parser
  robot_state_publisher
  ocs2_core
  ocs2_slq 
  ocs2_ocs2
  ocs2_mpc
  ocs2_comm_interfaces
)
find_package(Boost REQUIRED COMPONENTS 
  system
  filesystem 
)

## Eign3
set( EIGEN3_INCLUDE_DIRS "$ENV{LOCAL_EIGEN3_INCLUDE_DIR}" )
if( NOT EIGEN3_INCLUDE_DIRS )
    find_package(Eigen3 REQUIRED)
    message( WARNING "Uses Globally installed Eigen since LOCAL_EIGEN3_INCLUDE_DIR is not defined!")
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fopenmp -std=c++11 -Wfatal-errors -Wl,--no-as-needed")

## Set PACKAGE_PATH
set(PACKAGE_PATH ${${PROJECT_NAME}_SOURCE_DIR})
configure_file(include/pathfile.h.in pathfile.h)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS 
    include
  CATKIN_DEPENDS
    tf
    urdf
    kdl_parser
    robot_state_publisher
    ocs2_core
    ocs2_slq 
    ocs2_ocs2
    ocs2_mpc
    ocs2_comm_interfaces
  DEPENDS 
    boost 
    eigen3 
)

###########
## Build ##
###########

include_directories(
  include
  include/ocs2_robotic_examples/rbd_libraries/robcogen
  ${CMAKE_CURRENT_BINARY_DIR}
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

#############
## Doxygen ##
#############

find_package(Doxygen)
if(DOXYGEN_FOUND)

    set(doxyfile_in ${CMAKE_CURRENT_SOURCE_DIR}/doc/ocs2_robotic_examples_doxyfile.in)
    set(doxyfile ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile)

    configure_file(${doxyfile_in} ${doxyfile} @ONLY)

    add_custom_target(doc
            COMMAND ${CMAKE_COMMAND} -E echo_append "Building OCS2 Documentation..."
            COMMAND ${DOXYGEN_EXECUTABLE} ${doxyfile}
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/doc
            COMMAND ${CMAKE_COMMAND} -E echo_append "OCS2 Documentation built in ${CMAKE_CURRENT_SOURCE_DIR}/doc"
            VERBATIM)
endif()

#############
## Testing ##
#############
## Info ==============================
## to run tests, cd package folder and run
## $ catkin build -DCMAKE_BUILD_TYPE=RelWithDebInfo --this
## $ catkin run_tests --no-deps --this
## to see the summary of unit test results run
## $ catkin_test_results ../../../build/ocs2_ocs2

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide

###############
## Quadrotor ##
###############

add_executable(quadrotor_mpc
  src/quadrotor/QuadrotorInterface.cpp
  src/quadrotor/MPC_Quadrotor_Node.cpp
)
add_dependencies(quadrotor_mpc
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(quadrotor_mpc
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(quadrotor_dummy_test
  src/quadrotor/QuadrotorInterface.cpp
  src/quadrotor/Dummy_Quadrotor_Node.cpp
)
add_dependencies(quadrotor_dummy_test
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(quadrotor_dummy_test
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(quadrotor_target
  src/quadrotor/QuadrotorTargetPoseCommand.cpp
)
add_dependencies(quadrotor_target
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(quadrotor_target
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

#############
## Ballbot ##
#############

add_executable(ballbot_mpc
  src/ballbot/BallbotInterface.cpp
  src/ballbot/MPC_Ballbot_Node.cpp
)
add_dependencies(ballbot_mpc
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(ballbot_mpc
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  dl
)

add_executable(ballbot_dummy_test
  src/ballbot/BallbotInterface.cpp
  src/ballbot/Dummy_Ballbot_Node.cpp
)
add_dependencies(ballbot_dummy_test
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(ballbot_dummy_test
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  dl
)

add_executable(ballbot_target
  src/ballbot/BallbotTargetPoseCommand.cpp
)
add_dependencies(ballbot_target
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(ballbot_target
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

#######################
## Double Integrator ##
#######################

add_executable(double_integrator_mpc
  src/double_integrator/DoubleIntegratorInterface.cpp
  src/double_integrator/MPC_Double_Integrator_Node.cpp
)
add_dependencies(double_integrator_mpc
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(double_integrator_mpc
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  dl
)

add_executable(double_integrator_dummy_test
  src/double_integrator/DoubleIntegratorInterface.cpp
  src/double_integrator/Dummy_Double_Integrator_Node.cpp
)
add_dependencies(double_integrator_dummy_test
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(double_integrator_dummy_test
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  dl
)

add_executable(double_integrator_target
  src/double_integrator/DoubleIntegratorTargetPoseCommand.cpp
)
add_dependencies(double_integrator_target
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(double_integrator_target
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

###############
## Cart Pole ##
###############

add_executable(cartpole_mpc
  src/cart_pole/CartPoleInterface.cpp
  src/cart_pole/MPC_Cartpole_Node.cpp
)
add_dependencies(cartpole_mpc
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(cartpole_mpc
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  dl
)

add_executable(cartpole_dummy_test
  src/cart_pole/CartPoleInterface.cpp
  src/cart_pole/Dummy_Cartpole_Node.cpp
)
add_dependencies(cartpole_dummy_test
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(cartpole_dummy_test
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  dl
)

