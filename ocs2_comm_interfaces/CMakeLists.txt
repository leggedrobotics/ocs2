cmake_minimum_required(VERSION 2.8.3)
project(ocs2_comm_interfaces)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  message_generation
  message_runtime
  std_msgs
  ocs2_core
  ocs2_slq
  ocs2_ocs2
  ocs2_mpc
)

## Eigen3
set( EIGEN3_INCLUDE_DIRS "$ENV{LOCAL_EIGEN3_INCLUDE_DIR}" )
if( NOT EIGEN3_INCLUDE_DIRS )
    find_package(Eigen3 REQUIRED)
    message( WARNING "Uses Globally installed Eigen since LOCAL_EIGEN3_INCLUDE_DIR is not defined!")
endif()

find_package(Boost REQUIRED COMPONENTS
  filesystem
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fopenmp -std=c++11 -Wfatal-errors -Wl,--no-as-needed")

###################################
## Generate messages 
###################################
## Add messages
add_message_files(
  DIRECTORY 
    include/ocs2_comm_interfaces/ocs2_ros_interfaces/msg
  FILES
    dummy.msg
    mpc_state.msg
    mpc_input.msg
    mpc_observation.msg
    mode_sequence.msg
    slq_controller.msg
    mpc_feedback_policy.msg
    mpc_feedforward_policy.msg
    mpc_target_trajectories.msg
)

## Add services
add_service_files(
  DIRECTORY 
    include/ocs2_comm_interfaces/ocs2_ros_interfaces/srv
  FILES
    reset.srv
)

## Generate added messages and services
generate_messages(
  DEPENDENCIES
    std_msgs
)


###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS 
    include 
    test/include
#  LIBRARIES ocs2_quadruped_interface
  CATKIN_DEPENDS 
    roscpp
    message_generation 
    message_runtime 
    std_msgs
    ocs2_core 
    ocs2_slq 
    ocs2_ocs2 
    ocs2_mpc 
  DEPENDS 
    boost 
    eigen3
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  test/include
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

#############
## Doxygen ##
#############

find_package(Doxygen)
if(DOXYGEN_FOUND)

    set(doxyfile_in ${CMAKE_CURRENT_SOURCE_DIR}/doc/ocs2_comm_interfaces_doxyfile.in)
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
## $ catkin_test_results ../../../build/ocs2_comm_interfaces

# catkin_add_gtest(${PROJECT_NAME}-test test/test_ocs2_comm_interfaces.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
