cmake_minimum_required(VERSION 3.0)
project(ocs2_anymal_interface)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roslib
  ocs2_anymal_switched_model
  ocs2_quadruped_interface
)

## Eigen3
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

find_package(Boost REQUIRED COMPONENTS
  filesystem
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fopenmp -std=c++11 -Wfatal-errors -ffast-math -fPIC")

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS 
    include
    ${EIGEN_INCLUDE_DIRS}
  
  LIBRARIES 
    ocs2_anymal_interface
    mpc_ros_anymal
    mrt_ros_anymal
    
  CATKIN_DEPENDS
    ocs2_anymal_switched_model
    ocs2_quadruped_interface

  DEPENDS
    Boost
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${EIGEN_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# Declare a C++ library
add_library(ocs2_anymal_interface
  src/OCS2AnymalInterface.cpp
)
add_dependencies(ocs2_anymal_interface
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(ocs2_anymal_interface
  ${catkin_LIBRARIES}
)
#set_property(TARGET ocs2_anymal_interface PROPERTY POSITION_INDEPENDENT_CODE ON)

add_library(mpc_ros_anymal
  src/MPC_ROS_Anymal.cpp
)
add_dependencies(mpc_ros_anymal
  ocs2_anymal_interface
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(mpc_ros_anymal
  ocs2_anymal_interface
  ${catkin_LIBRARIES}
)

add_library(mrt_ros_anymal
  src/MRT_ROS_Anymal.cpp
)
add_dependencies(mrt_ros_anymal
  ocs2_anymal_interface
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(mrt_ros_anymal
  ocs2_anymal_interface
  ${catkin_LIBRARIES}
)

# Declare a C++ Executable
add_executable(target_command_node
  src/AnymalTargetPoseCommand.cpp
)
add_dependencies(target_command_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(target_command_node
  ${catkin_LIBRARIES}
)

add_executable(target_command_node_velocity
  src/AnymalTargetVelocityCommand.cpp
)
add_dependencies(target_command_node_velocity
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(target_command_node_velocity
  ${catkin_LIBRARIES}
)

add_executable(gait_command_node
  src/AnymalModeSequenceCommand.cpp
)
add_dependencies(gait_command_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(gait_command_node
  ${catkin_LIBRARIES}
)

add_executable(planning
        src/Planning.cpp
)
add_dependencies(planning
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(planning
  ocs2_anymal_interface
  ${catkin_LIBRARIES}
)

add_executable(planning
        src/planning.cpp
        )
add_dependencies(planning
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
        )
target_link_libraries(planning
        ocs2_anymal_interface
        ${catkin_LIBRARIES}
        )

add_executable(anymal_mpc_node
  src/AnymalMPC.cpp
)
add_dependencies(anymal_mpc_node
  mpc_ros_anymal
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(anymal_mpc_node
  mpc_ros_anymal
  ${catkin_LIBRARIES}
)

add_executable(dummy_mrt_node
  test/DummyMRT.cpp
)
add_dependencies(dummy_mrt_node
  ocs2_anymal_interface
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(dummy_mrt_node
  ocs2_anymal_interface
  ${catkin_LIBRARIES}
)


#############
## Install ##
#############

install(TARGETS
            ocs2_anymal_interface
            mpc_ros_anymal
            mrt_ros_anymal
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS
            target_command_node
            target_command_node_velocity
            gait_command_node
            planning
            anymal_mpc_node
            dummy_mrt_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ocs2_anymal_interface.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
