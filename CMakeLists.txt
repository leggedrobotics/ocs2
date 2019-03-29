cmake_minimum_required(VERSION 3.0)
project(ocs2_quadruped_interface)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  ocs2_core
  ocs2_slq
  ocs2_mpc
  ocs2_comm_interfaces
  ocs2_robotic_tools
  ocs2_switched_model_interface
  rosbag
  xpp_msgs
)

## Eigen3
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

find_package(Boost REQUIRED COMPONENTS
  filesystem
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fopenmp -std=c++11 -Wfatal-errors -ffast-math")

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
      test/include
      ${EIGEN_INCLUDE_DIRS}
#  LIBRARIES ocs2_quadruped_interface
   CATKIN_DEPENDS
      ocs2_core
      ocs2_slq
      ocs2_mpc
      ocs2_comm_interfaces
      ocs2_robotic_tools
      ocs2_switched_model_interface
      rosbag
      xpp_msgs
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
  test/include
  ${EIGEN_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

install(DIRECTORY
            include/${PROJECT_NAME}/
            test/include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
