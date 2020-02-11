cmake_minimum_required(VERSION 3.0)
project(ocs2_quadruped_augmented_interface)

## Catkin Dependencies
set(CATKIN_PACKAGE_DEPENDENCIES
  ocs2_core
  ocs2_quadruped_interface
)
find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_PACKAGE_DEPENDENCIES}
)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fopenmp -std=c++11 -Wfatal-errors")

###################################
## catkin specific configuration ##
###################################
catkin_package(
   INCLUDE_DIRS
        include
        ${EIGEN_INCLUDE_DIRS}
   CATKIN_DEPENDS
        ${CATKIN_PACKAGE_DEPENDENCIES}
   DEPENDS
)

###########
## Build ##
###########

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

install(DIRECTORY
        include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

