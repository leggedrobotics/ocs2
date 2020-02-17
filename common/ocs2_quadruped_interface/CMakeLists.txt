cmake_minimum_required(VERSION 3.0)
project(ocs2_quadruped_interface)

## Catkin Dependencies
set(CATKIN_PACKAGE_DEPENDENCIES
  ocs2_core
  ocs2_ddp
  ocs2_mpc
  ocs2_comm_interfaces
  ocs2_robotic_tools
  ocs2_switched_model_interface
  roscpp
  xpp_msgs
)
find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_PACKAGE_DEPENDENCIES}
)

## Eigen3
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fopenmp -std=c++11 -Wfatal-errors")

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
      ${PROJECT_NAME}
   CATKIN_DEPENDS
      ${CATKIN_PACKAGE_DEPENDENCIES}
   DEPENDS
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
add_library(${PROJECT_NAME}
        src/QuadrupedInterface.cpp
        src/QuadrupedDummyNode.cpp
        src/QuadrupedMpcNode.cpp
        src/QuadrupedXppVisualizer.cpp
        )
add_dependencies(${PROJECT_NAME}
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
        )
target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        )

#############
## Install ##
#############

install(DIRECTORY
            include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
