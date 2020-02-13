cmake_minimum_required(VERSION 3.0)
project(ocs2_anymal_bear_loopshaping)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roslib
        ocs2_anymal_bear_switched_model
        ocs2_quadruped_interface
        )

## Eigen3
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

find_package(Boost REQUIRED COMPONENTS
        filesystem
        )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fopenmp -std=c++11 -Wfatal-errors")

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
    roslib
    ocs2_anymal_bear_switched_model
    ocs2_quadruped_interface
DEPENDS
    Boost
)

###########
## Build ##
###########

include_directories(
        include
        ${EIGEN3_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
)

# Declare a C++ library
add_library(${PROJECT_NAME}
        src/AnymalBearLoopshapingInterface.cpp
        )
add_dependencies(${PROJECT_NAME}
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
        )
target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        )

add_executable(${PROJECT_NAME}_mpc_node
        src/AnymalBearLoopshapingMpcNode.cpp
        )
add_dependencies(${PROJECT_NAME}_mpc_node
        ${PROJECT_NAME}
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
        )
target_link_libraries(${PROJECT_NAME}_mpc_node
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        )

add_executable(${PROJECT_NAME}_dummy_mrt_node
        src/AnymalBearLoopshapingDummyMrt.cpp
        src/AnymalBearLoopshapingVisualization.cpp
        )
add_dependencies(${PROJECT_NAME}_dummy_mrt_node
        ${PROJECT_NAME}
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
        )
target_link_libraries(${PROJECT_NAME}_dummy_mrt_node
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        )

