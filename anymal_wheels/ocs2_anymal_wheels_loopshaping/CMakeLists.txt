cmake_minimum_required(VERSION 3.0)
project(ocs2_anymal_wheels_loopshaping)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roslib
    ocs2_anymal_wheels
    ocs2_quadruped_loopshaping_interface
    )

## Eigen3
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

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
    ocs2_anymal_wheels
    ocs2_quadruped_loopshaping_interface
  DEPENDS
)

###########
## Build ##
###########

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  src/AnymalWheelsLoopshapingInterface.cpp)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_mpc_node
  src/AnymalWheelsLoopshapingMpcNode.cpp)
target_link_libraries(${PROJECT_NAME}_mpc_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_dummy_mrt_node
  src/AnymalWheelsLoopshapingDummyMrt.cpp)
target_link_libraries(${PROJECT_NAME}_dummy_mrt_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES})

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
  message(STATUS "Run clang tooling")
  add_clang_tooling(
    TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_mpc_node ${PROJECT_NAME}_dummy_mrt_node
    SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
    CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
    CF_WERROR
  )
endif (cmake_clang_tools_FOUND)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS
    ${PROJECT_NAME}_mpc_node
    ${PROJECT_NAME}_dummy_mrt_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY config launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
