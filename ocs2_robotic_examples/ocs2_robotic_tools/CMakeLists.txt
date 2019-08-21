cmake_minimum_required(VERSION 3.0)
project(ocs2_robotic_tools)

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED COMPONENTS
        ocs2_slq
        ocs2_core
        ocs2_mpc
        ocs2_comm_interfaces
)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

###################################
## catkin specific configuration ##
###################################

include_directories(
  include
  include/${PROJECT_NAME}/rbd_libraries/robcogen
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS 
    include
    include/${PROJECT_NAME}/rbd_libraries/robcogen
    ${EIGEN3_INCLUDE_DIRS}
  CATKIN_DEPENDS
    ocs2_slq
    ocs2_core
    ocs2_mpc
    ocs2_comm_interfaces
  DEPENDS
)

add_executable(${PROJECT_NAME}_lintTarget
  src/lintTarget.cpp
)

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  message(STATUS "Run clang tooling for target " ${PROJECT_NAME}_lintTarget)
  add_clang_tooling(
    TARGETS ${PROJECT_NAME}_lintTarget
    SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include/command ${CMAKE_CURRENT_SOURCE_DIR}/include/common
    CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
    CF_WERROR
  )
endif(cmake_clang_tools_FOUND)

#############
## Install ##
#############

install(DIRECTORY
            include/${PROJECT_NAME}/
        DESTINATION
            ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY
        include/${PROJECT_NAME}/rbd_libraries/robcogen/
        DESTINATION
        "${CATKIN_PACKAGE_INCLUDE_DESTINATION}/../")


