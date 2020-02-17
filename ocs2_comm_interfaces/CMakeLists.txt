cmake_minimum_required(VERSION 3.0)
project(ocs2_comm_interfaces)

set(CATKIN_PACKAGE_DEPENDENCIES
  roscpp
  ocs2_msgs
  ocs2_core
  ocs2_ddp
  ocs2_ocs2
  ocs2_mpc
)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  ${CATKIN_PACKAGE_DEPENDENCIES}
)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

find_package(Boost REQUIRED COMPONENTS filesystem)

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fopenmp -std=c++11 -Wfatal-errors -Wl,--no-as-needed")

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

## Specify additional locations of header files
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(test_custom_callback_queue
  test/test_custom_callback_queue.cpp
)
add_dependencies(test_custom_callback_queue
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(test_custom_callback_queue
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_lintTarget
  src/lintTarget.cpp
)
add_dependencies(${PROJECT_NAME}_lintTarget
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}_lintTarget
  ${catkin_LIBRARIES}
)

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  message(STATUS "Run clang tooling for target " ${PROJECT_NAME}_lintTarget)
  add_clang_tooling(
  TARGETS ${PROJECT_NAME}_lintTarget
    SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
    CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
    CF_WERROR
)
endif(cmake_clang_tools_FOUND)

#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

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

catkin_add_gtest(ocs2_interface_mpc_test
  test/test_ocs2_interfaces_mpc.cpp
  )
add_dependencies(ocs2_interface_mpc_test
  ${catkin_EXPORTED_TARGETS}
  )
target_link_libraries(ocs2_interface_mpc_test
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  )