cmake_minimum_required(VERSION 3.0.2)
project(ocs2_ocs2)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  ocs2_core
  ocs2_ddp
  ocs2_frank_wolfe
)
find_package(Boost REQUIRED COMPONENTS
  system
  filesystem
)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS
    include
    ${EIGEN3_INCLUDE_DIRS}
  CATKIN_DEPENDS
    ocs2_core
    ocs2_ddp
    ocs2_frank_wolfe
  DEPENDS
    Boost
)

###########
## Build ##
###########

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_lintTarget
  src/lintTarget.cpp
)
target_link_libraries(${PROJECT_NAME}_lintTarget
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  message(STATUS "Run clang tooling for target ocs2_ocs2")
  add_clang_tooling(
    TARGETS ${PROJECT_NAME}_lintTarget
    SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include  ${CMAKE_CURRENT_SOURCE_DIR}/test
    CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
    CF_WERROR
)
endif(cmake_clang_tools_FOUND)

#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

#############
## Testing ##
#############
## Info ==============================
## to run tests, cd package folder and run
## $ catkin build -DCMAKE_BUILD_TYPE=RelWithDebInfo --this
## $ catkin run_tests --no-deps --this
## to see the summary of unit test results run
## $ catkin_test_results ../../../build/ocs2_ocs2

#catkin_add_gtest(exp0_gddp_test
#  test/exp0_gddp_test.cpp
#)
#target_link_libraries(exp0_gddp_test
#  ${catkin_LIBRARIES}
#  ${Boost_LIBRARIES}
#)
#target_compile_options(exp0_gddp_test PRIVATE ${OCS2_CXX_FLAGS})

#catkin_add_gtest(exp1_gddp_test
#  test/exp1_gddp_test.cpp
#)
#target_link_libraries(exp1_gddp_test
#  ${catkin_LIBRARIES}
#  ${Boost_LIBRARIES}
#)
#target_compile_options(exp1_gddp_test PRIVATE ${OCS2_CXX_FLAGS})

#catkin_add_gtest(exp0_ocs2_test
#  test/exp0_ocs2_test.cpp
#)
#target_link_libraries(exp0_ocs2_test
#  ${catkin_LIBRARIES}
#  ${Boost_LIBRARIES}
#)
#target_compile_options(exp0_ocs2_test PRIVATE ${OCS2_CXX_FLAGS})

#catkin_add_gtest(exp1_ocs2_test
#  test/exp1_ocs2_test.cpp
#)
#target_link_libraries(exp1_ocs2_test
#  ${catkin_LIBRARIES}
#  ${Boost_LIBRARIES}
#)
#target_compile_options(exp1_ocs2_test PRIVATE ${OCS2_CXX_FLAGS})

