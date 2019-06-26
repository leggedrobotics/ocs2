cmake_minimum_required(VERSION 2.8.3)
project(ocs2_ilqr)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS 
  cmake_modules 
  ocs2_core
  ocs2_ddp_base
)
find_package(Boost REQUIRED COMPONENTS 
  system
  filesystem 
)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11 -Wfatal-errors -Wl,--no-as-needed)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS 
    include
    ${EIGEN3_INCLUDE_DIRS}
  CATKIN_DEPENDS 
    ocs2_core
    ocs2_ddp_base
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

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
    message(STATUS "Run clang tooling for target " ${PROJECT_NAME}_lintTarget)
    add_clang_tooling(
            TARGET ${PROJECT_NAME}_lintTarget
            SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
            CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
            CF_QUIET # temporarily set to quiet, because the current amount of warning crashes the buildserver
            CT_QUIET
            # CF_FIX # temporarily disabled
    )
endif(cmake_clang_tools_FOUND)

#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
        
#############
## Doxygen ##
#############

#find_package(Doxygen)
#if(DOXYGEN_FOUND)
#
#    set(doxyfile_in ${CMAKE_CURRENT_SOURCE_DIR}/doc/ocs2_ilqr_doxyfile.in)
#    set(doxyfile ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile)
#
#    configure_file(${doxyfile_in} ${doxyfile} @ONLY)
#
#    add_custom_target(doc
#            COMMAND ${CMAKE_COMMAND} -E echo_append "Building OCS2 Documentation..."
#            COMMAND ${DOXYGEN_EXECUTABLE} ${doxyfile}
#            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/doc
#            COMMAND ${CMAKE_COMMAND} -E echo_append "OCS2 Documentation built in ${CMAKE_CURRENT_SOURCE_DIR}/doc"
#            VERBATIM)
#endif()

#############
## Testing ##
#############
## Info ==============================
## to run tests, cd package folder and run
## $ catkin build -DCMAKE_BUILD_TYPE=RelWithDebInfo --this
## $ catkin run_tests --no-deps --this
## to see the summary of unit test results run
## $ catkin_test_results ../../../build/ocs2_ilqr

catkin_add_gtest(exp0_ilqr_test 
  test/exp0_ilqr_test.cpp
)
target_link_libraries(exp0_ilqr_test
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

catkin_add_gtest(exp1_ilqr_test 
  test/exp1_ilqr_test.cpp
)
target_link_libraries(exp1_ilqr_test
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)
