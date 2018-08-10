cmake_minimum_required(VERSION 2.8.3)
project(ocs2_ocs2)

find_package(catkin REQUIRED COMPONENTS 
  cmake_modules 
  ocs2_core 
  ocs2_slq 
  ocs2_frank_wolfe
)
find_package(Boost REQUIRED COMPONENTS 
  system
  filesystem 
)

## Eign3
set( EIGEN3_INCLUDE_DIRS "$ENV{LOCAL_EIGEN3_INCLUDE_DIR}" )
if( NOT EIGEN3_INCLUDE_DIRS )
    find_package(Eigen3 REQUIRED)
    message( WARNING "Uses Globally installed Eigen since LOCAL_EIGEN3_INCLUDE_DIR is not defined!")
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fopenmp -std=c++11 -Wfatal-errors -Wl,--no-as-needed")

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS 
    include
  CATKIN_DEPENDS 
    ocs2_core 
    ocs2_slq 
    ocs2_frank_wolfe
  DEPENDS 
    boost 
    eigen3 
    glpk
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

#############
## Doxygen ##
#############

find_package(Doxygen)
if(DOXYGEN_FOUND)

    set(doxyfile_in ${CMAKE_CURRENT_SOURCE_DIR}/doc/ocs2_ocs2_doxyfile.in)
    set(doxyfile ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile)

    configure_file(${doxyfile_in} ${doxyfile} @ONLY)

    add_custom_target(doc
            COMMAND ${CMAKE_COMMAND} -E echo_append "Building OCS2 Documentation..."
            COMMAND ${DOXYGEN_EXECUTABLE} ${doxyfile}
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/doc
            COMMAND ${CMAKE_COMMAND} -E echo_append "OCS2 Documentation built in ${CMAKE_CURRENT_SOURCE_DIR}/doc"
            VERBATIM)
endif()

#############
## Testing ##
#############
## Info ==============================
## to run tests, cd package folder and run
## $ catkin build -DCMAKE_BUILD_TYPE=RelWithDebInfo --this
## $ catkin run_tests --no-deps --this
## to see the summary of unit test results run
## $ catkin_test_results ../../../build/ocs2_ocs2

catkin_add_gtest(event_time_indexer_unittest 
  test/testEventTimeIndexer.cpp)
target_link_libraries(event_time_indexer_unittest
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

catkin_add_gtest(exp0_gslq_test 
  test/exp0_gslq_test.cpp
)
target_link_libraries(exp0_gslq_test
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

catkin_add_gtest(exp1_gslq_test 
  test/exp1_gslq_test.cpp
)
target_link_libraries(exp1_gslq_test
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

catkin_add_gtest(exp0_ocs2_test 
  test/exp0_ocs2_test.cpp
)
target_link_libraries(exp0_ocs2_test
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  glpk
)

catkin_add_gtest(exp1_ocs2_test 
  test/exp1_ocs2_test.cpp
)
target_link_libraries(exp1_ocs2_test
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  glpk
)

