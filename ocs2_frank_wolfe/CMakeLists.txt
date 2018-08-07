cmake_minimum_required (VERSION 2.8.3)
project (ocs2_frank_wolfe)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
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
   DEPENDS 
     eigen3
)

###########
## Build ##
###########

## Include directories
include_directories(
    include
    ${EIGEN3_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

#############
## Doxygen ##
#############

find_package(Doxygen)
if(DOXYGEN_FOUND)

    set(doxyfile_in ${CMAKE_CURRENT_SOURCE_DIR}/doc/ocs2_frank_wolfe_doxyfile.in)
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
## $ catkin_test_results ../../../build/ocs2_frank_wolfe

catkin_add_gtest(quadratic_test 
  test/QuadraticTest.cpp
)
target_link_libraries(quadratic_test
  ${catkin_LIBRARIES}
  glpk
)

catkin_add_gtest(matyas_test 
  test/MatyasTest.cpp
)
target_link_libraries(matyas_test
  ${catkin_LIBRARIES}
  glpk
)

#catkin_add_gtest(glpk_test 
#  test/testGLPK.cpp
#)
#target_link_libraries(glpk_test
#  ${catkin_LIBRARIES}
#  glpk
#)
