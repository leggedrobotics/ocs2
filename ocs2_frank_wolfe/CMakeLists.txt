cmake_minimum_required (VERSION 3.0)
project (ocs2_frank_wolfe)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  ocs2_core
)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fopenmp -std=c++11 -Wfatal-errors -Wl,--no-as-needed")


###################################
## catkin specific configuration ##
###################################
catkin_package(
   INCLUDE_DIRS 
     include
     ${EIGEN3_INCLUDE_DIRS}
   CATKIN_DEPENDS 
     ocs2_core 
   DEPENDS
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
## $ catkin_test_results ../../../build/ocs2_frank_wolfe

#catkin_add_gtest(quadratic_test 
#  test/QuadraticTest.cpp
#)
#target_link_libraries(quadratic_test
#  ${catkin_LIBRARIES}
#  glpk
#)

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
