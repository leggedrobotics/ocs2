cmake_minimum_required(VERSION 2.8.3)
project(ocs2_slq)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS 
  cmake_modules 
  ocs2_core
  ocs2_oc
)
find_package(Boost REQUIRED COMPONENTS 
  system
  filesystem 
)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fopenmp -std=c++11 -Wfatal-errors -Wl,--no-as-needed")

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS 
    include
    test/include
    ${EIGEN3_INCLUDE_DIRS}
  CATKIN_DEPENDS 
    ocs2_core
    ocs2_oc
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

#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY test/include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})


#############
## Testing ##
#############
## Info ==============================
## to run tests, cd package folder and run
## $ catkin build -DCMAKE_BUILD_TYPE=RelWithDebInfo --this
## $ catkin run_tests --no-deps --this
## to see the summary of unit test results run
## $ catkin_test_results ../../../build/ocs2_slq

catkin_add_gtest(exp0_slq_test 
  test/exp0_slq_test.cpp
)
target_link_libraries(exp0_slq_test
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

catkin_add_gtest(exp1_slq_test 
  test/exp1_slq_test.cpp
)
target_link_libraries(exp1_slq_test
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

catkin_add_gtest(exp2_slq_test 
  test/exp2_slq_test.cpp
)
target_link_libraries(exp2_slq_test
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

