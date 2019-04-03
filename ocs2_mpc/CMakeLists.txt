cmake_minimum_required(VERSION 3.0)
project(ocs2_mpc)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  ocs2_core
  ocs2_slq
  ocs2_ocs2
)
find_package(Boost REQUIRED COMPONENTS
  system 
  filesystem
)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS 
    include
    ${EIGEN3_INCLUDE_DIRS}
  CATKIN_DEPENDS 
    ocs2_core 
    ocs2_slq 
    ocs2_ocs2
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

#############
## Testing ##
#############
## Info ==============================
## to run tests, cd package folder and run
## $ catkin build -DCMAKE_BUILD_TYPE=RelWithDebInfo --this
## $ catkin run_tests --no-deps --this
## to see the summary of unit test results run
## $ catkin_test_results ../../../build/ocs2_mpc


