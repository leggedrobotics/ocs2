cmake_minimum_required(VERSION 3.0)
project(ocs2_mpc)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  ocs2_core
  ocs2_slq
  ocs2_ilqr
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
    ocs2_ilqr
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

catkin_add_gtest(testMPC_OCS2
        test/testMPC_OCS2.cpp
        )
target_link_libraries(testMPC_OCS2
        ${catkin_LIBRARIES}
        )


