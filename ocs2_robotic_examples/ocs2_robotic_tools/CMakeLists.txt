cmake_minimum_required(VERSION 3.0)
project(ocs2_robotic_tools)

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

#############
## Install ##
#############

install(DIRECTORY
            include/${PROJECT_NAME}/
            include/rbd_libraries/robcogen/iit/
        DESTINATION
            ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
