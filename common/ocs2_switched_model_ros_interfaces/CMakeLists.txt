cmake_minimum_required(VERSION 3.0)
project(ocs2_switched_model_ros_interfaces)

find_package(catkin REQUIRED COMPONENTS
  ocs2_msgs
  ocs2_comm_interfaces
  switched_model_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ocs2_msgs
    ocs2_comm_interfaces
    switched_model_msgs
  DEPENDS
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

