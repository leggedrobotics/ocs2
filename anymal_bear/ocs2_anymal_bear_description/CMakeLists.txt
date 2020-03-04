cmake_minimum_required(VERSION 3.0)
project(ocs2_anymal_bear_description)

find_package(catkin REQUIRED COMPONENTS
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
    INCLUDE_DIRS
    CATKIN_DEPENDS
)

###########
## Build ##
###########

#############
## Install ##
#############

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

#############
## Testing ##
#############
