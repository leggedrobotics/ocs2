cmake_minimum_required(VERSION 2.8.3)
project(c_switched_model_interface)

add_definitions("-std=c++11")

find_package(catkin REQUIRED
	cmake_modules
)

## Eign3
set( EIGEN_INCLUDE_DIRS "$ENV{LOCAL_EIGEN3_INCLUDE_DIR}" )
if( NOT EIGEN_INCLUDE_DIRS )
    find_package(Eigen REQUIRED)
    message( WARNING "Uses Globally installed Eigen since LOCAL_EIGEN3_INCLUDE_DIR is not defined!")
endif()

###################################
## catkin specific configuration ##
###################################
catkin_package(
	INCLUDE_DIRS include
	DEPENDS eigen
)

###########
## Build ##
###########

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIR}
)