cmake_minimum_required(VERSION 2.8.3)
project(c_switched_model_interface)

add_definitions("-std=c++11")

find_package(catkin REQUIRED
	cmake_modules
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
	INCLUDE_DIRS include
	DEPENDS eigen
)

find_package(Eigen3 REQUIRED)

###########
## Build ##
###########

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIR}
)