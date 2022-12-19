cmake_minimum_required(VERSION 3.10)
project(segmented_planes_terrain_model)

# Catkin dependencies
set(CATKIN_PACKAGE_DEPENDENCIES
	convex_plane_decomposition
	convex_plane_decomposition_msgs
	convex_plane_decomposition_ros
	grid_map_filters_rsl
	grid_map_ros
	grid_map_sdf
	ocs2_ros_interfaces
	ocs2_switched_model_interface
	roscpp
	sensor_msgs
	visualization_msgs
)

find_package(catkin REQUIRED COMPONENTS
	${CATKIN_PACKAGE_DEPENDENCIES}
)

# Cpp standard version
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

###################################
## catkin specific configuration ##
###################################
catkin_package(
	INCLUDE_DIRS include
	LIBRARIES ${PROJECT_NAME}
	CATKIN_DEPENDS ${CATKIN_PACKAGE_DEPENDENCIES}
	DEPENDS
)

###########
## Build ##
###########

include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
	src/SegmentedPlanesTerrainModel.cpp
	src/SegmentedPlanesTerrainModelRos.cpp
	src/SegmentedPlanesTerrainVisualization.cpp
	)
add_dependencies(${PROJECT_NAME}
	${catkin_EXPORTED_TARGETS}
	)
target_link_libraries(${PROJECT_NAME}
	${catkin_LIBRARIES}
	)
target_compile_options(${PROJECT_NAME}
	PUBLIC -DCGAL_HAS_THREADS
	)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	)
install(DIRECTORY include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	)

#############
## Testing ##
#############
