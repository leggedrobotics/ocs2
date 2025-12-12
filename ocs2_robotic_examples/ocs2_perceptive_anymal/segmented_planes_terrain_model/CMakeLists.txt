cmake_minimum_required(VERSION 3.10)
project(segmented_planes_terrain_model)

# ament_cmake dependencies
set(dependencies
	convex_plane_decomposition
	convex_plane_decomposition_msgs
	convex_plane_decomposition_ros
	grid_map_filters_rsl
	grid_map_ros
	grid_map_sdf
	ocs2_ros_interfaces
	ocs2_switched_model_interface
	rclcpp
	sensor_msgs
	visualization_msgs
)

find_package(ament_cmake REQUIRED)
find_package(convex_plane_decomposition REQUIRED)
find_package(convex_plane_decomposition_msgs REQUIRED)
find_package(convex_plane_decomposition_ros REQUIRED)
find_package(grid_map_filters_rsl REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_sdf REQUIRED)
find_package(ocs2_ros_interfaces REQUIRED)
find_package(ocs2_switched_model_interface REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rclcpp REQUIRED) 
find_package(PCL REQUIRED)

find_package(Boost REQUIRED COMPONENTS
  system
  filesystem
  log_setup
  log
)

# Cpp standard version
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

###########
## Build ##
###########

include_directories(
	include
	${PCL_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
	src/SegmentedPlanesTerrainModel.cpp
	src/SegmentedPlanesTerrainModelRos.cpp
	src/SegmentedPlanesTerrainVisualization.cpp
	)
ament_target_dependencies(${PROJECT_NAME}
	${dependencies}
	)
target_compile_options(${PROJECT_NAME}
	PUBLIC -DCGAL_HAS_THREADS
	)

#############
## Install ##
#############

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

ament_export_dependencies(${dependencies})  
ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()