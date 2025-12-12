cmake_minimum_required(VERSION 3.10)
project(convex_plane_decomposition_ros)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

find_package(convex_plane_decomposition REQUIRED)
find_package(convex_plane_decomposition_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

set(dependencies
  convex_plane_decomposition
  convex_plane_decomposition_msgs
  geometry_msgs
  grid_map_core
  grid_map_msgs
  grid_map_ros
  rclcpp
  std_msgs
  visualization_msgs
)

###########
## Build ##
###########

add_library(${PROJECT_NAME}
  src/MessageConversion.cpp
  src/ParameterLoading.cpp
  src/RosVisualizations.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

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
install(
  FILES
    include/convex_plane_decomposition_ros/MessageConversion.h
    include/convex_plane_decomposition_ros/ParameterLoading.h
    include/convex_plane_decomposition_ros/RosVisualizations.h
  DESTINATION include/${PROJECT_NAME}/convex_plane_decomposition_ros
)
install(DIRECTORY config data DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(${dependencies})
ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3)
ament_package()

# NOTE: The original ROS 1 package includes several nodes and tests.
# For this Jazzy workspace we only build the library used by OCS2 demos.
