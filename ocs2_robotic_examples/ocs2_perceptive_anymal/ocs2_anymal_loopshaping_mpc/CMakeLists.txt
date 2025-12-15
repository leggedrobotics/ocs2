cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_STANDARD 17)
project(ocs2_anymal_loopshaping_mpc)

## Find ament_cmake macros and libraries
find_package(ament_cmake REQUIRED)
find_package(convex_plane_decomposition REQUIRED)
find_package(convex_plane_decomposition_ros REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ocs2_anymal_mpc REQUIRED)
find_package(ocs2_quadruped_loopshaping_interface REQUIRED)
find_package(PCL REQUIRED)
find_package(segmented_planes_terrain_model REQUIRED)

## Eigen3
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

find_package(Boost REQUIRED COMPONENTS
  system
  filesystem
  log_setup
  log
)




set(dependencies
  convex_plane_decomposition
  convex_plane_decomposition_ros
  grid_map_ros
  rclcpp
  ocs2_anymal_mpc
  ocs2_quadruped_loopshaping_interface
  segmented_planes_terrain_model
)

###########
## Build ##
###########

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/AnymalLoopshapingInterface.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
)
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

add_executable(${PROJECT_NAME}_mpc_node
  src/AnymalLoopshapingMpcNode.cpp
)
ament_target_dependencies(${PROJECT_NAME}_mpc_node
  ${dependencies}
)
target_link_libraries(${PROJECT_NAME}_mpc_node
  ${PROJECT_NAME}
)

add_executable(${PROJECT_NAME}_dummy_mrt_node
  src/AnymalLoopshapingDummyMrt.cpp
)
ament_target_dependencies(${PROJECT_NAME}_dummy_mrt_node
  ${dependencies}
)
target_link_libraries(${PROJECT_NAME}_dummy_mrt_node
  ${PROJECT_NAME}
)

add_executable(${PROJECT_NAME}_perceptive_demo
  src/PerceptiveMpcDemo.cpp
  )
ament_target_dependencies(${PROJECT_NAME}_perceptive_demo
  ${dependencies}
)
target_link_libraries(${PROJECT_NAME}_perceptive_demo
  ${PROJECT_NAME}
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
install(TARGETS 
  ${PROJECT_NAME}_mpc_node 
  ${PROJECT_NAME}_dummy_mrt_node 
  ${PROJECT_NAME}_perceptive_demo
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}/
)

#############
## Testing ##
#############
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)

  # Build unit tests
  ament_add_gtest(${PROJECT_NAME}_test
    test/testProblemFormulation.cpp
    test/testMotionTracking.cpp
    test/testSensitivity.cpp
  )
  ament_target_dependencies(${PROJECT_NAME}_test
    ${dependencies}
  )
  target_link_libraries(${PROJECT_NAME}_test
    ${PROJECT_NAME}
  )
  set_tests_properties(${PROJECT_NAME}_test PROPERTIES TIMEOUT 3600)
endif()

ament_export_dependencies(${dependencies})  
ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()
