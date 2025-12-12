cmake_minimum_required(VERSION 3.5)
project(ocs2_ros_interfaces)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ocs2_msgs REQUIRED)
find_package(ocs2_core REQUIRED)
find_package(ocs2_mpc REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Boost REQUIRED COMPONENTS system filesystem log_setup log)

add_library(${PROJECT_NAME}
  src/command/TargetTrajectoriesRosPublisher.cpp
  src/command/TargetTrajectoriesInteractiveMarker.cpp
  src/command/TargetTrajectoriesKeyboardPublisher.cpp
  src/common/RosMsgConversions.cpp
  src/common/RosMsgHelpers.cpp
  src/mpc/MPC_ROS_Interface.cpp
  src/mrt/LoopshapingDummyObserver.cpp
  src/mrt/MRT_ROS_Dummy_Loop.cpp
  src/mrt/MRT_ROS_Interface.cpp
  src/synchronized_module/RosReferenceManager.cpp
  src/synchronized_module/SolverObserverRosCallbacks.cpp
  src/visualization/VisualizationHelpers.cpp
  src/visualization/VisualizationColors.cpp
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  ocs2_msgs
  ocs2_core
  ocs2_mpc
  std_msgs
  visualization_msgs
  geometry_msgs
  interactive_markers
)
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

# multiplot remap node
add_executable(multiplot_remap
  src/multiplot/MultiplotRemap.cpp
)
target_link_libraries(multiplot_remap
  ${PROJECT_NAME}
)
target_compile_options(multiplot_remap PRIVATE ${OCS2_CXX_FLAGS})

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS multiplot_remap
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)
install(DIRECTORY launch multiplot
  DESTINATION share/${PROJECT_NAME}
)

install(
  EXPORT export_${PROJECT_NAME}
  DESTINATION share/${PROJECT_NAME}/cmake
)

ament_export_include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  ocs2_msgs
  ocs2_core
  ocs2_mpc
  std_msgs
  visualization_msgs
  geometry_msgs
  interactive_markers
  Eigen3
  Boost
)

############
# Testing ##
############

if(BUILD_TESTING)
  add_executable(test_custom_callback_queue
    test/test_custom_callback_queue.cpp
  )
  target_link_libraries(test_custom_callback_queue
    ${PROJECT_NAME}
  )
  target_compile_options(test_custom_callback_queue PRIVATE ${OCS2_CXX_FLAGS})
endif()

ament_package()
