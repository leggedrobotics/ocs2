cmake_minimum_required(VERSION 3.14.4)
project(ocs2_ros_interfaces LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wno-extra -Wpedantic -Wno-reorder)
endif()

## Find ament_cmake macros and libraries
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ocs2_msgs REQUIRED)
find_package(ocs2_core REQUIRED)
find_package(ocs2_mpc REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)

find_package(ocs2_pinocchio_interface REQUIRED)
pkg_check_modules(pinocchio REQUIRED pinocchio)
find_package(pinocchio REQUIRED)

find_package(urdf REQUIRED)
find_package(urdfdom REQUIRED)

pkg_check_modules(hpp-fcl REQUIRED hpp-fcl)
find_package(hpp-fcl REQUIRED)

###########
## Build ##
###########

set(dependencies
  rclcpp
  ocs2_msgs
  ocs2_core
  ocs2_mpc
  std_msgs
  visualization_msgs
  geometry_msgs
  interactive_markers
  Boost
  Eigen3
  ocs2_pinocchio_interface
  pinocchio
  hpp-fcl
  urdf
  urdfdom
  Eigen3
)

link_directories(
  ${pinocchio_LIBRARY_DIRS}
)

add_library(${PROJECT_NAME} SHARED
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
set(includes_${PROJECT_NAME}
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_include_directories(${PROJECT_NAME} PUBLIC
  ${includes_${PROJECT_NAME}}
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
  ocs2_pinocchio_interface
  pinocchio
  hpp-fcl
  urdf
  urdfdom
  Eigen3
)

target_link_libraries(${PROJECT_NAME}
  ${Boost_LIBRARIES}
  ${pinocchio_LIBRARIES}
)

add_executable(test_custom_callback_queue
  test/test_custom_callback_queue.cpp
)
ament_target_dependencies(test_custom_callback_queue
  ${dependencies}
)
target_link_libraries(test_custom_callback_queue
  ${PROJECT_NAME}
)
target_compile_options(test_custom_callback_queue PRIVATE ${OCS2_CXX_FLAGS})

# multiplot remap node
add_executable(multiplot_remap
  src/multiplot/MultiplotRemap.cpp
)
ament_target_dependencies(multiplot_remap
  ${dependencies} 
)
target_link_libraries(test_custom_callback_queue
  ${PROJECT_NAME}
)
target_compile_options(multiplot_remap PRIVATE ${OCS2_CXX_FLAGS})

add_executable(${PROJECT_NAME}_lintTarget
  src/lintTarget.cpp
)
target_include_directories(${PROJECT_NAME}_lintTarget PUBLIC
  ${includes_${PROJECT_NAME}}
)
ament_target_dependencies(${PROJECT_NAME}_lintTarget
  ${dependencies}
)

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  message(STATUS "Run clang tooling for target " ${PROJECT_NAME}_lintTarget)
  add_clang_tooling(
    TARGETS ${PROJECT_NAME}_lintTarget
    SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
    CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
    CF_WERROR
  )
endif(cmake_clang_tools_FOUND)

#############
## Install ##
#############

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(TARGETS multiplot_remap DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY multiplot
  DESTINATION include/${PROJECT_NAME}/
)
ament_export_dependencies(
  ${dependencies}
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

#############
## Testing ##
#############
## Info ==============================
## to run tests, cd package folder and run
## $ catkin build -DCMAKE_BUILD_TYPE=RelWithDebInfo --this
## $ catkin run_tests --no-deps --this
## to see the summary of unit test results run
## $ catkin_test_results ../../../build/ocs2_ros_interfaces

ament_package()