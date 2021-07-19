cmake_minimum_required(VERSION 3.0.2)
project(ocs2_ros_interfaces)

set(CATKIN_PACKAGE_DEPENDENCIES
  roscpp
  ocs2_msgs
  ocs2_core
  ocs2_ddp
  ocs2_mpc
  std_msgs
  visualization_msgs
  geometry_msgs
  interactive_markers
)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  ${CATKIN_PACKAGE_DEPENDENCIES}
)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

find_package(Boost REQUIRED COMPONENTS filesystem)

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS
    include
    ${EIGEN3_INCLUDE_DIRS}
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
  DEPENDS
    Boost
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

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
  src/visualization/VisualizationHelpers.cpp
  src/visualization/VisualizationColors.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

add_executable(test_custom_callback_queue
  test/test_custom_callback_queue.cpp
)
add_dependencies(test_custom_callback_queue
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(test_custom_callback_queue
  ${catkin_LIBRARIES}
)
target_compile_options(test_custom_callback_queue PRIVATE ${OCS2_CXX_FLAGS})

# multiplot remap node
add_executable(multiplot_remap
  src/multiplot/MultiplotRemap.cpp
)
add_dependencies(multiplot_remap
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(multiplot_remap
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
target_compile_options(multiplot_remap PRIVATE ${OCS2_CXX_FLAGS})

add_executable(${PROJECT_NAME}_lintTarget
  src/lintTarget.cpp
)
add_dependencies(${PROJECT_NAME}_lintTarget
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}_lintTarget
  ${catkin_LIBRARIES}
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

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(TARGETS multiplot_remap
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch multiplot
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############
## Info ==============================
## to run tests, cd package folder and run
## $ catkin build -DCMAKE_BUILD_TYPE=RelWithDebInfo --this
## $ catkin run_tests --no-deps --this
## to see the summary of unit test results run
## $ catkin_test_results ../../../build/ocs2_ros_interfaces
