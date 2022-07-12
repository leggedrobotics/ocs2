cmake_minimum_required(VERSION 3.0)
project(ocs2_anymal_motion_tracking)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roslib
  ocs2_anymal_mpc
  ocs2_quadruped_loopshaping_interface
  ocs2_robotic_assets
  ocs2_pinocchio_interface
  ocs2_anymal_loopshaping_mpc
)

## Eigen3
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

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
  roslib
    ocs2_anymal_mpc
    ocs2_quadruped_loopshaping_interface
    ocs2_robotic_assets
    ocs2_pinocchio_interface
    ocs2_anymal_loopshaping_mpc
  DEPENDS
)

###########
## Build ##
###########

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/CartesianMotionTrackingCost.cpp
  src/MotionTracking.cpp
  src/ModelParentSkeleton.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

#add_executable(${PROJECT_NAME}_viz_node
#  src/VisualizeTargetsNode.cpp
#)
#target_link_libraries(${PROJECT_NAME}_viz_node
#  ${PROJECT_NAME}
#  ${catkin_LIBRARIES}
#)

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
  message(STATUS "Run clang tooling")
  add_clang_tooling(
    TARGETS ${PROJECT_NAME}
    SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
    CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
    CF_WERROR
  )
endif (cmake_clang_tools_FOUND)

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
install(TARGETS ${PROJECT_NAME}_viz_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

# Build unit tests
catkin_add_gtest(${PROJECT_NAME}_test
  test/testModelParentSkeleton.cpp
)
target_link_libraries(${PROJECT_NAME}_test
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  gtest_main
)