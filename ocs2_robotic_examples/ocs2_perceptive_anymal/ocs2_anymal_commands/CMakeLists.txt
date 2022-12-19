cmake_minimum_required(VERSION 3.0)
project(ocs2_anymal_commands)

find_package(catkin REQUIRED COMPONENTS
  roslib
  ocs2_ros_interfaces
  ocs2_robotic_tools
  ocs2_switched_model_interface
  grid_map_filters_rsl
)

## Eigen3
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

find_package(Boost REQUIRED COMPONENTS
  filesystem
)

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
    ocs2_ros_interfaces
    ocs2_robotic_tools
    ocs2_switched_model_interface
  DEPENDS
    Boost
)

###########
## Build ##
###########

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# Declare a C++ library
add_library(${PROJECT_NAME}
  src/LoadMotions.cpp
  src/ModeSequenceKeyboard.cpp
  src/MotionCommandController.cpp
  src/MotionCommandDummy.cpp
  src/MotionCommandInterface.cpp
  src/PoseCommandToCostDesiredRos.cpp
  src/ReferenceExtrapolation.cpp
  src/TerrainAdaptation.cpp
  )
add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  )
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

# Declare a C++ Executable
add_executable(target_command_node
  src/AnymalPoseCommandNode.cpp
  )
add_dependencies(target_command_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )
target_link_libraries(target_command_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  )
target_compile_options(target_command_node PRIVATE ${OCS2_CXX_FLAGS})

add_executable(gait_command_node
  src/AnymalGaitNode.cpp
)
add_dependencies(gait_command_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(gait_command_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
target_compile_options(gait_command_node PRIVATE ${OCS2_CXX_FLAGS})

add_executable(motion_command_node
  src/AnymalMotionCommandNode.cpp
  )
add_dependencies(motion_command_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )
target_link_libraries(motion_command_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  )
target_compile_options(motion_command_node PRIVATE ${OCS2_CXX_FLAGS})

#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
install(TARGETS target_command_node gait_command_node motion_command_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

catkin_add_gtest(test_${PROJECT_NAME}
  test/testLoadMotions.cpp
  test/testReferenceExtrapolation.cpp
  test/testTerrainAdaptation.cpp
)
target_link_libraries(test_${PROJECT_NAME}
  ${PROJECT_NAME}
  -lstdc++fs
  gtest_main
)
