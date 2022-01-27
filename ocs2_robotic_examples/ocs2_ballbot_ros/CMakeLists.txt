cmake_minimum_required(VERSION 3.0.2)
project(ocs2_ballbot_ros)

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CATKIN_PACKAGE_DEPENDENCIES
  tf
  urdf
  kdl_parser
  robot_state_publisher
  visualization_msgs
  ocs2_core
  ocs2_ddp
  ocs2_mpc
  ocs2_sqp
  ocs2_ros_interfaces
  ocs2_robotic_tools
  ocs2_ballbot
  ocs2_robotic_assets
)

find_package(catkin REQUIRED COMPONENTS
  roslib
  ${CATKIN_PACKAGE_DEPENDENCIES}
)

find_package(Boost REQUIRED COMPONENTS
  system
  filesystem
)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS
    include
    ${EIGEN3_INCLUDE_DIRS}
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
  DEPENDS
    Boost
)

###########
## Build ##
###########

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# Mpc node
add_executable(ballbot_mpc
  src/BallbotMpcNode.cpp
)
add_dependencies(ballbot_mpc
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(ballbot_mpc
  ${catkin_LIBRARIES}
)
target_compile_options(ballbot_mpc PRIVATE ${OCS2_CXX_FLAGS})

# Dummy node
add_executable(ballbot_dummy_test
  src/DummyBallbotNode.cpp
  src/BallbotDummyVisualization.cpp
)
add_dependencies(ballbot_dummy_test
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(ballbot_dummy_test
  ${catkin_LIBRARIES}
)
target_compile_options(ballbot_dummy_test PRIVATE ${OCS2_CXX_FLAGS})

# Target node
add_executable(ballbot_target
  src/BallbotTargetPoseCommand.cpp
)
add_dependencies(ballbot_target
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(ballbot_target
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)
target_compile_options(ballbot_target PRIVATE ${OCS2_CXX_FLAGS})

## Combined MPC MRT example
add_executable(ballbot_mpc_mrt
  src/BallbotMpcMrtNode.cpp
  src/BallbotDummyVisualization.cpp
  )
add_dependencies(ballbot_mpc_mrt
  ${catkin_EXPORTED_TARGETS}
  )
target_link_libraries(ballbot_mpc_mrt
  ${catkin_LIBRARIES}
  )
target_compile_options(ballbot_mpc_mrt  PRIVATE ${OCS2_CXX_FLAGS})

## SQP node for ballbot
add_executable(ballbot_sqp
  src/MultipleShootingBallbotNode.cpp
)
add_dependencies(ballbot_sqp
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(ballbot_sqp
  ${catkin_LIBRARIES}
)
target_compile_options(ballbot_sqp  PRIVATE ${OCS2_CXX_FLAGS})


#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  message(STATUS "Run clang tooling for target ocs2_ballbot")
  add_clang_tooling(
    TARGETS
        ballbot_mpc
        ballbot_dummy_test
        ballbot_target
    SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
    CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
    CF_WERROR
  )
endif(cmake_clang_tools_FOUND)

#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(TARGETS ballbot_mpc ballbot_dummy_test ballbot_target ballbot_sqp
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


#############
## Testing ##
#############

catkin_add_gtest(ballbot_integration_test
  test/ballbotIntegrationTest.cpp
)
target_link_libraries(ballbot_integration_test
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)
