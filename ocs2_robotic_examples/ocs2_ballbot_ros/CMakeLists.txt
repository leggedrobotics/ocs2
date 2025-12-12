cmake_minimum_required(VERSION 3.5)
project(ocs2_ballbot_ros)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(robot_state_publisher REQUIRED)

find_package(ocs2_core REQUIRED)
find_package(ocs2_oc REQUIRED)
find_package(ocs2_ddp REQUIRED)
find_package(ocs2_mpc REQUIRED)
find_package(ocs2_sqp REQUIRED)
find_package(ocs2_slp REQUIRED)
find_package(ocs2_ros_interfaces REQUIRED)
find_package(ocs2_robotic_tools REQUIRED)
find_package(ocs2_robotic_assets REQUIRED)
find_package(ocs2_ballbot REQUIRED)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Boost REQUIRED COMPONENTS system filesystem log_setup log)

add_library(${PROJECT_NAME}
  src/BallbotDummyVisualization.cpp
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(${PROJECT_NAME}
  ament_index_cpp
  rclcpp
  tf2_ros
  sensor_msgs
  geometry_msgs
  visualization_msgs
  urdf
  kdl_parser
  robot_state_publisher
  ocs2_core
  ocs2_oc
  ocs2_ddp
  ocs2_mpc
  ocs2_sqp
  ocs2_slp
  ocs2_ros_interfaces
  ocs2_robotic_tools
  ocs2_robotic_assets
  ocs2_ballbot
)
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

add_executable(ballbot_ddp src/BallbotDdpMpcNode.cpp)
target_link_libraries(ballbot_ddp ${PROJECT_NAME})
target_compile_options(ballbot_ddp PRIVATE ${OCS2_CXX_FLAGS})

add_executable(ballbot_sqp src/BallbotSqpMpcNode.cpp)
target_link_libraries(ballbot_sqp ${PROJECT_NAME})
target_compile_options(ballbot_sqp PRIVATE ${OCS2_CXX_FLAGS})

add_executable(ballbot_slp src/BallbotSlpMpcNode.cpp)
target_link_libraries(ballbot_slp ${PROJECT_NAME})
target_compile_options(ballbot_slp PRIVATE ${OCS2_CXX_FLAGS})

add_executable(ballbot_dummy_test src/DummyBallbotNode.cpp)
target_link_libraries(ballbot_dummy_test ${PROJECT_NAME})
target_compile_options(ballbot_dummy_test PRIVATE ${OCS2_CXX_FLAGS})

add_executable(ballbot_target src/BallbotTargetPoseCommand.cpp)
target_link_libraries(ballbot_target ${PROJECT_NAME})
target_compile_options(ballbot_target PRIVATE ${OCS2_CXX_FLAGS})

add_executable(ballbot_mpc_mrt src/BallbotMpcMrtNode.cpp)
target_link_libraries(ballbot_mpc_mrt ${PROJECT_NAME})
target_compile_options(ballbot_mpc_mrt PRIVATE ${OCS2_CXX_FLAGS})

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  ballbot_ddp
  ballbot_sqp
  ballbot_slp
  ballbot_dummy_test
  ballbot_target
  ballbot_mpc_mrt
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch rviz
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
  ament_index_cpp
  rclcpp
  tf2_ros
  sensor_msgs
  geometry_msgs
  visualization_msgs
  urdf
  kdl_parser
  robot_state_publisher
  ocs2_core
  ocs2_oc
  ocs2_ddp
  ocs2_mpc
  ocs2_sqp
  ocs2_slp
  ocs2_ros_interfaces
  ocs2_robotic_tools
  ocs2_robotic_assets
  ocs2_ballbot
  Eigen3
  Boost
)

ament_package()
