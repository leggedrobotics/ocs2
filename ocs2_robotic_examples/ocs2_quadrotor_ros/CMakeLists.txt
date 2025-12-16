cmake_minimum_required(VERSION 3.5)
project(ocs2_quadrotor_ros)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(ocs2_ddp REQUIRED)
find_package(ocs2_mpc REQUIRED)
find_package(ocs2_ros_interfaces REQUIRED)
find_package(ocs2_robotic_tools REQUIRED)
find_package(ocs2_robotic_assets REQUIRED)
find_package(ocs2_quadrotor REQUIRED)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Boost REQUIRED COMPONENTS system filesystem log_setup log)

add_library(${PROJECT_NAME}
  src/QuadrotorDummyVisualization.cpp
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
  geometry_msgs
  ocs2_ddp
  ocs2_mpc
  ocs2_ros_interfaces
  ocs2_robotic_tools
  ocs2_robotic_assets
  ocs2_quadrotor
)
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

add_executable(quadrotor_mpc src/QuadrotorMpcNode.cpp)
target_link_libraries(quadrotor_mpc ${PROJECT_NAME})
target_compile_options(quadrotor_mpc PRIVATE ${OCS2_CXX_FLAGS})

add_executable(quadrotor_dummy_test src/DummyQuadrotorNode.cpp)
target_link_libraries(quadrotor_dummy_test ${PROJECT_NAME})
target_compile_options(quadrotor_dummy_test PRIVATE ${OCS2_CXX_FLAGS})

add_executable(quadrotor_target src/QuadrotorTargetPoseCommand.cpp)
target_link_libraries(quadrotor_target ${PROJECT_NAME})
target_compile_options(quadrotor_target PRIVATE ${OCS2_CXX_FLAGS})

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS quadrotor_mpc quadrotor_dummy_test quadrotor_target
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
  geometry_msgs
  ocs2_ddp
  ocs2_mpc
  ocs2_ros_interfaces
  ocs2_robotic_tools
  ocs2_robotic_assets
  ocs2_quadrotor
  Eigen3
  Boost
)

ament_package()
