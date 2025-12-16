cmake_minimum_required(VERSION 3.5)
project(ocs2_cartpole_ros)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(ocs2_core REQUIRED)
find_package(ocs2_ddp REQUIRED)
find_package(ocs2_mpc REQUIRED)
find_package(ocs2_ros_interfaces REQUIRED)
find_package(ocs2_robotic_tools REQUIRED)
find_package(ocs2_cartpole REQUIRED)
find_package(ocs2_robotic_assets REQUIRED)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Boost REQUIRED COMPONENTS system filesystem log_setup log)

add_library(${PROJECT_NAME}
  src/CartpoleDummyVisualization.cpp
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
  sensor_msgs
  ocs2_core
  ocs2_ddp
  ocs2_mpc
  ocs2_ros_interfaces
  ocs2_robotic_tools
  ocs2_cartpole
  ocs2_robotic_assets
)
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

add_executable(cartpole_mpc src/CartpoleMpcNode.cpp)
target_link_libraries(cartpole_mpc ${PROJECT_NAME})
target_compile_options(cartpole_mpc PRIVATE ${OCS2_CXX_FLAGS})

add_executable(cartpole_dummy_test src/DummyCartpoleNode.cpp)
target_link_libraries(cartpole_dummy_test ${PROJECT_NAME})
target_compile_options(cartpole_dummy_test PRIVATE ${OCS2_CXX_FLAGS})

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS cartpole_mpc cartpole_dummy_test
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
  sensor_msgs
  ocs2_core
  ocs2_ddp
  ocs2_mpc
  ocs2_ros_interfaces
  ocs2_robotic_tools
  ocs2_cartpole
  ocs2_robotic_assets
  Eigen3
  Boost
)

ament_package()
