cmake_minimum_required(VERSION 3.5)
project(ocs2_pinocchio_interface)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(ocs2_core REQUIRED)
find_package(ocs2_robotic_tools REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Boost REQUIRED COMPONENTS system filesystem log_setup log)

find_package(pinocchio REQUIRED)

# Add pinocchio configurations
include(cmake/pinocchio_config.cmake)

# ocs2 pinocchio interface library
add_library(${PROJECT_NAME}
  src/PinocchioInterface.cpp
  src/PinocchioInterfaceCppAd.cpp
  src/PinocchioEndEffectorKinematics.cpp
  src/PinocchioEndEffectorKinematicsCppAd.cpp
  src/urdf.cpp
)
target_link_libraries(${PROJECT_NAME}
  pinocchio::pinocchio
  urdfdom_model
  urdfdom_world
  urdfdom_sensor
  urdfdom_model_state
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIRS}
    ${pinocchio_INCLUDE_DIRS}
)
ament_target_dependencies(${PROJECT_NAME}
  ocs2_core
  ocs2_robotic_tools
)
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_PINOCCHIO_FLAGS})

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(
  EXPORT export_${PROJECT_NAME}
  DESTINATION share/${PROJECT_NAME}/cmake
)

install(FILES cmake/pinocchio_config.cmake
  DESTINATION share/${PROJECT_NAME}/cmake
)

ament_export_include_directories(
  include
  ${pinocchio_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  ocs2_core
  ocs2_robotic_tools
  Eigen3
  Boost
  pinocchio
)

############
# Testing ##
############

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

ament_add_gtest(testPinocchioInterface
  test/testPinocchioInterface.cpp
  test/testPinocchioEndEffectorKinematics.cpp
)
target_link_libraries(testPinocchioInterface
  gtest_main
  ${PROJECT_NAME}
  pinocchio::pinocchio
  urdfdom_model
  urdfdom_world
  urdfdom_sensor
  urdfdom_model_state
)
endif()

ament_package(CONFIG_EXTRAS "cmake/pinocchio_config.cmake")
