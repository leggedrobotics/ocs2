cmake_minimum_required(VERSION 3.5)
project(ocs2_mobile_manipulator)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(ocs2_core REQUIRED)
find_package(ocs2_ddp REQUIRED)
find_package(ocs2_mpc REQUIRED)
find_package(ocs2_robotic_tools REQUIRED)
find_package(ocs2_robotic_assets REQUIRED)
find_package(ocs2_pinocchio_interface REQUIRED)
find_package(ocs2_self_collision REQUIRED)
find_package(pinocchio REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Boost REQUIRED COMPONENTS system filesystem log_setup log)

# Resolve for the package path at compile time.
configure_file(
  "${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/package_path.h.in"
  "${PROJECT_BINARY_DIR}/include/${PROJECT_NAME}/package_path.h"
  @ONLY
)

add_library(${PROJECT_NAME}
  src/FactoryFunctions.cpp
  src/MobileManipulatorPreComputation.cpp
  src/MobileManipulatorPinocchioMapping.cpp
  src/constraint/EndEffectorConstraint.cpp
  src/dynamics/WheelBasedMobileManipulatorDynamics.cpp
  src/dynamics/DefaultManipulatorDynamics.cpp
  src/dynamics/FloatingArmManipulatorDynamics.cpp
  src/dynamics/FullyActuatedFloatingArmManipulatorDynamics.cpp
  src/MobileManipulatorInterface.cpp
)
target_link_libraries(${PROJECT_NAME}
  pinocchio::pinocchio
  Boost::filesystem
  Boost::system
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIRS}
    ${pinocchio_INCLUDE_DIRS}
)
ament_target_dependencies(${PROJECT_NAME}
  ocs2_core
  ocs2_ddp
  ocs2_mpc
  ocs2_robotic_tools
  ocs2_robotic_assets
  ocs2_pinocchio_interface
  ocs2_self_collision
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
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  EXPORT export_${PROJECT_NAME}
  DESTINATION share/${PROJECT_NAME}/cmake
)

ament_export_include_directories(
  include
  ${PROJECT_BINARY_DIR}/include
  ${pinocchio_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  ocs2_core
  ocs2_ddp
  ocs2_mpc
  ocs2_robotic_tools
  ocs2_robotic_assets
  ocs2_pinocchio_interface
  ocs2_self_collision
  pinocchio
  Eigen3
  Boost
)

############
# Testing ##
############

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  macro(add_ocs2_test APP_NAME APP_SRC)
    ament_add_gtest(${APP_NAME}
      ${APP_SRC}
    )
    target_include_directories(${APP_NAME} PRIVATE
      ${PROJECT_BINARY_DIR}/include
    )
    target_link_libraries(${APP_NAME}
      gtest_main
      ${PROJECT_NAME}
    )
  endmacro()

  add_ocs2_test(SelfCollisionTest test/testSelfCollision.cpp)
  add_ocs2_test(EndEffectorConstraintTest test/testEndEffectorConstraint.cpp)
  add_ocs2_test(DummyMobileManipulatorTest test/testDummyMobileManipulator.cpp)
  set_tests_properties(DummyMobileManipulatorTest PROPERTIES TIMEOUT 600)
endif()

ament_package()
