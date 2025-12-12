cmake_minimum_required(VERSION 3.5)
project(ocs2_sphere_approximation)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(ocs2_core REQUIRED)
find_package(ocs2_robotic_tools REQUIRED)
find_package(ocs2_pinocchio_interface REQUIRED)
find_package(ocs2_robotic_assets REQUIRED)
find_package(pinocchio REQUIRED)
find_package(tinyxml2 REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Boost REQUIRED COMPONENTS system filesystem log_setup log)

add_library(${PROJECT_NAME}
  src/SphereApproximation.cpp
  src/PinocchioSphereInterface.cpp
  src/PinocchioSphereKinematics.cpp
  src/PinocchioSphereKinematicsCppAd.cpp
)
target_link_libraries(${PROJECT_NAME}
  pinocchio::pinocchio
  tinyxml2::tinyxml2
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
  ocs2_pinocchio_interface
  ocs2_robotic_assets
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

ament_export_include_directories(
  include
  ${pinocchio_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  ocs2_core
  ocs2_robotic_tools
  ocs2_pinocchio_interface
  ocs2_robotic_assets
  Eigen3
  Boost
  pinocchio
  tinyxml2
)

############
# Testing ##
############

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(PinocchioSphereKinematicsTest
    test/testPinocchioSphereKinematics.cpp
  )
  target_link_libraries(PinocchioSphereKinematicsTest
    gtest_main
    ${PROJECT_NAME}
    pinocchio::pinocchio
  )
  target_compile_options(PinocchioSphereKinematicsTest PRIVATE ${OCS2_PINOCCHIO_FLAGS})
endif()

ament_package()
