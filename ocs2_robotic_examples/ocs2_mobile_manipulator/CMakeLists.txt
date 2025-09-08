cmake_minimum_required(VERSION 3.14.4)
project(ocs2_mobile_manipulator LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wno-extra -Wpedantic -Wno-reorder -Wno-unused-function)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(ocs2_core REQUIRED)
find_package(ocs2_ddp REQUIRED)
find_package(ocs2_mpc REQUIRED)
find_package(ocs2_robotic_tools REQUIRED)
find_package(ocs2_robotic_assets REQUIRED)
find_package(ocs2_pinocchio_interface REQUIRED)
find_package(ocs2_self_collision REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(PkgConfig REQUIRED)
pkg_check_modules(pinocchio REQUIRED pinocchio)
pkg_check_modules(hpp-fcl REQUIRED hpp-fcl)
find_package(urdf REQUIRED)
find_package(urdfdom REQUIRED)

set(dependencies
  ocs2_core
  ocs2_ddp
  ocs2_mpc
  ocs2_robotic_tools
  ocs2_robotic_assets
  ocs2_pinocchio_interface
  ocs2_self_collision
  Eigen3
  Boost
  urdf
  urdfdom
  hpp-fcl
)

###########
## Build ##
###########

set(FLAGS
  ${OCS2_CXX_FLAGS}
  ${pinocchio_CFLAGS_OTHER}
  -Wno-ignored-attributes
  -Wno-invalid-partial-specialization   # to silence warning with unsupported Eigen Tensor
  -DPINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR
  -DPINOCCHIO_URDFDOM_USE_STD_SHARED_PTR
)

link_directories(
  ${pinocchio_LIBRARY_DIRS}
)

# Resolve for the package path at compile time.
configure_file (
  "${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/package_path.h.in"
  "${PROJECT_BINARY_DIR}/include/${PROJECT_NAME}/package_path.h" @ONLY
)

set(includes_${PROJECT_NAME}
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
add_library(${PROJECT_NAME} SHARED
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
target_include_directories(${PROJECT_NAME} PUBLIC
  ${includes_${PROJECT_NAME}}
)
ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
)
target_compile_options(${PROJECT_NAME} PUBLIC ${FLAGS})

####################
## Clang tooling ###
####################

find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  message(STATUS "Run clang tooling for target " ${PROJECT_NAME})
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

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  DIRECTORY include/ 
  DESTINATION include/${PROJECT_NAME}
)
install(
  DIRECTORY config 
  DESTINATION share/${PROJECT_NAME}
)


ament_export_dependencies(${dependencies})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

#############
## Testing ##
#############
find_package(gtest_vendor REQUIRED)
find_package(ament_cmake_gtest REQUIRED)

set(test_includes_${PROJECT_NAME}
  ${includes_${PROJECT_NAME}}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test>
  $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}/include>
)

function(ament_add_gtest_wrapper test_name)
  ament_add_gtest(${test_name} ${ARGN})
  target_include_directories(${test_name} PUBLIC
    ${test_includes_${PROJECT_NAME}}
  )
  set_tests_properties(${test_name} PROPERTIES TIMEOUT 150)
endfunction()

ament_add_gtest_wrapper(test_self_collision
  test/testSelfCollision.cpp
)
ament_target_dependencies(test_self_collision
  ${dependencies}
)
target_link_libraries(test_self_collision
  ${PROJECT_NAME}
)

ament_add_gtest_wrapper(test_end_effector_constraint
  test/testEndEffectorConstraint.cpp
)
ament_target_dependencies(test_end_effector_constraint
  ${dependencies}
)
target_link_libraries(test_end_effector_constraint
  ${PROJECT_NAME}
)

ament_add_gtest_wrapper(test_dummy_mobile_manipulator
  test/testDummyMobileManipulator.cpp
)
ament_target_dependencies(test_dummy_mobile_manipulator
  ${dependencies}
)
target_link_libraries(test_dummy_mobile_manipulator
  ${PROJECT_NAME}
)

ament_package()
