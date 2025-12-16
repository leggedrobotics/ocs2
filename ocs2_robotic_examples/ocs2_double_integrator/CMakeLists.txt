cmake_minimum_required(VERSION 3.5)
project(ocs2_double_integrator)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development.Module)
find_package(pybind11 REQUIRED)
find_package(ocs2_core REQUIRED)
find_package(ocs2_mpc REQUIRED)
find_package(ocs2_ddp REQUIRED)
find_package(ocs2_python_interface REQUIRED)
find_package(ocs2_robotic_tools REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Boost REQUIRED COMPONENTS system filesystem log_setup log)

# Resolve for the package path at compile time.
configure_file(
  "${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/package_path.h.in"
  "${PROJECT_BINARY_DIR}/include/${PROJECT_NAME}/package_path.h"
  @ONLY
)

add_library(${PROJECT_NAME}
  src/DoubleIntegratorInterface.cpp
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(${PROJECT_NAME}
  ocs2_core
  ocs2_mpc
  ocs2_ddp
  ocs2_python_interface
  ocs2_robotic_tools
)
target_link_libraries(${PROJECT_NAME}
  Boost::filesystem
  Boost::system
  dl
)
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

# python bindings
pybind11_add_module(DoubleIntegratorPyBindings
  src/pyBindModule.cpp
)
target_link_libraries(DoubleIntegratorPyBindings PRIVATE
  ${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME}
  PACKAGE_DIR src/${PROJECT_NAME}
)

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
install(TARGETS DoubleIntegratorPyBindings
  LIBRARY DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME}
)

install(
  EXPORT export_${PROJECT_NAME}
  DESTINATION share/${PROJECT_NAME}/cmake
)

ament_export_include_directories(
  include
  ${PROJECT_BINARY_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  ocs2_core
  ocs2_mpc
  ocs2_ddp
  ocs2_python_interface
  ocs2_robotic_tools
  Eigen3
  Boost
  pybind11_vendor
)

############
# Testing ##
############

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(ocs2_double_integrator_no_ros_integration_test
    test/DoubleIntegratorNoRosIntegrationTest.cpp
  )
  target_include_directories(ocs2_double_integrator_no_ros_integration_test
    PRIVATE ${PROJECT_BINARY_DIR}/include
  )
  target_link_libraries(ocs2_double_integrator_no_ros_integration_test
    gtest_main
    ${PROJECT_NAME}
  )

  ament_add_gtest(ocs2_double_integrator_pybinding_test
    test/DoubleIntegratorPyBindingTest.cpp
  )
  target_include_directories(ocs2_double_integrator_pybinding_test
    PRIVATE ${PROJECT_BINARY_DIR}/include
  )
  target_link_libraries(ocs2_double_integrator_pybinding_test
    ${PROJECT_NAME}
  )
endif()

ament_package()
