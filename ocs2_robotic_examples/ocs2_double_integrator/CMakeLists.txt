cmake_minimum_required(VERSION 3.0.2)
project(ocs2_double_integrator)

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CATKIN_PACKAGE_DEPENDENCIES
  pybind11_catkin
  ocs2_core
  ocs2_mpc
  ocs2_ddp
  ocs2_python_interface
  ocs2_robotic_tools
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
  LIBRARIES
    ${PROJECT_NAME}
)

###########
## Build ##
###########

# Resolve for the package path at compile time.
configure_file (
  "${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/package_path.h.in"
  "${PROJECT_BINARY_DIR}/include/${PROJECT_NAME}/package_path.h" @ONLY
)

# Add directories for all targets
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# double integrator interface library
add_library(${PROJECT_NAME}
  src/DoubleIntegratorInterface.cpp
)
add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  dl
)
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})


# python bindings
pybind11_add_module(DoubleIntegratorPyBindings SHARED
  src/pyBindModule.cpp
)
add_dependencies(DoubleIntegratorPyBindings
  ${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(DoubleIntegratorPyBindings PRIVATE
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
set_target_properties(DoubleIntegratorPyBindings
    PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}
)

catkin_python_setup()

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  message(STATUS "Run clang tooling for target ocs2_double_integrator")
  add_clang_tooling(
    TARGETS
      ${PROJECT_NAME}
    SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/test
    CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
    CF_WERROR
  )
endif(cmake_clang_tools_FOUND)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(TARGETS DoubleIntegratorPyBindings
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
)

#############
## Testing ##
#############

catkin_add_gtest(ocs2_double_integrator_no_ros_integration_test
  test/DoubleIntegratorNoRosIntegrationTest.cpp
)
add_dependencies(ocs2_double_integrator_no_ros_integration_test
  ${catkin_EXPORTED_TARGETS}
)
target_include_directories(ocs2_double_integrator_no_ros_integration_test
  PRIVATE ${PROJECT_BINARY_DIR}/include
)
target_link_libraries(ocs2_double_integrator_no_ros_integration_test
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  gtest_main
)

catkin_add_gtest(ocs2_double_integrator_pybinding_test
  test/DoubleIntegratorPyBindingTest.cpp
)
add_dependencies(ocs2_double_integrator_pybinding_test
  ${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)
target_include_directories(ocs2_double_integrator_pybinding_test
  PRIVATE ${PROJECT_BINARY_DIR}/include
)
target_link_libraries(ocs2_double_integrator_pybinding_test
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# python tests
catkin_add_nosetests(test/DoubleIntegratorPyBindingTest.py)
