cmake_minimum_required(VERSION 3.0)
project(ocs2_double_integrator_example)

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CATKIN_PACKAGE_DEPENDENCIES
  roslib
  ocs2_core
  ocs2_ddp
  ocs2_ocs2
  ocs2_mpc
  ocs2_comm_interfaces
  ocs2_robotic_tools
)

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_PACKAGE_DEPENDENCIES}
)

find_package(Boost REQUIRED COMPONENTS
  system
  filesystem
)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Pybind11
option(USE_PYBIND_PYTHON_3 "Use python3-compatible python bindings" OFF)
if(USE_PYBIND_PYTHON_3)
  set(PYTHON_EXECUTABLE /usr/bin/python3)
  set(PYBIND11_PYTHON_VERSION 3.6 CACHE STRING "")
endif(USE_PYBIND_PYTHON_3)
find_package(pybind11 QUIET)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fopenmp -Wfatal-errors -Wl,--no-as-needed")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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

# double integrator interface library
add_library(double_integrator_interface
  src/DoubleIntegratorInterface.cpp
)
add_dependencies(double_integrator_interface
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(double_integrator_interface
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  dl
)

# Mpc node
add_executable(double_integrator_mpc
  src/MPC_Double_Integrator_Node.cpp
)
add_dependencies(double_integrator_mpc
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(double_integrator_mpc
  double_integrator_interface
  ${catkin_LIBRARIES}
)

# Dummy node
add_executable(double_integrator_dummy_test
  src/Dummy_Double_Integrator_Node.cpp
  src/DoubleIntegratorDummyVisualization.cpp
)
add_dependencies(double_integrator_dummy_test
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(double_integrator_dummy_test
  double_integrator_interface
  ${catkin_LIBRARIES}
)

# Target node
add_executable(double_integrator_target
  src/DoubleIntegratorTargetPoseCommand.cpp
)
add_dependencies(double_integrator_target
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(double_integrator_target
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

# Pybindings Cpp library
add_library(DoubleIntegratorPyBindings_Cpp
  src/DoubleIntegratorPyBindings.cpp
)
add_dependencies(DoubleIntegratorPyBindings_Cpp
  double_integrator_interface
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(DoubleIntegratorPyBindings_Cpp
  double_integrator_interface
  ${catkin_LIBRARIES}
)

# Build python bindings if possible
if(${pybind11_FOUND})
  pybind11_add_module(DoubleIntegratorPyBindings SHARED
    src/pyBindModule.cpp
  )
  add_dependencies(DoubleIntegratorPyBindings
    DoubleIntegratorPyBindings_Cpp
    ${catkin_EXPORTED_TARGETS}
  )
  target_link_libraries(DoubleIntegratorPyBindings PRIVATE
    DoubleIntegratorPyBindings_Cpp
    ${catkin_LIBRARIES}
  )

  if(USE_PYBIND_PYTHON_3)
    # unfortunately this appears not to be discoverable yet because the sourcing process assumes python2.7
    set_target_properties(DoubleIntegratorPyBindings
      PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib/python3.6/dist-packages/${PROJECT_NAME}
    )
  else(USE_PYBIND_PYTHON_3)
    set_target_properties(DoubleIntegratorPyBindings
        PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}
    )
  endif(USE_PYBIND_PYTHON_3)

  catkin_python_setup()
endif()

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  message(STATUS "Run clang tooling for target ocs2_double_integrator_example")
  add_clang_tooling(
    TARGETS
      double_integrator_interface
      double_integrator_mpc
      double_integrator_dummy_test
      double_integrator_target
    SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/test
    CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
    CF_WERROR
  )
endif(cmake_clang_tools_FOUND)

#############
## Install ##
#############

install(TARGETS double_integrator_interface DoubleIntegratorPyBindings_Cpp
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(TARGETS 
  double_integrator_mpc
  double_integrator_dummy_test
  double_integrator_target
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY urdf launch config rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(${pybind11_FOUND})
  # TODO handle python2/3 case properly
  install(TARGETS DoubleIntegratorPyBindings
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
  )
endif()

#############
## Testing ##
#############

catkin_add_gtest(ocs2_double_integrator_no_ros_integration_test
  test/DoubleIntegratorNoRosIntegrationTest.cpp
  )
add_dependencies(ocs2_double_integrator_no_ros_integration_test
  ${catkin_EXPORTED_TARGETS}
  )
target_link_libraries(ocs2_double_integrator_no_ros_integration_test
  double_integrator_interface
  ${catkin_LIBRARIES}
  )

catkin_add_gtest(ocs2_double_integrator_pybinding_test
  test/DoubleIntegratorPyBindingTest.cpp
  )
add_dependencies(ocs2_double_integrator_pybinding_test
  DoubleIntegratorPyBindings_Cpp
  ${catkin_EXPORTED_TARGETS}
  )
target_link_libraries(ocs2_double_integrator_pybinding_test
  DoubleIntegratorPyBindings_Cpp
  ${catkin_LIBRARIES}
  )

if(${pybind11_FOUND})
  if (CATKIN_ENABLE_TESTING)
    catkin_add_nosetests(test/DoubleIntegratorPyBindingTest.py)
  endif(CATKIN_ENABLE_TESTING)
endif(${pybind11_FOUND})
