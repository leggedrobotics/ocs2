cmake_minimum_required(VERSION 3.0)
project(ocs2_quadrotor_example)

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED COMPONENTS
  roslib
  tf
  ocs2_mpc
  ocs2_comm_interfaces
  ocs2_robotic_tools
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

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fopenmp -std=c++11 -Wfatal-errors -Wl,--no-as-needed")

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS
    include
    ${EIGEN3_INCLUDE_DIRS}
  CATKIN_DEPENDS
    tf
    ocs2_mpc
    ocs2_comm_interfaces
    ocs2_robotic_tools
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

# Quadrotor interface library
add_library(quadrotor_interface
  src/QuadrotorInterface.cpp
)
add_dependencies(quadrotor_interface
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(quadrotor_interface
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  dl
)

# MPC node
add_executable(quadrotor_mpc
  src/MPC_Quadrotor_Node.cpp
)
add_dependencies(quadrotor_mpc
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(quadrotor_mpc
  quadrotor_interface
  ${catkin_LIBRARIES}
)

# Dummy node
add_executable(quadrotor_dummy_test
  src/Dummy_Quadrotor_Node.cpp
  src/QuadrotorDummyVisualization.cpp
)
add_dependencies(quadrotor_dummy_test
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(quadrotor_dummy_test
  quadrotor_interface
  ${catkin_LIBRARIES}
)

# Target node
add_executable(quadrotor_target
  src/QuadrotorTargetPoseCommand.cpp
)
add_dependencies(quadrotor_target
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(quadrotor_target
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

# PyBindings library
add_library(quadrotorPyBindings_Cpp
  src/QuadrotorPyBindings.cpp
)
add_dependencies(quadrotorPyBindings_Cpp
  quadrotor_interface
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(quadrotorPyBindings_Cpp
  quadrotor_interface
  ${catkin_LIBRARIES}
)

if(${pybind11_FOUND})
  pybind11_add_module(QuadrotorPyBindings SHARED
    src/pyBindModule.cpp
  )
  add_dependencies(QuadrotorPyBindings
    quadrotorPyBindings_Cpp
    ${catkin_EXPORTED_TARGETS}
  )
  target_link_libraries(QuadrotorPyBindings PRIVATE
    quadrotorPyBindings_Cpp
    ${catkin_LIBRARIES}
  )

  if(USE_PYBIND_PYTHON_3)
  # unfortunately this appears not to be discoverable yet because the sourcing process assumes python2.7
  set_target_properties(QuadrotorPyBindings
    PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib/python3.6/dist-packages/${PROJECT_NAME}
  )
  else(USE_PYBIND_PYTHON_3)
  set_target_properties(QuadrotorPyBindings
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
  message(STATUS "Run clang tooling for target ocs2_quadrotor_example")
  add_clang_tooling(
    TARGETS
      quadrotor_interface
      quadrotor_mpc
      quadrotor_dummy_test
      quadrotor_target
      quadrotorPyBindings_Cpp
    SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
    CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
    CF_WERROR
  )
endif(cmake_clang_tools_FOUND)

#############
## Install ##
#############

install(TARGETS quadrotor_interface quadrotorPyBindings_Cpp
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(TARGETS quadrotor_mpc quadrotor_dummy_test quadrotor_target
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY urdf launch config rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(${pybind11_FOUND})
  # TODO handle python2/3 case properly
  install(TARGETS QuadrotorPyBindings
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
  )
endif()

#############
## Testing ##
#############

catkin_add_gtest(${PROJECT_NAME}_PyBindingsTest
  test/PyBindingsTest.cpp
)
target_link_libraries(${PROJECT_NAME}_PyBindingsTest
  quadrotorPyBindings_Cpp
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)
