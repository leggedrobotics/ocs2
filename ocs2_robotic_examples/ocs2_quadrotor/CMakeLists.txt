cmake_minimum_required(VERSION 3.0.2)
project(ocs2_quadrotor)

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CATKIN_PACKAGE_DEPENDENCIES
  tf
  pybind11_catkin
  ocs2_mpc
  ocs2_ros_interfaces
  ocs2_robotic_tools
  ocs2_python_interface
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
    quadrotor_interface
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
  src/QuadrotorSystemDynamics.cpp
)
add_dependencies(quadrotor_interface
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(quadrotor_interface
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  dl
)
target_compile_options(quadrotor_interface PUBLIC ${OCS2_CXX_FLAGS})

# MPC node
add_executable(quadrotor_mpc
  src/QuadrotorMpcNode.cpp
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
  src/DummyQuadrotorNode.cpp
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
target_compile_options(quadrotor_target PRIVATE ${OCS2_CXX_FLAGS})

# python bindings
pybind11_add_module(QuadrotorPyBindings SHARED
  src/pyBindModule.cpp
)
add_dependencies(QuadrotorPyBindings
  quadrotor_interface
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(QuadrotorPyBindings PRIVATE
  quadrotor_interface
  ${catkin_LIBRARIES}
)
set_target_properties(QuadrotorPyBindings
    PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}
)

catkin_python_setup()

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  message(STATUS "Run clang tooling for target ocs2_quadrotor")
  add_clang_tooling(
    TARGETS
      quadrotor_interface
      quadrotor_mpc
      quadrotor_dummy_test
      quadrotor_target
    SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
    CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
    CF_WERROR
  )
endif(cmake_clang_tools_FOUND)

#############
## Install ##
#############

install(TARGETS quadrotor_interface
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

install(TARGETS QuadrotorPyBindings
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
)

#############
## Testing ##
#############

catkin_add_gtest(${PROJECT_NAME}_PyBindingsTest
  test/PyBindingsTest.cpp
)
target_link_libraries(${PROJECT_NAME}_PyBindingsTest
  quadrotor_interface
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)
