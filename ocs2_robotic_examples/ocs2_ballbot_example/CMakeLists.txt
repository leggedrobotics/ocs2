cmake_minimum_required(VERSION 3.0)
project(ocs2_ballbot_example)

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CATKIN_PACKAGE_DEPENDENCIES
    tf
    urdf
    kdl_parser
    robot_state_publisher
    visualization_msgs
    ocs2_core
    ocs2_ddp
    ocs2_ocs2
    ocs2_mpc
    ocs2_comm_interfaces
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
    ${CATKIN_PACKAGE_DEPENDENCIES}
  LIBRARIES
    ballbot_SLQ
    ocs2_ballbot_dynamics
    ocs2_ballbot
    ocs2_ballbot_visualization
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

# SLQ
add_library(ballbot_SLQ
  src/solvers/BallbotSLQ.cpp
)
add_dependencies(ballbot_SLQ
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(ballbot_SLQ
  ${catkin_LIBRARIES}
)

add_library(ocs2_ballbot_dynamics
  src/dynamics/BallbotSystemDynamics.cpp
)

# Ballbot interface library
add_library(ocs2_ballbot
  src/BallbotInterface.cpp
)
add_dependencies(ocs2_ballbot
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(ocs2_ballbot
  ballbot_SLQ
  ocs2_ballbot_dynamics
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  dl
)

# Ballbot visualization
add_library(ocs2_ballbot_visualization
  src/BallbotDummyVisualization.cpp
)
add_dependencies(ocs2_ballbot_visualization
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(ocs2_ballbot_visualization
  ${catkin_LIBRARIES}
)

# Mpc node
add_executable(ballbot_mpc
  src/MPC_Ballbot_Node.cpp
)
add_dependencies(ballbot_mpc
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(ballbot_mpc
  ocs2_ballbot
  ${catkin_LIBRARIES}
)

# Dummy node
add_executable(ballbot_dummy_test
  src/Dummy_Ballbot_Node.cpp
)
add_dependencies(ballbot_dummy_test
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(ballbot_dummy_test
  ocs2_ballbot
  ocs2_ballbot_visualization
  ${catkin_LIBRARIES}
)

# Target node
add_executable(ballbot_target
  src/BallbotTargetPoseCommand.cpp
)
add_dependencies(ballbot_target
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(ballbot_target
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)
        
# Simple example
add_executable(ballbot_simple_example
  src/BallbotSimpleExample.cpp
)
add_dependencies(ballbot_simple_example
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(ballbot_simple_example
  ballbot_SLQ
  ocs2_ballbot_dynamics
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  dl
)

# Pybindings Cpp library
add_library(ballbotPyBindings_cpp
  src/BallbotPyBindings.cpp
)
add_dependencies(ballbotPyBindings_cpp
  ocs2_ballbot
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(ballbotPyBindings_cpp
  ocs2_ballbot
  ${catkin_LIBRARIES}
)

# Build python bindings if possible
if(${pybind11_FOUND})
  pybind11_add_module(BallbotPyBindings SHARED
    src/pyBindModule.cpp
  )
  add_dependencies(BallbotPyBindings
    ballbotPyBindings_cpp
    ${catkin_EXPORTED_TARGETS}
  )
  target_link_libraries(BallbotPyBindings PRIVATE
    ballbotPyBindings_cpp
    ${catkin_LIBRARIES}
  )

  if(USE_PYBIND_PYTHON_3)
    # unfortunately this appears not to be discoverable yet because the sourcing process assumes python2.7
    set_target_properties(BallbotPyBindings
      PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib/python3.6/dist-packages/${PROJECT_NAME}
    )
  else(USE_PYBIND_PYTHON_3)
    set_target_properties(BallbotPyBindings
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
  message(STATUS "Run clang tooling for target ocs2_ballbot_example")
  add_clang_tooling(
    TARGETS
        ocs2_ballbot
        ballbot_mpc
        ballbot_dummy_test
        ballbot_target
        ballbotPyBindings_cpp
    SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
    CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
    CT_HEADER_EXCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/generated
    CF_WERROR
  )
endif(cmake_clang_tools_FOUND)

#############
## Install ##
#############

install(TARGETS ocs2_ballbot ballbot_SLQ ocs2_ballbot_dynamics ocs2_ballbot_visualization ballbotPyBindings_cpp
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(TARGETS ballbot_mpc ballbot_dummy_test ballbot_target
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY urdf launch config rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(${pybind11_FOUND})
  # TODO handle python2/3 case properly
  install(TARGETS BallbotPyBindings
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
  )
endif()

#############
## Testing ##
#############

catkin_add_gtest(ballbot_integration_test
  test/ballbotIntegrationTest.cpp
)
target_link_libraries(ballbot_integration_test
  ocs2_ballbot
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)


catkin_add_gtest(ballbot_pybindings_test
  test/PyBindingsTest.cpp
)
target_link_libraries(ballbot_pybindings_test
  ballbotPyBindings_cpp
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)
