cmake_minimum_required(VERSION 3.0)
project(ocs2_anymal_mpc)

find_package(catkin REQUIRED COMPONENTS
  roslib
  pybind11_catkin
  ocs2_anymal_models
  ocs2_quadruped_interface
  ocs2_python_interface
)

## Eigen3
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

find_package(Boost REQUIRED COMPONENTS
  filesystem
)

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS
    include
    ${EIGEN3_INCLUDE_DIRS}
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ocs2_anymal_models
    ocs2_quadruped_interface
    ocs2_python_interface
  DEPENDS
    Boost
)

###########
## Build ##
###########

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# Declare a C++ library
add_library(${PROJECT_NAME}
  src/AnymalInterface.cpp
)
add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

add_executable(${PROJECT_NAME}_mpc_node
  src/AnymalMpcNode.cpp
)
add_dependencies(${PROJECT_NAME}_mpc_node
  ${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}_mpc_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_dummy_mrt_node
  src/AnymalDummyMRT.cpp
)
add_dependencies(${PROJECT_NAME}_dummy_mrt_node
  ${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}_dummy_mrt_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_wheels_dummy_visualization_node
  src/AnymalWheelsDummyVisualization.cpp
)
add_dependencies(${PROJECT_NAME}_wheels_dummy_visualization_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}_wheels_dummy_visualization_node
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_test_terrain_node
  src/testTerrainNode.cpp
  )
add_dependencies(${PROJECT_NAME}_test_terrain_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )
target_link_libraries(${PROJECT_NAME}_test_terrain_node
  ${catkin_LIBRARIES}
  )

# PyBindings library (c++ side)
add_library(${PROJECT_NAME}_PyBindings_cpp
  src/AnymalPyBindings.cpp
)
add_dependencies(${PROJECT_NAME}_PyBindings_cpp
  ${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}_PyBindings_cpp
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# Build python bindings
pybind11_add_module(AnymalPyBindings SHARED
  src/pyBindModule.cpp
)
add_dependencies(AnymalPyBindings
  ${PROJECT_NAME}_PyBindings_cpp
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(AnymalPyBindings PRIVATE
  ${PROJECT_NAME}_PyBindings_cpp
  ${catkin_LIBRARIES}
)

set_target_properties(AnymalPyBindings
    PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}
)
catkin_python_setup()

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
  message(STATUS "Run clang tooling")
  add_clang_tooling(
    TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_mpc_node ${PROJECT_NAME}_dummy_mrt_node
    SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
    CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
    CF_WERROR
  )
endif (cmake_clang_tools_FOUND)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS ${PROJECT_NAME}_mpc_node ${PROJECT_NAME}_dummy_mrt_node ${PROJECT_NAME}_wheels_dummy_visualization_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(TARGETS AnymalPyBindings
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
)

#############
## Testing ##
#############

catkin_add_nosetests(test)

# Build unit tests
catkin_add_gtest(${PROJECT_NAME}_test
  test/testProblemFormulation.cpp
  )
target_link_libraries(${PROJECT_NAME}_test
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  gtest_main
  )
