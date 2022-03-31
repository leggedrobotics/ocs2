cmake_minimum_required(VERSION 3.0.2)
project(ocs2_mpcnet_core)

set(CATKIN_PACKAGE_DEPENDENCIES
  pybind11_catkin
  ocs2_mpc
  ocs2_python_interface
  ocs2_ros_interfaces
)

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_PACKAGE_DEPENDENCIES}
)

find_package(onnxruntime REQUIRED)

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
  DEPENDS
    onnxruntime
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# main library
add_library(${PROJECT_NAME}
  src/control/MpcnetBehavioralController.cpp
  src/control/MpcnetOnnxController.cpp
  src/dummy/MpcnetDummyLoopRos.cpp
  src/dummy/MpcnetDummyObserverRos.cpp
  src/rollout/MpcnetDataGeneration.cpp
  src/rollout/MpcnetPolicyEvaluation.cpp
  src/rollout/MpcnetRolloutBase.cpp
  src/rollout/MpcnetRolloutManager.cpp
  src/MpcnetInterfaceBase.cpp
)
add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  onnxruntime
)

# python bindings
pybind11_add_module(MpcnetPybindings SHARED
  src/MpcnetPybindings.cpp
)
add_dependencies(MpcnetPybindings
  ${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(MpcnetPybindings PRIVATE
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
set_target_properties(MpcnetPybindings
    PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}
)

catkin_python_setup()

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
   message(STATUS "Run clang tooling for target ocs2_mpcnet_core")
   add_clang_tooling(
     TARGETS ${PROJECT_NAME}
     SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
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
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS MpcnetPybindings
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
)

#############
## Testing ##
#############

# TODO(areske)
