cmake_minimum_required(VERSION 3.0)
project(ocs2_anymal_raisim)

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Catkin packages we depend on
set(CATKIN_PACKAGE_DEPENDENCIES
  roslib
  ocs2_raisim_core
  ocs2_raisim_ros
  ocs2_quadruped_interface
  ocs2_anymal_models
  ocs2_anymal_mpc
)

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_PACKAGE_DEPENDENCIES}
)

# Raisim Visualizer
option(USE_RAISIM_VISUALIZER "Compile Raisim with OGRE Visualizer" OFF)
if(USE_RAISIM_VISUALIZER)
  find_package(raisimOgre REQUIRED)
  add_definitions(-DUSE_RAISIM_VISUALIZER)
endif(USE_RAISIM_VISUALIZER)

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
)


###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/AnymalRaisimConversions.cpp
  src/AnymalRaisimDummy.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
if(USE_RAISIM_VISUALIZER)
  target_link_libraries(${PROJECT_NAME}
    raisim::raisimOgre
  )
endif(USE_RAISIM_VISUALIZER)
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

# Dummy node with raisim rollout
add_executable(anymal_dummy_raisim
  src/AnymalRaisimDummyNode.cpp
  )
target_link_libraries(anymal_dummy_raisim
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
  )
if(USE_RAISIM_VISUALIZER)
  target_link_libraries(anymal_dummy_raisim
    raisim::raisimOgre
  )
endif(USE_RAISIM_VISUALIZER)


#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  message(STATUS "Run clang tooling for ${PROJECT_NAME}")
  add_clang_tooling(
    TARGETS ${PROJECT_NAME}
    SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
    CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
    CF_WERROR
    CT_WERROR
)
endif(cmake_clang_tools_FOUND)

#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############

# Build unit tests
catkin_add_gtest(${PROJECT_NAME}_test_conversions
  test/testAnymalRaisimConversions.cpp
)
target_link_libraries(${PROJECT_NAME}_test_conversions
  gtest_main
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
