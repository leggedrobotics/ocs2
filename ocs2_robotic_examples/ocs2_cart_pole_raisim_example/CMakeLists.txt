cmake_minimum_required(VERSION 3.0.2)
project(ocs2_cart_pole_raisim_example)

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Catkin packages we depend on
set(CATKIN_PACKAGE_DEPENDENCIES
  ocs2_cart_pole_example
  ocs2_raisim
  roslib
)

find_package(catkin REQUIRED COMPONENTS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS
    include
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

# Dummy node with raisim rollout
add_executable(cartpole_dummy_raisim
  src/DummyCartpoleRaisimNode.cpp
)
target_link_libraries(cartpole_dummy_raisim
  ${catkin_LIBRARIES}
)
target_compile_options(cartpole_dummy_raisim PRIVATE ${OCS2_CXX_FLAGS})



#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  message(STATUS "Run clang tooling for target cartpole_dummy_raisim")
  add_clang_tooling(
    TARGETS cartpole_dummy_raisim
    SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
    CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
    CF_WERROR
)
endif(cmake_clang_tools_FOUND)

#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS cartpole_dummy_raisim
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############
catkin_add_gtest(cartpole_raisim_rollout
  test/RolloutTest.cpp
)
target_link_libraries(cartpole_raisim_rollout
  ${catkin_LIBRARIES}
  gtest_main
)
target_compile_options(cartpole_raisim_rollout PRIVATE ${OCS2_CXX_FLAGS})