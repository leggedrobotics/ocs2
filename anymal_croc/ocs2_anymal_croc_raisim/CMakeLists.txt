cmake_minimum_required(VERSION 3.0)
project(ocs2_anymal_croc_raisim)

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Catkin packages we depend on
set(CATKIN_PACKAGE_DEPENDENCIES
  ocs2_anymal_croc
  ocs2_anymal_raisim
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
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
  LIBRARIES
    ${PROJECT_NAME}
)


###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/inputToRaisimGeneralizedForce.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})


# Dummy node with raisim rollout
add_executable(anymal_dummy_raisim
  src/AnymalRaisimDummy.cpp
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
  message(STATUS "Run clang tooling for target cartpole_dummy_raisim")
  add_clang_tooling(
    TARGETS anymal_dummy_raisim
    SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
    CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
    CF_WERROR
    CT_WERROR
)
endif(cmake_clang_tools_FOUND)

#############
## Install ##
#############

install(TARGETS anymal_dummy_raisim
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

