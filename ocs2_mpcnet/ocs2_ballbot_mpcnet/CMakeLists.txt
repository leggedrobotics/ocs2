cmake_minimum_required(VERSION 3.0.2)
project(ocs2_ballbot_mpcnet)

set(CATKIN_PACKAGE_DEPENDENCIES
  ocs2_ballbot
  ocs2_ballbot_ros
  ocs2_mpcnet_core
)

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_PACKAGE_DEPENDENCIES}
)

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
  src/BallbotMpcnetDefinition.cpp
  src/BallbotMpcnetInterface.cpp
)
add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# python bindings
pybind11_add_module(BallbotMpcnetPybindings SHARED
  src/BallbotMpcnetPybindings.cpp
)
add_dependencies(BallbotMpcnetPybindings
  ${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(BallbotMpcnetPybindings PRIVATE
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
set_target_properties(BallbotMpcnetPybindings
    PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}
)

# MPC-Net dummy node
add_executable(ballbot_mpcnet_dummy
  src/BallbotMpcnetDummyNode.cpp
)
add_dependencies(ballbot_mpcnet_dummy
  ${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(ballbot_mpcnet_dummy
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
target_compile_options(ballbot_mpcnet_dummy PRIVATE ${OCS2_CXX_FLAGS})

catkin_python_setup()

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
   message(STATUS "Run clang tooling for target ocs2_ballbot_mpcnet")
   add_clang_tooling(
     TARGETS ${PROJECT_NAME} ballbot_mpcnet_dummy
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

install(TARGETS BallbotMpcnetPybindings
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
)

install(TARGETS ballbot_mpcnet_dummy
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch policy
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

# TODO(areske)
