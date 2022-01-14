cmake_minimum_required(VERSION 3.0.2)
project(ocs2_mpc)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  ocs2_core
  ocs2_oc
)
find_package(Boost REQUIRED COMPONENTS
  system
  filesystem
)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

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
    ocs2_core
    ocs2_oc
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

add_library(${PROJECT_NAME}
  src/LoopshapingSystemObservation.cpp
  src/MPC_BASE.cpp
  src/MPC_Settings.cpp
  src/SystemObservation.cpp
  src/MRT_BASE.cpp
  src/MPC_MRT_Interface.cpp
  # src/MPC_OCS2.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

add_executable(${PROJECT_NAME}_lintTarget
  src/lintTarget.cpp
)
target_link_libraries(${PROJECT_NAME}_lintTarget
  ${catkin_LIBRARIES}
)

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  message(STATUS "Run clang tooling for target " ${PROJECT_NAME}_lintTarget)
  add_clang_tooling(
    TARGETS ${PROJECT_NAME}_lintTarget
    SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
    CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
    CF_WERROR
)
endif(cmake_clang_tools_FOUND)

#############
## Install ##
#############

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

#############
## Testing ##
#############

#catkin_add_gtest(testMPC_OCS2
#  test/testMPC_OCS2.cpp
#)
#target_link_libraries(testMPC_OCS2
#  ${catkin_LIBRARIES}
#  ${Boost_LIBRARIES}
#)
#target_compile_options(testMPC_OCS2 PRIVATE ${OCS2_CXX_FLAGS})

