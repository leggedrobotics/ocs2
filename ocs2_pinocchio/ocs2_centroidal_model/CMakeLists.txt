cmake_minimum_required(VERSION 3.0.2)
project(ocs2_centroidal_model)

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CATKIN_PACKAGE_DEPENDENCIES
  roscpp
  roslib
  ocs2_core
  ocs2_robotic_tools
  ocs2_pinocchio_interface
)

find_package(catkin REQUIRED
  ${CATKIN_PACKAGE_DEPENDENCIES}
)

find_package(Boost REQUIRED COMPONENTS
  system
  filesystem
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(pinocchio REQUIRED pinocchio)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

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
    pinocchio
    Boost
)

###########
## Build ##
###########

include_directories(
  include
  test/include
  ${pinocchio_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

link_directories(
  ${pinocchio_LIBRARY_DIRS}
)

add_library(${PROJECT_NAME}
  src/PinocchioCentroidalDynamics.cpp
  src/PinocchioCentroidalDynamicsAD.cpp
  src/CentroidalModelRbdConversions.cpp
  src/CentroidalModelPinocchioMapping.cpp
  src/CentroidalModelInfo.cpp
  src/FactoryFunctions.cpp
  src/ModelHelperFunctions.cpp
  src/PinocchioCentroidalInverseDynamicsPD.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${pinocchio_LIBRARIES}
)
target_compile_options(${PROJECT_NAME} PUBLIC
  ${OCS2_CXX_FLAGS}
  ${pinocchio_CFLAGS_OTHER}
  -Wno-invalid-partial-specialization # to silence warning with unsupported Eigen Tensor
)

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  message(STATUS "Run clang tooling for target " ${PROJECT_NAME})
  add_clang_tooling(
          TARGETS
          ${PROJECT_NAME}
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

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

#############
## Testing ##
#############
catkin_add_gtest(${PROJECT_NAME}_test
  # test/testAccessMethods.cpp
  test/testAnymalSingleRigidBody.cpp
  test/testAnymalFullCentroidal.cpp
)
target_link_libraries(${PROJECT_NAME}_test
  gtest_main
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${pinocchio_LIBRARIES}
)
target_compile_options(${PROJECT_NAME}_test PUBLIC
  ${OCS2_CXX_FLAGS}
  ${pinocchio_CFLAGS_OTHER}
)
