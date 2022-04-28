cmake_minimum_required(VERSION 3.0)
project(ocs2_anymal_models)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    ocs2_switched_model_interface
    ocs2_robotic_tools
    ocs2_pinocchio_interface
    ocs2_robotic_assets
    roslib
)

## Eigen3
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

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
    ocs2_switched_model_interface
    ocs2_robotic_tools
    ocs2_pinocchio_interface
    ocs2_robotic_assets
    roslib
#   DEPENDS
)

###########
## Build ##
###########

# Resolve for the package path at compile time.
configure_file(
  "${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/package_path.h.in"
  "${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/package_path.h" @ONLY
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/AnymalModels.cpp
  src/FrameDeclaration.cpp
  src/QuadrupedPinocchioMapping.cpp
  src/QuadrupedKinematics.cpp
  src/QuadrupedCom.cpp
  src/bear/AnymalBearCom.cpp
  src/bear/AnymalBearKinematics.cpp
  src/camel/AnymalCamelCom.cpp
  src/camel/AnymalCamelKinematics.cpp
  src/cerberus/AnymalCerberusCom.cpp
  src/cerberus/AnymalCerberusKinematics.cpp
  src/chimera/AnymalChimeraCom.cpp
  src/chimera/AnymalChimeraKinematics.cpp
  src/wheels/AnymalWheelsCom.cpp
  src/wheels/AnymalWheelsKinematics.cpp
  src/wheels_chimera/AnymalWheelsChimeraCom.cpp
  src/wheels_chimera/AnymalWheelsChimeraKinematics.cpp
)
add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
   dl
)
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY kindsl launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

# Build unit tests
catkin_add_gtest(${PROJECT_NAME}_switched_model_test
  test/TestAnymalBear.cpp
  test/TestAnymalCamel.cpp
  test/TestAnymalCerberus.cpp
  test/TestAnymalChimera.cpp
  test/TestAnymalWheels.cpp
  test/TestAnymalWheelsChimera.cpp
  test/TestDynamicsHelpers.cpp
)
target_link_libraries(${PROJECT_NAME}_switched_model_test
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  gtest_main
)

catkin_add_gtest(TestQuadrupedPinocchio
  test/TestQuadrupedPinocchioCom.cpp
  test/TestQuadrupedPinocchioKinematics.cpp
  test/TestFrameDeclaration.cpp
)
target_link_libraries(TestQuadrupedPinocchio
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  gtest_main
)
