cmake_minimum_required(VERSION 3.0)
project(ocs2_switched_model_interface)

## Catkin Dependencies
set(CATKIN_PACKAGE_DEPENDENCIES
  roscpp
  ocs2_core
  ocs2_msgs
  ocs2_oc
)
find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_PACKAGE_DEPENDENCIES}
)

## Eigen3
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

###################################
## catkin specific configuration ##
###################################
catkin_package(
	INCLUDE_DIRS
		include
		${EIGEN_INCLUDE_DIRS}
	LIBRARIES
		${PROJECT_NAME}
	CATKIN_DEPENDS
		${CATKIN_PACKAGE_DEPENDENCIES}
#	DEPENDS
)

###########
## Build ##
###########
include_directories(
	include
	${EIGEN_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
	src/foot_planner/CubicSpline.cpp
	src/foot_planner/FootPhase.cpp
	src/foot_planner/SplineCpg.cpp
	src/foot_planner/SwingTrajectoryPlanner.cpp
	src/logic/Gait.cpp
	src/logic/GaitReceiver.cpp
	src/logic/GaitSchedule.cpp
	src/logic/ModeSequenceTemplate.cpp
	src/logic/SingleLegLogic.cpp
	src/logic/SwitchedModelModeScheduleManager.cpp
	src/terrain/TerrainModelPlanar.cpp
	src/terrain/TerrainPlane.cpp
	src/EndEffectorConstraint.cpp
	src/FootNormalConstraint.cpp
	src/ComKinoConstraintBaseAd.cpp
	src/AnymalWheelsComKinoConstraintAd.cpp
	src/ComKinoOperatingPointsBase.cpp
	src/ComKinoSystemDynamicsAd.cpp
	src/ComModelBase.cpp
	src/KinematicsModelBase.cpp
	src/ModelSettings.cpp
	src/SwitchedModelCostBase.cpp
	src/SwitchedModelStateEstimator.cpp
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

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
	message(STATUS "Run clang tooling")
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
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

#############
## Testing ##
#############

catkin_add_gtest(test_constraintTerm
	test/constraint/testConstraintTerm.cpp
)
target_link_libraries(test_constraintTerm
	${PROJECT_NAME}
	${catkin_LIBRARIES}
)

catkin_add_gtest(test_constraintCollection
	test/constraint/testConstraintCollection.cpp
)
target_link_libraries(test_constraintCollection
	${PROJECT_NAME}
	${catkin_LIBRARIES}
)

catkin_add_gtest(test_zeroForceConstraint
	test/constraint/testZeroForceConstraint.cpp)
target_link_libraries(test_zeroForceConstraint
	${PROJECT_NAME}
	${catkin_LIBRARIES}
)

catkin_add_gtest(test_frictionConeConstraint
	test/constraint/testFrictionConeConstraint.cpp
)
target_link_libraries(test_frictionConeConstraint
	${PROJECT_NAME}
	${catkin_LIBRARIES}
)

catkin_add_gtest(test_${PROJECT_NAME}_logic
	test/logic/testExtractContractTimings.cpp
	test/logic/testGait.cpp
	test/logic/testGaitSwitching.cpp
	test/logic/testGaitSchedule.cpp
	test/logic/testSingleLegLogic.cpp
)
target_link_libraries(test_${PROJECT_NAME}_logic
	${PROJECT_NAME}
	gtest_main
)

catkin_add_gtest(test_${PROJECT_NAME}_terrain
	test/terrain/testTerrainPlane.cpp
	)
target_link_libraries(test_${PROJECT_NAME}_terrain
	${PROJECT_NAME}
	gtest_main
	)