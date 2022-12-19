cmake_minimum_required(VERSION 3.0)
project(ocs2_switched_model_interface)

## Catkin Dependencies
set(CATKIN_PACKAGE_DEPENDENCIES
  roscpp
  ocs2_core
  ocs2_msgs
  ocs2_oc
  ocs2_ros_interfaces
  ocs2_robotic_tools
  ocs2_switched_model_msgs
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
		${EIGEN3_INCLUDE_DIRS}
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
	${EIGEN3_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
	src/analytical_inverse_kinematics/AnalyticalInverseKinematics.cpp
	src/analytical_inverse_kinematics/LegInverseKinematicParameters.cpp
	src/constraint/EndEffectorVelocityConstraint.cpp
	src/constraint/FootNormalConstraint.cpp
	src/constraint/FrictionConeConstraint.cpp
	src/constraint/ZeroForceConstraint.cpp
	src/dynamics/ComKinoSystemDynamicsAd.cpp
	src/dynamics/ComKinoDynamicsParameters.cpp
	src/cost/CollisionAvoidanceCost.cpp
	src/cost/FootPlacementCost.cpp
	src/cost/FrictionConeCost.cpp
	src/cost/LinearStateInequalitySoftConstraint.cpp
	src/cost/MotionTrackingCost.cpp
	src/cost/MotionTrackingTerminalCost.cpp
	src/cost/TorqueLimitsSoftConstraint.cpp
	src/foot_planner/KinematicFootPlacementPenalty.cpp
	src/foot_planner/CubicSpline.cpp
	src/foot_planner/FootPhase.cpp
	src/foot_planner/QuinticSplineSwing.cpp
	src/foot_planner/SplineCpg.cpp
	src/foot_planner/SwingSpline3d.cpp
	src/foot_planner/SwingTrajectoryPlanner.cpp
	src/initialization/ComKinoInitializer.cpp
	src/logic/Gait.cpp
	src/logic/GaitAdaptation.cpp
	src/logic/GaitReceiver.cpp
	src/logic/GaitSchedule.cpp
	src/logic/ModeSequenceTemplate.cpp
	src/logic/SingleLegLogic.cpp
	src/logic/DynamicsParametersSynchronizedModule.cpp
	src/logic/SwitchedModelModeScheduleManager.cpp
	src/ros_msg_conversions/RosMsgConversions.cpp
	src/terrain/PlanarSignedDistanceField.cpp
	src/terrain/PlanarTerrainModel.cpp
	src/terrain/PlaneFitting.cpp
	src/terrain/TerrainPlane.cpp
	src/core/ComModelBase.cpp
	src/core/KinematicsModelBase.cpp
	src/core/ModelSettings.cpp
	src/core/SwitchedModelPrecomputation.cpp
	src/core/TorqueApproximation.cpp
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

catkin_add_gtest(test_constraints
	test/constraint/testZeroForceConstraint.cpp
	test/constraint/testFrictionConeConstraint.cpp
)
target_link_libraries(test_constraints
	${PROJECT_NAME}
	${catkin_LIBRARIES}
	gtest_main
)

catkin_add_gtest(test_${PROJECT_NAME}_core
	test/core/testRotation.cpp
)
target_link_libraries(test_${PROJECT_NAME}_core
	${PROJECT_NAME}
	gtest_main
)

catkin_add_gtest(test_${PROJECT_NAME}_cost
	test/cost/testFootplacementCost.cpp
	test/cost/testFrictionConeCost.cpp
	test/cost/testTorqueLimitsSoftConstraint.cpp
)
target_link_libraries(test_${PROJECT_NAME}_cost
	${PROJECT_NAME}
	gtest_main
)

catkin_add_gtest(test_${PROJECT_NAME}_footplanner
	test/foot_planner/testSwingPhase.cpp
)
target_link_libraries(test_${PROJECT_NAME}_footplanner
	${PROJECT_NAME}
	gtest_main
)

catkin_add_gtest(test_${PROJECT_NAME}_logic
	test/logic/testEarlyTouchDown.cpp
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

catkin_add_gtest(test_${PROJECT_NAME}_convexTerrain
	test/terrain/testConvexTerrain.cpp
)
target_link_libraries(test_${PROJECT_NAME}_convexTerrain
	${PROJECT_NAME}
	gtest_main
)
