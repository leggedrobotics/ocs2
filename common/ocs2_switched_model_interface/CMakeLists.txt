cmake_minimum_required(VERSION 3.0)
project(ocs2_switched_model_interface)

## Catkin Dependencies
set(CATKIN_PACKAGE_DEPENDENCIES
  ocs2_core
)
find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_PACKAGE_DEPENDENCIES}
)

## Eigen3
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fopenmp -std=c++11 -Wfatal-errors -ffast-math")

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
		src/ComKinoConstraintBaseAd.cpp
		src/ComKinoSystemDynamicsAd.cpp
		src/SwitchedModelCostBase.cpp
		src/ComKinoOperatingPointsBase.cpp
        src/SwitchedModelStateEstimator.cpp
		src/SwitchedModelLogicRulesBase.cpp
		src/KinematicsModelBase.cpp
		src/ComModelBase.cpp
		src/Model_Settings.cpp
		)

add_dependencies(${PROJECT_NAME}
		${${PROJECT_NAME}_EXPORTED_TARGETS}
		${catkin_EXPORTED_TARGETS}
		)

target_link_libraries(${PROJECT_NAME}
		${catkin_LIBRARIES}
		dl
		)

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

#############
## Testing ##
#############

catkin_add_gtest(test_constraintTerm
        test/constraint/testConstraintTerm.cpp
        )
target_link_libraries(test_constraintTerm
        ${catkin_LIBRARIES}
        )

catkin_add_gtest(test_constraintCollection
        test/constraint/testConstraintCollection.cpp
        )
target_link_libraries(test_constraintCollection
        ${catkin_LIBRARIES}
        )

catkin_add_gtest(test_zeroForceConstraint
		test/constraint/testZeroForceConstraint.cpp
		)

catkin_add_gtest(test_frictionConeConstraint
		test/constraint/testFrictionConeConstraint.cpp
		)
