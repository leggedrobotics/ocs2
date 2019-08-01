cmake_minimum_required(VERSION 3.0)
project(ocs2_switched_model_interface)

find_package(catkin REQUIRED
	ocs2_core
	ocs2_mpc
	roscpp
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
		TerrainModel
	CATKIN_DEPENDS 
	    ocs2_core
		ocs2_mpc
	    roscpp
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

# Declare a C++ library
add_library(${PROJECT_NAME}
  src/state_constraint/EndEffectorConstraintBase.cpp
)

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# Declare a C++ library
add_library(TerrainModel
		src/ground/ConvexPlanarPolytope3d.cpp
		src/ground/TerrainModel.cpp
		)

add_dependencies(TerrainModel
		${${PROJECT_NAME}_EXPORTED_TARGETS}
		${catkin_EXPORTED_TARGETS}
		)

target_link_libraries(TerrainModel
		${catkin_LIBRARIES}
		)

add_executable(TerrainVisualizationNode
		src/TerrainVisualizationNode.cpp)
target_link_libraries(TerrainVisualizationNode
		TerrainModel
		${catkin_LIBRARIES}
		)

## Declare a C++ library
#add_executable(testConstraints
#		src/testConstraints.cpp
#		)
#
#target_link_libraries(testConstraints
#		dl
#		)


#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME} TerrainModel
		ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
		)

install(DIRECTORY include/${PROJECT_NAME}/
		DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

#############
## Testing ##
#############

# Build unit tests
catkin_add_gtest(testTerrainModel
		test/ground/testTerrainModel.cpp
		)
target_link_libraries(testTerrainModel
		TerrainModel
		${catkin_LIBRARIES}
		)

###########################
## doxygen documentation ##
###########################
if (CMAKE_BUILD_TYPE MATCHES "^[Rr]elease")

	# first we can indicate the documentation build as an option and set it to ON by default
	option(BUILD_DOC "Build documentation" ON)

	# check if Doxygen is installed
	find_package(Doxygen)
	if (DOXYGEN_FOUND)
	    # set input and output files
	    set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/docs/Doxyfile.in)
	    set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

	    # request to configure the file
	    configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)
	    message("Doxygen build started")

	    # note the option ALL which allows to build the docs together with the application
	    add_custom_target( doc_doxygen ALL
	        COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
	        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
	        COMMENT "Generating API documentation with Doxygen"
	        VERBATIM )
	else (DOXYGEN_FOUND)
	  message("Doxygen need to be installed to generate the doxygen documentation")
	endif (DOXYGEN_FOUND)
endif()
