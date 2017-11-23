cmake_minimum_required(VERSION 2.8.3)
project(c_switched_model_interface)

find_package(catkin REQUIRED
	cmake_modules
	ocs2_core
	roscpp
)

## Eigen3
set( EIGEN_INCLUDE_DIRS "$ENV{LOCAL_EIGEN3_INCLUDE_DIR}" )
if( NOT EIGEN_INCLUDE_DIRS )
    find_package(Eigen REQUIRED)
    message( WARNING "Uses Globally installed Eigen since LOCAL_EIGEN3_INCLUDE_DIR is not defined!")
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fopenmp -std=c++11 -Wfatal-errors -ffast-math")

###################################
## catkin specific configuration ##
###################################
catkin_package(
	INCLUDE_DIRS include
	LIBRARIES ${PROJECT_NAME}
	CATKIN_DEPENDS 
	    ocs2_core 
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
