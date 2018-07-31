cmake_minimum_required (VERSION 2.8)

project (ocs2_frank_wolfe)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
)

## Eign3
set( EIGEN_INCLUDE_DIRS "$ENV{LOCAL_EIGEN3_INCLUDE_DIR}" )
if( NOT EIGEN_INCLUDE_DIRS )
    find_package(Eigen REQUIRED)
    message( WARNING "Uses Globally installed Eigen since LOCAL_EIGEN3_INCLUDE_DIR is not defined!")
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fopenmp -std=c++11 -Wfatal-errors -Wl,--no-as-needed")


###################################
## catkin specific configuration ##
###################################
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ocs2_frank_wolfe_lib
#  CATKIN_DEPENDS cmake_modules
   DEPENDS eigen
)

###########
## Build ##
###########

## Include directories
include_directories(
    include
    ${EIGEN_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(ocs2_frank_wolfe_lib
  src/ocs2_frank_wolfe/GradientDescent.cpp
)

## Declare a C++ executable
add_executable(quadratic_test test/QuadraticTest.cpp)
add_executable(matyas_test test/MatyasTest.cpp)
add_executable(glpk_test src/glpk_test/testGLPK.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(ocs2_frank_wolfe_lib
    ${catkin_LIBRARIES}
    glpk
)

target_link_libraries(quadratic_test
    ocs2_frank_wolfe_lib
    ${catkin_LIBRARIES}
)

target_link_libraries(matyas_test
    ocs2_frank_wolfe_lib
    ${catkin_LIBRARIES}
)

target_link_libraries(glpk_test
    ${catkin_LIBRARIES}
    glpk
)

find_package(Doxygen)
if(DOXYGEN_FOUND)

    set(doxyfile_in ${CMAKE_CURRENT_SOURCE_DIR}/doc/ocs2_frank_wolfe_doxyfile.in)
    set(doxyfile ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile)

    configure_file(${doxyfile_in} ${doxyfile} @ONLY)

    add_custom_target(doc
            COMMAND ${CMAKE_COMMAND} -E echo_append "Building OCS2 Documentation..."
            COMMAND ${DOXYGEN_EXECUTABLE} ${doxyfile}
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/doc
            COMMAND ${CMAKE_COMMAND} -E echo_append "OCS2 Documentation built in ${CMAKE_CURRENT_SOURCE_DIR}/doc"
            VERBATIM)
endif()
