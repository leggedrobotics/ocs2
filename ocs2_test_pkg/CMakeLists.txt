cmake_minimum_required (VERSION 2.8)

project (ocs2_test_pkg)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wfatal-errors -std=c++11 -Wall")

find_package(catkin REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS  REQUIRED)

include_directories(
        ${BOOST_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
)

include_directories (
        include
        include/external
        ${EIGEN3_INCLUDE_DIR}
)

catkin_package(
        INCLUDE_DIRS
        ${BOOST_INCLUDE_DIRS}
        include
        CATKIN_DEPENDS
)

find_package(Doxygen)
if(DOXYGEN_FOUND)

    set(doxyfile_in ${CMAKE_CURRENT_SOURCE_DIR}/doc/ocs2_test_pkg_doxyfile.in)
    set(doxyfile ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile)

    configure_file(${doxyfile_in} ${doxyfile} @ONLY)

    add_custom_target(doc
            COMMAND ${CMAKE_COMMAND} -E echo_append "Building OCS2 Documentation..."
            COMMAND ${DOXYGEN_EXECUTABLE} ${doxyfile}
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/doc
            COMMAND ${CMAKE_COMMAND} -E echo_append "OCS2 Documentation built in ${CMAKE_CURRENT_SOURCE_DIR}/doc"
            VERBATIM)
endif()