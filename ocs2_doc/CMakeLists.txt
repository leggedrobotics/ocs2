cmake_minimum_required(VERSION 3.10.2)

# TODO: dervive the version from a proper source. E.g., the most recent git tag
# with Git describe
project (ocs2_doc VERSION 1.0.0)

# Modules to document
list(APPEND OCS2_MODULES
  ocs2_core
  ocs2_frank_wolfe
  ocs2_oc
  ocs2_mpc
  ocs2_ddp
  ocs2_sqp
  ocs2_python_interface
  ocs2_ros_interfaces
  ocs2_robotic_tools
  ocs2_robotic_examples
  ocs2_pinocchio
)

# Set up CMake environment
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})

find_package(Doxygen REQUIRED)
find_package(Sphinx REQUIRED)

# C++ Doxygen XML documentation generation 

set(DOXYFILE_IN ${CMAKE_CURRENT_SOURCE_DIR}/tools/doxygen/Doxyfile.in)
set(DOXYFILE ${CMAKE_CURRENT_BINARY_DIR}/tools/doxygen/Doxyfile)

set(DOXYGEN_PROJECT_NAME ${PROJECT})
set(DOXYGEN_PROJECT_NUMBER ${CMAKE_PROJECT_VERSION})
set(DOXYGEN_OUTPUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/output/doxygen)
set(DOXYGEN_INDEX_FILE ${DOXYGEN_OUTPUT_DIR}/xml/index.xml)

# Convert list to space-separated string for feeding to doxygen
foreach(MODULE ${OCS2_MODULES})
  set(MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../${MODULE})
  if (NOT EXISTS ${MODULE_PATH})
    message(WARNING "Could not find module ${MODULE}")
  endif()
  list(APPEND DOXYGEN_INPUT_LIST ${MODULE_PATH})
  unset(MODULE_PATH)
endforeach()
set(DOXYGEN_INPUT "${DOXYGEN_INPUT_LIST}")
string(REPLACE ";" " " DOXYGEN_INPUT "${DOXYGEN_INPUT}")

message(STATUS "Generating ${DOXYFILE}")
configure_file(${DOXYFILE_IN} ${DOXYFILE} IMMEDIATE)

file(MAKE_DIRECTORY ${DOXYGEN_OUTPUT_DIR})
add_custom_command(OUTPUT ${DOXYGEN_INDEX_FILE}
  BYPRODUCTS ${DOXYGEN_OUTPUT_DIR}
  COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYFILE}
  MAIN_DEPENDENCY ${DOXYFILE} ${DOXYFILE_IN}
  COMMENT "Generating C++ API documentation XML"
  )

add_custom_target(Doxygen ALL DEPENDS ${DOXYGEN_INDEX_FILE})

# Sphinx documentation generation
set(SPHINX_SOURCE ${CMAKE_CURRENT_SOURCE_DIR}/docs)
file(GLOB_RECURSE SPHINX_RST_SOURCES ${SPHINX_SOURCE} *.rst *.bib *.md)
set(SPHINX_CONF ${CMAKE_CURRENT_SOURCE_DIR}/tools/sphinx/)
set(SPHINX_BUILD ${CMAKE_CURRENT_BINARY_DIR}/output/sphinx)
set(SPHINX_INDEX_FILE ${SPHINX_BUILD}/index.html)

file(MAKE_DIRECTORY ${SPHINX_BUILD})
add_custom_command(OUTPUT ${SPHINX_INDEX_FILE}
  COMMAND 
    ${SPHINX_EXECUTABLE} -b html -c ${SPHINX_CONF}
    -Dbreathe_projects.ocs2=${DOXYGEN_OUTPUT_DIR}
    -Drelease="${CMAKE_PROJECT_VERSION}"
    ${SPHINX_SOURCE} ${SPHINX_BUILD} 
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
  BYPRODUCT ${SPHINX_BUILD}
  DEPENDS
  ${SPHINX_RST_SOURCES}
  ${DOXYGEN_INDEX_FILE}
  COMMENT "Generating Sphinx HTML documentation"
  )

add_custom_target(Sphinx ALL DEPENDS ${SPHINX_INDEX_FILE})

# TODO Add ROS target?
