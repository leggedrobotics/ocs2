cmake_minimum_required(VERSION 3.0)
project(ocs2_anymal_bear)

find_package(catkin REQUIRED COMPONENTS
  roslib
  ocs2_anymal_bear_switched_model
  ocs2_quadruped_interface
)

## Eigen3
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

find_package(Boost REQUIRED COMPONENTS
  filesystem
)

# Pybind11
option(USE_PYBIND_PYTHON_3 "Use python3-compatible python bindings" OFF)
if(USE_PYBIND_PYTHON_3)
  set(PYTHON_EXECUTABLE /usr/bin/python3)
  set(PYBIND11_PYTHON_VERSION 3.6 CACHE STRING "")
endif(USE_PYBIND_PYTHON_3)
find_package(pybind11 QUIET)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fopenmp -std=c++11 -Wfatal-errors -fPIC")

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS 
    include
    ${EIGEN_INCLUDE_DIRS}
  
  LIBRARIES
    ${PROJECT_NAME}

  CATKIN_DEPENDS
    ocs2_anymal_bear_switched_model
    ocs2_quadruped_interface

  DEPENDS
    Boost
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${EIGEN_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# Declare a C++ library
add_library(${PROJECT_NAME}
        src/AnymalBearInterface.cpp
)
add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
#set_property(TARGET ocs2_anymal_bear_interface PROPERTY POSITION_INDEPENDENT_CODE ON)

add_executable(${PROJECT_NAME}_mpc_node
        src/AnymalBearMpcNode.cpp
)
add_dependencies(${PROJECT_NAME}_mpc_node
  ${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}_mpc_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_dummy_mrt_node
        src/AnymalBearDummyMRT.cpp
)
add_dependencies(${PROJECT_NAME}_dummy_mrt_node
  ${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}_dummy_mrt_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# PyBindings library (c++ side)
add_library(${PROJECT_NAME}_PyBindings_cpp
        src/AnymalBearPyBindings.cpp
)
add_dependencies(${PROJECT_NAME}_PyBindings_cpp
  ${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}_PyBindings_cpp
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# Build python bindings if possible
if(${pybind11_FOUND})
  pybind11_add_module(AnymalBearPyBindings SHARED
    src/pyBindModule.cpp
  )
  add_dependencies(AnymalBearPyBindings
    ${PROJECT_NAME}_PyBindings_cpp
    ${catkin_EXPORTED_TARGETS}
  )
  target_link_libraries(AnymalBearPyBindings PRIVATE
    ${PROJECT_NAME}_PyBindings_cpp
    ${catkin_LIBRARIES}
  )

  if(USE_PYBIND_PYTHON_3)
    # unfortunately this appears not to be discoverable yet because the sourcing process assumes python2.7
    set_target_properties(AnymalBearPyBindings
      PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib/python3.6/dist-packages/${PROJECT_NAME}
    )
  else(USE_PYBIND_PYTHON_3)
    set_target_properties(AnymalBearPyBindings
        PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}
    )
  endif(USE_PYBIND_PYTHON_3)

  catkin_python_setup()
endif()

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

install(TARGETS
            ${PROJECT_NAME}_mpc_node
            ${PROJECT_NAME}_dummy_mrt_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY config launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

#############
## Testing ##
#############
if(${pybind11_FOUND})
  catkin_add_gtest(${PROJECT_NAME}_pybindings_test
    test/PyBindingsTest.cpp
    )
  target_link_libraries(${PROJECT_NAME}_pybindings_test
    ${PROJECT_NAME}_PyBindings_cpp
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    )
endif()
