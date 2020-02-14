cmake_minimum_required(VERSION 3.0)
project(ocs2_anymal_interface)

set(CATKIN_PACKAGE_DEPENDENCIES
  ocs2_anymal_switched_model
  ocs2_quadruped_interface
  ocs2_scp_integration
)

find_package(catkin REQUIRED COMPONENTS
  roslib
  ${CATKIN_PACKAGE_DEPENDENCIES}
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

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fopenmp -std=c++17 -Wfatal-errors -ffast-math -fPIC")

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
    ocs2_anymal_interface

  CATKIN_DEPENDS
  ${CATKIN_PACKAGE_DEPENDENCIES}

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
add_library(ocs2_anymal_interface
  src/OCS2AnymalInterface.cpp
)
add_dependencies(ocs2_anymal_interface
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(ocs2_anymal_interface
  ${catkin_LIBRARIES}
)
#set_property(TARGET ocs2_anymal_interface PROPERTY POSITION_INDEPENDENT_CODE ON)

add_executable(planning
        src/planning.cpp
        )
add_dependencies(planning
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
        )
target_link_libraries(planning
        ocs2_anymal_interface
        ${catkin_LIBRARIES}
        )

add_executable(anymal_mpc_node
  src/AnymalMPC.cpp
)
add_dependencies(anymal_mpc_node
  ocs2_anymal_interface
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(anymal_mpc_node
  ocs2_anymal_interface
  ${catkin_LIBRARIES}
)

add_executable(dummy_mrt_node
  test/DummyMRT.cpp
)
add_dependencies(dummy_mrt_node
  ocs2_anymal_interface
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(dummy_mrt_node
  ocs2_anymal_interface
  ${catkin_LIBRARIES}
)

# PyBindings library (c++ side)
add_library(anymalPyBindings_cpp
  src/AnymalPyBindings.cpp
)
add_dependencies(anymalPyBindings_cpp
  ocs2_anymal_interface
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(anymalPyBindings_cpp
  ocs2_anymal_interface
  ${catkin_LIBRARIES}
)

# Build python bindings if possible
if(${pybind11_FOUND})
  pybind11_add_module(AnymalPyBindings SHARED
    src/pyBindModule.cpp
  )
  add_dependencies(AnymalPyBindings
    anymalPyBindings_cpp
    ${catkin_EXPORTED_TARGETS}
  )
  target_link_libraries(AnymalPyBindings PRIVATE
    anymalPyBindings_cpp
    ${catkin_LIBRARIES}
  )

  if(USE_PYBIND_PYTHON_3)
    # unfortunately this appears not to be discoverable yet because the sourcing process assumes python2.7
    set_target_properties(AnymalPyBindings
      PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib/python3.6/dist-packages/${PROJECT_NAME}
    )
  else(USE_PYBIND_PYTHON_3)
    set_target_properties(AnymalPyBindings
        PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}
    )
  endif(USE_PYBIND_PYTHON_3)

  catkin_python_setup()
endif()

#############
## Install ##
#############

install(TARGETS
            ocs2_anymal_interface
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS
            planning
            anymal_mpc_node
            dummy_mrt_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY config launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

#############
## Testing ##
#############
if(${pybind11_FOUND})
  catkin_add_gtest(anymal_pybindings_test
    test/PyBindingsTest.cpp
    )
  target_link_libraries(anymal_pybindings_test
    anymalPyBindings_cpp
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    )
endif()
