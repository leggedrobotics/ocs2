cmake_minimum_required(VERSION 3.0)
project(ocs2_double_integrator_noros_example)

find_package(catkin REQUIRED COMPONENTS
        roslib
        ocs2_core
        ocs2_slq
        ocs2_ocs2
        ocs2_mpc
        ocs2_comm_interfaces
        ocs2_robotic_tools
        )

find_package(Boost REQUIRED COMPONENTS
        system
        filesystem
        )

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# TODO hacky, fix this
set(PYTHON_EXECUTABLE /usr/bin/python3)
set(PYBIND11_PYTHON_VERSION 3.6 CACHE STRING "")
find_package(pybind11 QUIET)
message(WARNING "pybind libraries: ${pybind11_LIBRARIES}")
message(WARNING "PYTHON_MODULE_PREFIX ${PYTHON_MODULE_PREFIX}")
message(WARNING "PYTHON_MODULE_EXTENSION ${PYTHON_MODULE_EXTENSION}")
message(WARNING "PYBIND11_INCLUDE_DIR ${PYBIND11_INCLUDE_DIR}")
message(WARNING "pybind11_INCLUDE_DIR ${pybind11_INCLUDE_DIR}")
message(WARNING "PYTHON_INCLUDE_DIRS ${PYTHON_INCLUDE_DIRS}")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fopenmp -std=c++11 -Wfatal-errors -Wl,--no-as-needed")

###################################
## catkin specific configuration ##
###################################

catkin_package(
        INCLUDE_DIRS
            include
            ${EIGEN3_INCLUDE_DIRS}
        CATKIN_DEPENDS
            ocs2_core
            ocs2_slq
            ocs2_ocs2
            ocs2_mpc
            ocs2_comm_interfaces
            ocs2_robotic_tools
        DEPENDS
            Boost
)

###########
## Build ##
###########

include_directories(
        include
        ${EIGEN3_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
)

# double integrator interface library
add_library(double_integrator_noros_interface
        src/DoubleIntegratorInterface.cpp
        )
add_dependencies(double_integrator_noros_interface
        ${catkin_EXPORTED_TARGETS}
        )
target_link_libraries(double_integrator_noros_interface
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
        dl
        )

add_executable(double_integrator_synchronous_tracking
  src/DoubleIntegratorSynchronousTracking.cpp
        )
add_dependencies(double_integrator_synchronous_tracking
        ${catkin_EXPORTED_TARGETS}
        )
target_link_libraries(double_integrator_synchronous_tracking
        double_integrator_noros_interface
        ${catkin_LIBRARIES}
        )

add_executable(double_integrator_asynchronous_tracking
  src/DoubleIntegratorAsynchronousTracking.cpp
  )
add_dependencies(double_integrator_asynchronous_tracking
  ${catkin_EXPORTED_TARGETS}
  )
target_link_libraries(double_integrator_asynchronous_tracking
  double_integrator_noros_interface
  ${catkin_LIBRARIES}
  )

  # DoubleIntegratorPyBindings library
  add_library(DoubleIntegratorPyBindings
    src/DoubleIntegratorPyBindings.cpp
  )
  add_dependencies(DoubleIntegratorPyBindings
   double_integrator_noros_interface
   ${catkin_EXPORTED_TARGETS}
  )
  target_link_libraries(DoubleIntegratorPyBindings
    double_integrator_noros_interface
    ${catkin_LIBRARIES}
  )

if(${pybind11_FOUND})
  message(WARNING "python executable originally ${PYTHON_EXECUTABLE} and pybind11PythonVersion ${PYBIND11_PYTHON_VERSION}")
  set(PYTHON_EXECUTABLE_BACKUP ${PYTHON_EXECUTABLE})

#  # build and install for python2.XX
#  set(PYTHON_EXECUTABLE /usr/bin/python2)
#  message(WARNING "python2 executable ${PYTHON_EXECUTABLE} and pybind11PythonVersion ${PYBIND11_PYTHON_VERSION}")
#  pybind11_add_module(DoubleIntegratorPyBindings2 SHARED
#    src/DoubleIntegratorPyBindings.cpp
#    src/pyBindModule2.cpp
#  )
#  add_dependencies(DoubleIntegratorPyBindings2
#    ${catkin_EXPORTED_TARGETS}
#  )
#  target_link_libraries(DoubleIntegratorPyBindings2 PRIVATE
#    double_integrator_noros_interface
#    ${catkin_LIBRARIES}
#  )
#  set_target_properties(DoubleIntegratorPyBindings2
#    PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION})

  # build and install for python3.XX
  set(PYTHON_EXECUTABLE /usr/bin/python3)
  set(PYBIND11_PYTHON_VERSION 3.6 CACHE STRING "")
  message(WARNING "python3 executable ${PYTHON_EXECUTABLE} and pybind11PythonVersion ${PYBIND11_PYTHON_VERSION}")
  pybind11_add_module(DoubleIntegratorPyBindings3 SHARED
    src/pyBindModule3.cpp
  )
  add_dependencies(DoubleIntegratorPyBindings3
    DoubleIntegratorPyBindings
    ${catkin_EXPORTED_TARGETS}
  )
  target_link_libraries(DoubleIntegratorPyBindings3 PRIVATE
    DoubleIntegratorPyBindings
    ${catkin_LIBRARIES}
  )
  # TODO this appears not to be discoverable yet
  set_target_properties(DoubleIntegratorPyBindings3
    PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib/python3.6/dist-packages/${PROJECT_NAME})

  # reset python version and finish setup
  set(PYTHON_EXECUTABLE ${PYTHON_EXECUTABLE_BACKUP})
  message(WARNING "python executable restored ${PYTHON_EXECUTABLE} and pybind11PythonVersion ${PYBIND11_PYTHON_VERSION}")
  catkin_python_setup()
endif()

#############
## Install ##
#############

install(TARGETS double_integrator_noros_interface
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS double_integrator_synchronous_tracking double_integrator_asynchronous_tracking
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

if(${pybind11_FOUND})
  # TODO handle python2/3 case properly
  install(TARGETS DoubleIntegratorPyBindings3
          ARCHIVE DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
          LIBRARY DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
          )
endif()

#############
## Testing ##
#############

catkin_add_gtest(ocs2_double_integrator_integration_test
  test/DoubleIntegratorIntegrationTest.cpp
  )
add_dependencies(ocs2_double_integrator_integration_test
  ${catkin_EXPORTED_TARGETS}
  )
target_link_libraries(ocs2_double_integrator_integration_test
  double_integrator_noros_interface
  ${catkin_LIBRARIES}
  )

catkin_add_gtest(ocs2_double_integrator_pybinding_test
  test/DoubleIntegratorPyBindingTest.cpp)
add_dependencies(ocs2_double_integrator_pybinding_test
  DoubleIntegratorPyBindings
  ${catkin_EXPORTED_TARGETS}
  )
target_link_libraries(ocs2_double_integrator_pybinding_test
  DoubleIntegratorPyBindings
  ${catkin_LIBRARIES}
  )
