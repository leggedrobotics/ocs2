cmake_minimum_required(VERSION 3.0)
project(ocs2_ballbot_example)

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED COMPONENTS
        roslib
        tf
        visualization_msgs
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

# Pybind11
option(USE_PYBIND_PYTHON_3 "Use python3-compatible python bindings" ON)
if(USE_PYBIND_PYTHON_3)
  set(PYTHON_EXECUTABLE /usr/bin/python3)
  set(PYBIND11_PYTHON_VERSION 3.6 CACHE STRING "")
endif(USE_PYBIND_PYTHON_3)
find_package(pybind11 QUIET)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fopenmp -std=c++11 -Wfatal-errors -Wl,--no-as-needed")

###################################
## catkin specific configuration ##
###################################

catkin_package(
        INCLUDE_DIRS
            include
            ${EIGEN3_INCLUDE_DIRS}
        CATKIN_DEPENDS
            tf
            visualization_msgs
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

add_library(ballbot_PI
        src/solvers/BallbotPI.cpp)
add_dependencies(ballbot_PI
        ${catkin_EXPORTED_TARGETS}
        )
target_link_libraries(ballbot_PI
        ${catkin_LIBRARIES}
        )

add_library(ballbot_SLQ
        src/solvers/BallbotSLQ.cpp)
add_dependencies(ballbot_SLQ
        ${catkin_EXPORTED_TARGETS}
        )
target_link_libraries(ballbot_SLQ
        ${catkin_LIBRARIES}
        )

add_library(ballbot_dynamics
        src/dynamics/BallbotSystemDynamics.cpp)

# Ballbot interface library
add_library(ballbot_interface
        src/BallbotInterface.cpp)
add_dependencies(ballbot_interface
        ${catkin_EXPORTED_TARGETS}
        )
target_link_libraries(ballbot_interface
        ballbot_SLQ
        ballbot_PI
        ballbot_dynamics
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
        dl
        )

# Mpc node
add_executable(ballbot_mpc
        src/MPC_Ballbot_Node.cpp
        )
add_dependencies(ballbot_mpc
        ${catkin_EXPORTED_TARGETS}
        )
target_link_libraries(ballbot_mpc
        ballbot_interface
        ${catkin_LIBRARIES}
        )

# Path integral mpc
add_executable(ballbot_pi_mpc
  src/PI_Ballbot.cpp)
add_dependencies(ballbot_pi_mpc
        ${catkin_EXPORTED_TARGETS}
        )
target_link_libraries(ballbot_pi_mpc
        ballbot_interface
        ${catkin_LIBRARIES}
        )

# Dummy node
add_executable(ballbot_dummy_test
        src/Dummy_Ballbot_Node.cpp
        )
add_dependencies(ballbot_dummy_test
        ${catkin_EXPORTED_TARGETS}
        )
target_link_libraries(ballbot_dummy_test
        ballbot_interface
        ${catkin_LIBRARIES}
        )

# Target node
add_executable(ballbot_target
        src/BallbotTargetPoseCommand.cpp
        )
add_dependencies(ballbot_target
        ${catkin_EXPORTED_TARGETS}
        )
target_link_libraries(ballbot_target
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
        )

# Pybindings Cpp library
add_library(ballbotPyBindings_cpp
  src/BallbotPyBindings.cpp
)
add_dependencies(ballbotPyBindings_cpp
  ballbot_interface
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(ballbotPyBindings_cpp
  ballbot_interface
  ${catkin_LIBRARIES}
)

# Build python bindings if possible
if(${pybind11_FOUND})
  pybind11_add_module(BallbotPyBindings SHARED
    src/pyBindModule.cpp
  )
  add_dependencies(BallbotPyBindings
    ballbotPyBindings_cpp
    ${catkin_EXPORTED_TARGETS}
  )
  target_link_libraries(BallbotPyBindings PRIVATE
    ballbotPyBindings_cpp
    ${catkin_LIBRARIES}
  )

  if(USE_PYBIND_PYTHON_3)
    # unfortunately this appears not to be discoverable yet because the sourcing process assumes python2.7
    set_target_properties(BallbotPyBindings
      PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib/python3.6/dist-packages/${PROJECT_NAME}
    )
  else(USE_PYBIND_PYTHON_3)
    set_target_properties(BallbotPyBindings
        PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}
    )
  endif(USE_PYBIND_PYTHON_3)

  catkin_python_setup()
endif()


#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  message(STATUS "Run clang tooling for target ballbot_mpc")
  add_clang_tooling(
    TARGETS
        ballbot_interface
        ballbot_mpc
        ballbot_pi_mpc
        ballbot_dummy_test
        ballbot_target
        ballbotPyBindings_cpp
    SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
    CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
    CT_HEADER_EXCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/generated
    CF_WERROR
  )
endif(cmake_clang_tools_FOUND)

#############
## Install ##
#############

install(TARGETS ballbot_interface ballbot_PI ballbot_SLQ ballbot_dynamics ballbotPyBindings_cpp
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ballbot_mpc ballbot_dummy_test ballbot_target
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY urdf launch config rviz
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

#############
## Testing ##
#############

catkin_add_gtest(ballbot_integration_test
        test/ballbotIntegrationTest.cpp
        )
target_link_libraries(ballbot_integration_test
        ballbot_interface
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
        )


catkin_add_gtest(ballbot_pybindings_test
  test/PyBindingsTest.cpp
  )
target_link_libraries(ballbot_pybindings_test
  ballbotPyBindings_cpp
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  )

