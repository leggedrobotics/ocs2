cmake_minimum_required(VERSION 3.0)
project(ocs2_double_integrator_example)

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

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

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fopenmp -Wfatal-errors -Wl,--no-as-needed")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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
add_library(double_integrator_interface
        src/DoubleIntegratorInterface.cpp
        )
add_dependencies(double_integrator_interface
        ${catkin_EXPORTED_TARGETS}
        )
target_link_libraries(double_integrator_interface
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
        dl
        )

# Mpc node
add_executable(double_integrator_mpc
        src/MPC_Double_Integrator_Node.cpp
        )
add_dependencies(double_integrator_mpc
        ${catkin_EXPORTED_TARGETS}
        )
target_link_libraries(double_integrator_mpc
        double_integrator_interface
        ${catkin_LIBRARIES}
        )

add_executable(double_integrator_pi
  # src/double_integrator/DoubleIntegratorInterface.cpp
  src/PI_Double_Integrator.cpp
)
add_dependencies(double_integrator_pi
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(double_integrator_pi
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  dl
)

# Dummy node
add_executable(double_integrator_dummy_test
        src/Dummy_Double_Integrator_Node.cpp
        )
add_dependencies(double_integrator_dummy_test
        ${catkin_EXPORTED_TARGETS}
        )
target_link_libraries(double_integrator_dummy_test
        double_integrator_interface
        ${catkin_LIBRARIES}
        )

# Target node
add_executable(double_integrator_target
        src/DoubleIntegratorTargetPoseCommand.cpp
        )
add_dependencies(double_integrator_target
        ${catkin_EXPORTED_TARGETS}
        )
target_link_libraries(double_integrator_target
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
        )

# ROS-Independent nodes
add_executable(double_integrator_synchronous_tracking
  src/DoubleIntegratorSynchronousTracking.cpp
)
add_dependencies(double_integrator_synchronous_tracking
        ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(double_integrator_synchronous_tracking
        double_integrator_interface
        ${catkin_LIBRARIES}
)

add_executable(double_integrator_asynchronous_tracking
  src/DoubleIntegratorAsynchronousTracking.cpp
  )
add_dependencies(double_integrator_asynchronous_tracking
  ${catkin_EXPORTED_TARGETS}
  )
target_link_libraries(double_integrator_asynchronous_tracking
  double_integrator_interface
  ${catkin_LIBRARIES}
  )

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  message(STATUS "Run clang tooling for target " double_integrator_mpc)
  add_clang_tooling(
    TARGETS
        double_integrator_interface
        double_integrator_mpc
        double_integrator_pi
        double_integrator_dummy_test
        double_integrator_target
        double_integrator_synchronous_tracking
        double_integrator_asynchronous_tracking
    SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/test
    CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
    CF_WERROR
  )
endif(cmake_clang_tools_FOUND)

#############
## Install ##
#############

install(TARGETS double_integrator_interface
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS double_integrator_mpc
                double_integrator_dummy_test
                double_integrator_target
                double_integrator_synchronous_tracking
                double_integrator_asynchronous_tracking
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY urdf launch config rviz
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

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
  double_integrator_interface
  ${catkin_LIBRARIES}
  )
