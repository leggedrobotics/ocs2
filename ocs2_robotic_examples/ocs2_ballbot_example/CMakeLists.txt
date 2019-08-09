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

# Ballbot interface library
add_library(ballbot_interface
        src/BallbotInterface.cpp)
add_dependencies(ballbot_interface
        ${catkin_EXPORTED_TARGETS}
        )
target_link_libraries(ballbot_interface
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
    SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
    CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
    CT_HEADER_EXCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/generated
    CF_FIX
  )
endif(cmake_clang_tools_FOUND)

#############
## Install ##
#############

install(TARGETS ballbot_interface
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
