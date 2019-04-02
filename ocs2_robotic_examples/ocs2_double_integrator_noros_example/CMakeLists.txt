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

# Mpc node
add_executable(double_integrator_noros_mpc
        src/MPC_Double_Integrator_Node.cpp
        )
add_dependencies(double_integrator_noros_mpc
        ${catkin_EXPORTED_TARGETS}
        )
target_link_libraries(double_integrator_noros_mpc
        double_integrator_noros_interface
        ${catkin_LIBRARIES}
        )

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

install(TARGETS double_integrator_noros_mpc
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY urdf launch config rviz
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )
