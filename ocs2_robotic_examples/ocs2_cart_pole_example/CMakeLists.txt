cmake_minimum_required(VERSION 3.0)
project(ocs2_cart_pole_example)

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

find_package(raisim CONFIG QUIET)

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

# cartpole interface library
add_library(cartpole_interface
        src/CartPoleInterface.cpp
        )
add_dependencies(cartpole_interface
        ${catkin_EXPORTED_TARGETS}
        )
target_link_libraries(cartpole_interface
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
        dl
        )

# Mpc node
add_executable(cartpole_mpc
        src/MPC_Cartpole_Node.cpp
        )
add_dependencies(cartpole_mpc
        ${catkin_EXPORTED_TARGETS}
        )
target_link_libraries(cartpole_mpc
        cartpole_interface
        ${catkin_LIBRARIES}
        )

# Dummy node
add_executable(cartpole_dummy_test
        src/Dummy_Cartpole_Node.cpp
        )
add_dependencies(cartpole_dummy_test
        ${catkin_EXPORTED_TARGETS}
        )
target_link_libraries(cartpole_dummy_test
        cartpole_interface
        ${catkin_LIBRARIES}
        )

# Dummy node with simulator rollout
add_executable(cartpole_dummy_simulator
        src/Dummy_Cartpole_Simulator_Node.cpp
        )
add_dependencies(cartpole_dummy_simulator
        ${catkin_EXPORTED_TARGETS}
        )
target_link_libraries(cartpole_dummy_simulator
        cartpole_interface
        ${catkin_LIBRARIES}
        )

# Dummy node with raisim rollout
if(raisim_FOUND)
  add_executable(cartpole_dummy_raisim
    src/Dummy_Cartpole_Raisim_Node.cpp
  )
  add_dependencies(cartpole_dummy_raisim
    ${catkin_EXPORTED_TARGETS}
  )
  target_link_libraries(cartpole_dummy_raisim
    cartpole_interface
    ${catkin_LIBRARIES}
    raisim::raisim
  )
endif(raisim_FOUND)

#############
## Install ##
#############

install(TARGETS cartpole_interface
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS cartpole_mpc cartpole_dummy_test
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY urdf launch config rviz
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )
