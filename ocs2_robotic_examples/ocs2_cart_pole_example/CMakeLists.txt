cmake_minimum_required(VERSION 3.0)
project(ocs2_cart_pole_example)

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED COMPONENTS
   roslib
   ocs2_core
   ocs2_ddp
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
  LIBRARIES
    cartpole_interface
  CATKIN_DEPENDS
    ocs2_core
    ocs2_ddp
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
  src/CartpoleDummyVisualization.cpp
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


#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  message(STATUS "Running clang tooling.")
  add_clang_tooling(
    TARGETS
        cartpole_interface
        cartpole_mpc
        cartpole_dummy_test
    SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
    CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
    CT_HEADER_EXCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/generated
    CF_WERROR
    CT_WERROR
  )
endif(cmake_clang_tools_FOUND)

#############
## Install ##
#############

install(TARGETS cartpole_interface
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(TARGETS cartpole_mpc cartpole_dummy_test
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY urdf launch config rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
