cmake_minimum_required(VERSION 3.0)
project(ocs2_anymal_commands)

find_package(catkin REQUIRED COMPONENTS
  roslib
  ocs2_comm_interfaces
  ocs2_robotic_tools
  ocs2_switched_model_interface
)

## Eigen3
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

find_package(Boost REQUIRED COMPONENTS
  filesystem
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fopenmp -std=c++11 -Wfatal-errors -ffast-math -fPIC")

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

  CATKIN_DEPENDS
    ocs2_comm_interfaces
    ocs2_robotic_tools
    ocs2_switched_model_interface

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

# Declare a C++ Executable
add_executable(target_command_node
  src/AnymalTargetPoseCommand.cpp
)
add_dependencies(target_command_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(target_command_node
  ${catkin_LIBRARIES}
)

add_executable(target_command_node_velocity
  src/AnymalTargetVelocityCommand.cpp
)
add_dependencies(target_command_node_velocity
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(target_command_node_velocity
  ${catkin_LIBRARIES}
)

add_executable(gait_command_node
  src/AnymalModeSequenceCommand.cpp
)
add_dependencies(gait_command_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(gait_command_node
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS
            target_command_node
            target_command_node_velocity
            gait_command_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

#############
## Testing ##
#############
