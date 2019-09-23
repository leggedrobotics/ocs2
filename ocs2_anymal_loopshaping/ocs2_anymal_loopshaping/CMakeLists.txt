cmake_minimum_required(VERSION 3.0)
project(ocs2_anymal_loopshaping)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roslib
        ocs2_anymal_switched_model
        ocs2_quadruped_augmented_interface
        )

## Eigen3
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

find_package(Boost REQUIRED COMPONENTS
        filesystem
        )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fopenmp -std=c++11 -Wfatal-errors -ffast-math")

###################################
## catkin specific configuration ##
###################################
catkin_package(
INCLUDE_DIRS
    include
    ${EIGEN3_INCLUDE_DIRS}
LIBRARIES
    anymal_loopshaping_asConstraint
    anymal_loopshaping_replaceInput
CATKIN_DEPENDS
    roslib
    ocs2_anymal_switched_model
    ocs2_quadruped_augmented_interface
DEPENDS
    Boost
)

###########
## Build ##
###########

include_directories(
        include
        ${EIGEN3_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
)


##################
## asConstraint ##
##################
# Declare a library
add_library(anymal_loopshaping_asConstraint
        src/asConstraint/definitions.cpp)
add_dependencies(anymal_loopshaping_asConstraint
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
        )
target_link_libraries(anymal_loopshaping_asConstraint
        ${catkin_LIBRARIES}
        glpk
        )

# Declare a C++ Executable
add_executable(anymal_loopshaping_asConstraint_mpc_node
        src/asConstraint/mpc_node.cpp
        )
add_dependencies(anymal_loopshaping_asConstraint_mpc_node
        anymal_loopshaping_asConstraint
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
        )
target_link_libraries(anymal_loopshaping_asConstraint_mpc_node
        anymal_loopshaping_asConstraint
        ${catkin_LIBRARIES}
        glpk
        )

add_executable(anymal_loopshaping_asConstraint_planning
        src/asConstraint/planning.cpp
        )
add_dependencies(anymal_loopshaping_asConstraint_planning
        anymal_loopshaping_asConstraint
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
        )
target_link_libraries(anymal_loopshaping_asConstraint_planning
        anymal_loopshaping_asConstraint
        ${catkin_LIBRARIES}
        )

add_executable(anymal_loopshaping_asConstraint_dummy
        src/asConstraint/dummy.cpp
        )
add_dependencies(anymal_loopshaping_asConstraint_dummy
        anymal_loopshaping_asConstraint
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
        )
target_link_libraries(anymal_loopshaping_asConstraint_dummy
        anymal_loopshaping_asConstraint
        ${catkin_LIBRARIES}
        glpk
        )

##################
## replaceInput ##
##################
add_library(anymal_loopshaping_replaceInput
        src/replaceInput/definitions.cpp)
add_dependencies(anymal_loopshaping_replaceInput
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
        )
target_link_libraries(anymal_loopshaping_replaceInput
        ${catkin_LIBRARIES}
        glpk
        )

# Declare a C++ Executable
add_executable(anymal_loopshaping_replaceInput_mpc_node
        src/replaceInput/mpc_node.cpp
        )
add_dependencies(anymal_loopshaping_replaceInput_mpc_node
        anymal_loopshaping_replaceInput
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
        )
target_link_libraries(anymal_loopshaping_replaceInput_mpc_node
        anymal_loopshaping_replaceInput
        ${catkin_LIBRARIES}
        glpk
        )

add_executable(anymal_loopshaping_replaceInput_planning
        src/replaceInput/planning.cpp
        )
add_dependencies(anymal_loopshaping_replaceInput_planning
        anymal_loopshaping_replaceInput
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
        )
target_link_libraries(anymal_loopshaping_replaceInput_planning
        anymal_loopshaping_replaceInput
        ${catkin_LIBRARIES}
        )

add_executable(anymal_loopshaping_replaceInput_dummy
        src/replaceInput/dummy.cpp
        )
add_dependencies(anymal_loopshaping_replaceInput_dummy
        anymal_loopshaping_replaceInput
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
        )
target_link_libraries(anymal_loopshaping_replaceInput_dummy
        anymal_loopshaping_replaceInput
        ${catkin_LIBRARIES}
        glpk
        )


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ocs2_anymal_interface.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
