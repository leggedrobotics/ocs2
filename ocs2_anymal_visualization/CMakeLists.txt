cmake_minimum_required(VERSION 2.8.3)
project(ocs2_anymal_visualization)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    xpp_vis
)

## Eigen3
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fopenmp -std=c++11 -Wfatal-errors -ffast-math")

###################################
## catkin specific configuration ##
###################################

catkin_package(
    INCLUDE_DIRS
    CATKIN_DEPENDS
        xpp_vis
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(anymal_visualizer
        src/AnymalUrdfVisualization.cpp)
add_dependencies(anymal_visualizer
        ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(anymal_visualizer
        ${catkin_LIBRARIES}
)

#############
## Install ##
#############

#############
## Testing ##
#############
