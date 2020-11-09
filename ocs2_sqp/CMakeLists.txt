cmake_minimum_required(VERSION 3.0.2)
project(ocs2_sqp)

set(CMAKE_CXX_FLAGS "-std=c++11")
set(CATKIN_PACKAGE_DEPENDENCIES
  ocs2_core
  ocs2_ddp
  ocs2_mpc
  ocs2_ros_interfaces
  ocs2_robotic_tools
  ocs2_ballbot_example
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  ${CATKIN_PACKAGE_DEPENDENCIES}
)

find_package(Eigen3 REQUIRED)

list(APPEND CMAKE_PREFIX_PATH "/opt")
find_package(blasfeo QUIET)
find_package(hpipm QUIET)
if(blasfeo_FOUND AND hpipm_FOUND)
    message(STATUS "Found HPIPM and BLASFEO")
    set(HPIPM ON)
    list(APPEND HPIPM_LIBS hpipm blasfeo)
    list(APPEND ct_optcon_COMPILE_DEFINITIONS HPIPM)
else()
    message(WARNING "Could not find HPIPM or BLASFEO")
endif()

catkin_package(  
  INCLUDE_DIRS
    include
    ${EIGEN3_INCLUDE_DIRS}
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/ocs2_hpipm.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node src/ocs2_sqp_node.cpp)
target_include_directories(${PROJECT_NAME}_node INTERFACE
  ${blasfeo_INCLUDE_DIRS}
  ${hpipm_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node_new src/ocs2_sqp_node_new.cpp)
target_include_directories(${PROJECT_NAME}_node_new INTERFACE
  ${blasfeo_INCLUDE_DIRS}
  ${hpipm_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

if(CMAKE_C_COMPILER_ID MATCHES MSVC) # no explicit math library
	target_link_libraries(${PROJECT_NAME}_node ${HPIPM_LIBS} ${catkin_LIBRARIES})
	target_link_libraries(${PROJECT_NAME}_node_new ${HPIPM_LIBS} ${catkin_LIBRARIES})
else() # add explicit math library
	target_link_libraries(${PROJECT_NAME}_node ${HPIPM_LIBS} ${catkin_LIBRARIES} m)
	target_link_libraries(${PROJECT_NAME}_node_new ${HPIPM_LIBS} ${catkin_LIBRARIES} m)
endif()
