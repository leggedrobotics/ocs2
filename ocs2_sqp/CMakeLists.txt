cmake_minimum_required(VERSION 3.0.2)
project(ocs2_sqp)

# set(CMAKE_CXX_FLAGS "-std=c++11")
set(CATKIN_PACKAGE_DEPENDENCIES
  ocs2_core
  ocs2_ddp
  ocs2_mpc
  ocs2_ros_interfaces
  ocs2_robotic_tools
  ocs2_ballbot_example
)

find_package(catkin REQUIRED COMPONENTS
  roslib
  ${CATKIN_PACKAGE_DEPENDENCIES}
)

find_package(Boost REQUIRED COMPONENTS
  system
  filesystem
)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# here it is assumed that BLASFEO and HPIPM are all installed under /opt
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
    # below should be working for every package that depends on ocs2_sqp ${catkin_INCLUDE_DIR} will contains this 
    ${blasfeo_INCLUDE_DIRS}
    ${hpipm_INCLUDE_DIRS}
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
  LIBRARIES
    # maybe should add something like this 
    ${HPIPM_LIBS}
    dl
    m
    
    ${PROJECT_NAME}
  DEPENDS
    Boost
)

###########
## Build ##
###########

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# Multiple shooting solver library
# with equality constraint satisfied by QR decomposition, null space method & HPIPM inequality to equality 
add_library(${PROJECT_NAME} src/MultipleShootingSolver.cpp)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_include_directories(${PROJECT_NAME} INTERFACE
  ${blasfeo_INCLUDE_DIRS}
  ${hpipm_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
if(CMAKE_C_COMPILER_ID MATCHES MSVC) # no explicit math library
  target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${HPIPM_LIBS} dl)
else()
  target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${HPIPM_LIBS} dl m)
endif()
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})


# SQP node for ballbot
add_executable(ballbot_sqp src/MultipleShootingBallbotNode.cpp)
add_dependencies(ballbot_sqp ${catkin_EXPORTED_TARGETS})
target_include_directories(ballbot_sqp INTERFACE
  ${blasfeo_INCLUDE_DIRS}
  ${hpipm_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
if(CMAKE_C_COMPILER_ID MATCHES MSVC) # no explicit math library
  target_link_libraries(ballbot_sqp ${PROJECT_NAME} ${catkin_LIBRARIES} ${HPIPM_LIBS})
else()
  target_link_libraries(ballbot_sqp ${PROJECT_NAME} ${catkin_LIBRARIES} ${HPIPM_LIBS} m)
endif()
target_compile_options(ballbot_sqp PRIVATE ${OCS2_CXX_FLAGS})


# Circular kinematics example 
add_executable(circular_kinematics src/MultipleShootingCircularKinematics.cpp)
target_include_directories(circular_kinematics INTERFACE
  ${blasfeo_INCLUDE_DIRS}
  ${hpipm_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
if(CMAKE_C_COMPILER_ID MATCHES MSVC) # no explicit math library
	target_link_libraries(circular_kinematics ${HPIPM_LIBS} ${catkin_LIBRARIES} ${PROJECT_NAME})
else() # add explicit math library
	target_link_libraries(circular_kinematics ${HPIPM_LIBS} ${catkin_LIBRARIES} ${PROJECT_NAME} m)
endif()
target_compile_options(circular_kinematics PRIVATE ${OCS2_CXX_FLAGS})

#########################
###   CLANG TOOLING   ###
#########################

#############
## Install ##
#############
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

#############
## Testing ##
#############