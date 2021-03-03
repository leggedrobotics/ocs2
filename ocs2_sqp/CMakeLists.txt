cmake_minimum_required(VERSION 3.0.2)
project(ocs2_sqp)

# set(CMAKE_CXX_FLAGS "-std=c++11")
set(CATKIN_PACKAGE_DEPENDENCIES
  ocs2_core
  ocs2_ddp
  ocs2_mpc
  ocs2_oc
  ocs2_qp_solver
)

find_package(catkin REQUIRED COMPONENTS
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
else()
    message(WARNING "Could not find HPIPM or BLASFEO")
endif()

# something that might cause troubles 
# if(CMAKE_C_COMPILER_ID MATCHES MSVC) # no explicit math library

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
add_library(${PROJECT_NAME}
  src/ConstraintProjection.cpp
  src/HpipmInterface.cpp
  src/MultipleShootingSolver.cpp
  src/DynamicsDiscretization.cpp
  )
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_include_directories(${PROJECT_NAME} INTERFACE
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${HPIPM_LIBS})
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

# Circular kinematics example 
add_executable(${PROJECT_NAME}_circular_kinematics src/MultipleShootingCircularKinematics.cpp)
target_include_directories(${PROJECT_NAME}_circular_kinematics INTERFACE
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}_circular_kinematics ${HPIPM_LIBS} ${catkin_LIBRARIES} ${PROJECT_NAME})
target_compile_options(${PROJECT_NAME}_circular_kinematics PRIVATE ${OCS2_CXX_FLAGS})

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

catkin_add_gtest(test_${PROJECT_NAME}
  test/testHpipmInterface.cpp
  test/testProjection.cpp
)
target_link_libraries(test_${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
  gtest_main
)