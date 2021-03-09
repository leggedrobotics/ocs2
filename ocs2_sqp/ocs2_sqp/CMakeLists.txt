cmake_minimum_required(VERSION 3.0.2)
project(ocs2_sqp)

# set(CMAKE_CXX_FLAGS "-std=c++11")
set(CATKIN_PACKAGE_DEPENDENCIES
  ocs2_core
  ocs2_ddp
  ocs2_mpc
  ocs2_oc
  ocs2_qp_solver
  blasfeo_catkin
  hpipm_catkin
)

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_PACKAGE_DEPENDENCIES}
)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

message(STATUS "catkin_INCLUDE_DIRS: " ${catkin_INCLUDE_DIRS})

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS
    include
    ${EIGEN3_INCLUDE_DIRS}
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
  LIBRARIES
    ${PROJECT_NAME}
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# Multiple shooting solver library
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
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

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
  test/testCircularKinematics.cpp
  test/testDiscretization.cpp
  test/testHpipmInterface.cpp
  test/testProjection.cpp
)
target_link_libraries(test_${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
  gtest_main
)