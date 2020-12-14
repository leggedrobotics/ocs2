cmake_minimum_required(VERSION 3.0.2)
project (ocs2_frank_wolfe)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  ocs2_core
)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake)
find_package(GLPK MODULE REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
   INCLUDE_DIRS
     include
     ${EIGEN3_INCLUDE_DIRS}
   CATKIN_DEPENDS
     ocs2_core
   LIBRARIES
     ${PROJECT_NAME}
   DEPENDS
     GLPK
)

###########
## Build ##
###########

## Include directories
include_directories(
    include
    ${EIGEN3_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/FrankWolfeDescentDirection.cpp
  src/GradientDescent.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############
## Info ==============================
## to run tests, cd package folder and run
## $ catkin build -DCMAKE_BUILD_TYPE=RelWithDebInfo --this
## $ catkin run_tests --no-deps --this
## to see the summary of unit test results run
## $ catkin_test_results ../../../build/ocs2_frank_wolfe

catkin_add_gtest(quadratic_test
  test/QuadraticTest.cpp
)
target_link_libraries(quadratic_test
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  GLPK::GLPK
  gtest_main
)

catkin_add_gtest(matyas_test
  test/MatyasTest.cpp
)
target_link_libraries(matyas_test
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  GLPK::GLPK
  gtest_main
)

catkin_add_gtest(glpk_test
  test/testGLPK.cpp
)
target_link_libraries(glpk_test
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  GLPK::GLPK
  gtest_main
)
