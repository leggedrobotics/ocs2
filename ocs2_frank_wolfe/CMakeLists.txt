cmake_minimum_required(VERSION 3.5)
project(ocs2_frank_wolfe)

find_package(ament_cmake REQUIRED)
find_package(ocs2_core REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Boost REQUIRED COMPONENTS system filesystem log_setup log)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake)
find_package(GLPK MODULE REQUIRED)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

###########
## Build ##
###########

add_library(${PROJECT_NAME}
  src/FrankWolfeDescentDirection.cpp
  src/GradientDescent.cpp
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(${PROJECT_NAME}
  ocs2_core
)
target_link_libraries(${PROJECT_NAME}
  ${GLPK_LIBRARIES}
)
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION include/${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(
  EXPORT export_${PROJECT_NAME}
  DESTINATION share/${PROJECT_NAME}/cmake
)

ament_export_include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  ocs2_core
  Eigen3
  Boost
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

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

ament_add_gtest(quadratic_test
  test/QuadraticTest.cpp
)
target_link_libraries(quadratic_test
  ${PROJECT_NAME}
  ${GLPK_LIBRARIES}
  gtest_main
)

ament_add_gtest(matyas_test
  test/MatyasTest.cpp
)
target_link_libraries(matyas_test
  ${PROJECT_NAME}
  ${GLPK_LIBRARIES}
  gtest_main
)

ament_add_gtest(glpk_test
  test/testGLPK.cpp
)
target_link_libraries(glpk_test
  ${PROJECT_NAME}
  ${GLPK_LIBRARIES}
  gtest_main
)
endif()

ament_package()
