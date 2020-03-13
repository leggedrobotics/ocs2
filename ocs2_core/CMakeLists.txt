cmake_minimum_required(VERSION 3.0)
project(ocs2_core)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  ocs2_thirdparty
)
find_package(Boost REQUIRED COMPONENTS
  system
  filesystem
)
# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fopenmp -Wfatal-errors -Wl,--no-as-needed")
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS
    include
    ${EIGEN3_INCLUDE_DIRS}
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ocs2_thirdparty
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

# Declare a C++ library
add_library(${PROJECT_NAME}
  src/automatic_differentation/CppAdInterface.cpp
  src/automatic_differentation/CppAdSparsity.cpp
  src/integration/Integrator.cpp
  src/logic/ModeSchedule.cpp
  src/loopshaping/LoopshapingPropertyTree.cpp
  src/loopshaping/LoopshapingFilter.cpp
  src/misc/LinearAlgebra.cpp
  src/misc/ThreadPool.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_lintTarget
  src/lintTarget.cpp
)
target_link_libraries(${PROJECT_NAME}_lintTarget
  ${Boost_LIBRARIES}
)

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  message(STATUS "Run clang tooling for target " ${PROJECT_NAME}_lintTarget)
  add_clang_tooling(
    TARGETS
      ${PROJECT_NAME}
      ${PROJECT_NAME}_lintTarget
    SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include/ocs2_core
    CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include/ocs2_core
    CF_WERROR
  )
endif(cmake_clang_tools_FOUND)

#############
## Install ##
#############

install(
  TARGETS
      ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

#############
## Testing ##
#############
## Info ==============================
## to run tests, cd package folder and run
## $ catkin build -DCMAKE_BUILD_TYPE=RelWithDebInfo --this
## $ catkin run_tests --no-deps --this
## to see the summary of unit test results run
## $ catkin_test_results ../../../build/ocs2_core

catkin_add_gtest(integration_unittest
  test/integration/IntegrationTest.cpp
)
target_link_libraries(integration_unittest
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  gtest_main
)

catkin_add_gtest(interpolation_unittest
  test/misc/testInterpolation.cpp
)
target_link_libraries(interpolation_unittest
  ${catkin_LIBRARIES}
)

catkin_add_gtest(cppadcg_constraint_unittest
  test/cppad_cg/testCppADCG_constraint.cpp
)
target_link_libraries(cppadcg_constraint_unittest
  ${PROJECT_NAME}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES} -lm -ldl
)

catkin_add_gtest(cppadcg_cost_unittest
  test/cppad_cg/testCppADCG_cost.cpp
)
target_link_libraries(cppadcg_cost_unittest
  ${PROJECT_NAME}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES} -lm -ldl
)

catkin_add_gtest(cppadcg_dynamics_unittest
  test/cppad_cg/testCppADCG_dynamics.cpp
)
target_link_libraries(cppadcg_dynamics_unittest
  ${PROJECT_NAME}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES} -lm -ldl
)

catkin_add_gtest(testSparsityHelpers
  test/cppad_cg/testSparsityHelpers.cpp
)
target_link_libraries(testSparsityHelpers
  ${PROJECT_NAME}
  ${Boost_LIBRARIES}
  dl
)

catkin_add_gtest(testCppAdInterface
  test/cppad_cg/testCppAdInterface.cpp
)
target_link_libraries(testCppAdInterface
  ${PROJECT_NAME}
  ${Boost_LIBRARIES}
  dl
)

catkin_add_gtest(test_transferfunctionbase
  test/dynamics/testTransferfunctionBase.cpp
)
target_link_libraries(test_transferfunctionbase
  ${catkin_LIBRARIES}
)

catkin_add_gtest(test_loopshapingDefinition
  test/loopshaping/testLoopshapingDefinition.cpp
)
target_link_libraries(test_loopshapingDefinition
   ${PROJECT_NAME}
   ${Boost_LIBRARIES}
   ${catkin_LIBRARIES}
   -lstdc++fs
)

catkin_add_gtest(test_loopshapingDynamics
  test/loopshaping/testLoopshapingDynamics.cpp
)
target_link_libraries(test_loopshapingDynamics
  ${PROJECT_NAME}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  -lstdc++fs
)

catkin_add_gtest(test_loopshapingCost
  test/loopshaping/testLoopshapingCost.cpp
)
target_link_libraries(test_loopshapingCost
  ${PROJECT_NAME}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  -lstdc++fs
)

catkin_add_gtest(test_loopshapingConstraint
  test/loopshaping/testLoopshapingConstraint.cpp
)
target_link_libraries(test_loopshapingConstraint
  ${PROJECT_NAME}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  -lstdc++fs
)

catkin_add_gtest(test_loopshapingFilterDynamics
  test/loopshaping/testLoopshapingFilterDynamics.cpp
)
target_link_libraries(test_loopshapingFilterDynamics
  ${PROJECT_NAME}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  -lstdc++fs
)

catkin_add_gtest(test_LinearAlgebra
  test/misc/testLinearAlgebra.cpp
)
target_link_libraries(test_LinearAlgebra
  ${PROJECT_NAME}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)

catkin_add_gtest(test_lookup
  test/misc/testLookup.cpp
)
target_link_libraries(test_lookup
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)

catkin_add_gtest(derivative_checker_unittest
  test/dynamics/testSystemDynamicsLinearizer.cpp
)
target_link_libraries(derivative_checker_unittest
  ${catkin_LIBRARIES}
)

catkin_add_gtest(test_ThreadPool
  test/misc/testThreadPool.cpp
)
target_link_libraries(test_ThreadPool
  ${PROJECT_NAME}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  gtest_main
)

catkin_add_gtest(test_CostFunctionLinearCombination
  test/cost/testCostFunctionLinearCombination.cpp
)
target_link_libraries(test_CostFunctionLinearCombination
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  gtest_main
)