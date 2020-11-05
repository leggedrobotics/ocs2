cmake_minimum_required(VERSION 3.0.2)
project(ocs2_core)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  ocs2_thirdparty
)

find_package(Boost REQUIRED COMPONENTS
  system
  filesystem
  log_setup
  log
)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# pthread and OpenMp
set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)
find_package(Threads REQUIRED)
if (Threads_FOUND) # Rename for catkin
  set(Threads_INCLUDE_DIRS ${THREADS_PTHREADS_INCLUDE_DIR})
  set(Threads_LIBRARIES ${CMAKE_THREAD_LIBS_INIT})
endif (Threads_FOUND)
find_package(OpenMP REQUIRED)

# Load ocs2 compile flags
include(cmake/ocs2_cxx_flags.cmake)
message(STATUS "OCS2_CXX_FLAGS: " ${OCS2_CXX_FLAGS})

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
    OpenMP_CXX
    Threads
  CFG_EXTRAS
    ocs2_cxx_flags.cmake
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
  src/Types.cpp
  src/automatic_differentation/CppAdInterface.cpp
  src/automatic_differentation/CppAdSparsity.cpp
  src/automatic_differentation/FiniteDifferenceMethods.cpp
  src/constraint/ConstraintBase.cpp
  src/constraint/ConstraintBaseAD.cpp
  src/constraint/ConstraintCollection.cpp
  src/constraint/LinearConstraint.cpp
  src/constraint/PenaltyBase.cpp
  src/constraint/RelaxedBarrierPenalty.cpp
  src/control/FeedforwardController.cpp
  src/control/LinearController.cpp
  src/control/StateBasedLinearController.cpp
  src/control/TrajectorySpreadingControllerAdjustment.cpp
  src/cost/CostCollection.cpp
  src/cost/CostFunctionBase.cpp
  src/cost/CostFunctionBaseAD.cpp
  src/cost/CostDesiredTrajectories.cpp
  src/cost/RelaxedBarrierCost.cpp
  src/cost/CostFunctionLinearCombination.cpp
  src/cost/QuadraticCostFunction.cpp
  src/cost/QuadraticGaussNewtonCostBaseAD.cpp
  src/dynamics/ControlledSystemBase.cpp
  src/dynamics/LinearSystemDynamics.cpp
  src/dynamics/SystemDynamicsBase.cpp
  src/dynamics/SystemDynamicsBaseAD.cpp
  src/dynamics/SystemDynamicsLinearizer.cpp
  src/dynamics/TransferFunctionBase.cpp
  src/initialization/OperatingPoints.cpp
  src/integration/Integrator.cpp
  src/integration/IntegratorBase.cpp
  src/integration/OdeBase.cpp
  src/integration/Observer.cpp
  src/integration/StateTriggeredEventHandler.cpp
  src/integration/SystemEventHandler.cpp
  src/logic/ModeSchedule.cpp
  src/loopshaping/LoopshapingPropertyTree.cpp
  src/loopshaping/LoopshapingFilter.cpp
  src/loopshaping/LoopshapingOperatingPoint.cpp
  src/loopshaping/cost/LoopshapingCost.cpp
  src/loopshaping/cost/LoopshapingCostEliminatePattern.cpp
  src/loopshaping/cost/LoopshapingCostInputPattern.cpp
  src/loopshaping/cost/LoopshapingCostOutputPattern.cpp
  src/loopshaping/constraint/LoopshapingConstraint.cpp
  src/loopshaping/constraint/LoopshapingConstraintEliminatePattern.cpp
  src/loopshaping/constraint/LoopshapingConstraintInputPattern.cpp
  src/loopshaping/constraint/LoopshapingConstraintOutputPattern.cpp
  src/loopshaping/dynamics/LoopshapingDynamics.cpp
  src/loopshaping/dynamics/LoopshapingDynamicsEliminatePattern.cpp
  src/loopshaping/dynamics/LoopshapingDynamicsInputPattern.cpp
  src/loopshaping/dynamics/LoopshapingDynamicsOutputPattern.cpp
  src/loopshaping/dynamics/LoopshapingFilterDynamics.cpp
  src/model_data/ModelDataBase.cpp
  src/misc/LinearAlgebra.cpp
  src/misc/ThreadPool.cpp
  src/misc/Log.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  OpenMP::OpenMP_CXX
  Threads::Threads
)
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

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
    SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
    CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
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

catkin_add_gtest(test_control
  test/control/testLinearController.cpp
  test/control/testFeedforwardController.cpp
)
target_link_libraries(test_control
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  gtest_main
)

catkin_add_gtest(initialization_unittest
  test/initialization/InitializationTest.cpp
)
target_link_libraries(initialization_unittest
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  gtest_main
)

catkin_add_gtest(test_integration
  test/integration/IntegrationTest.cpp
  test/integration/TrapezoidalIntegrationTest.cpp
)
target_link_libraries(test_integration
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  gtest_main
)

catkin_add_gtest(interpolation_unittest
  test/misc/testInterpolation.cpp
)
target_link_libraries(interpolation_unittest
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  gtest_main
)

catkin_add_gtest(${PROJECT_NAME}_cppadcg
  test/cppad_cg/testCppADCG_constraint.cpp
  test/cppad_cg/testCppADCG_cost.cpp
  test/cppad_cg/testCppADCG_dynamics.cpp
  test/cppad_cg/testSparsityHelpers.cpp
  test/cppad_cg/testCppAdInterface.cpp
)
target_link_libraries(${PROJECT_NAME}_cppadcg
  ${PROJECT_NAME}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  -lm -ldl
  gtest_main
)

catkin_add_gtest(test_transferfunctionbase
  test/dynamics/testTransferfunctionBase.cpp
)
target_link_libraries(test_transferfunctionbase
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  gtest_main
)

catkin_add_gtest(${PROJECT_NAME}_loopshaping
  test/loopshaping/testLoopshapingConfiguration.cpp
  test/loopshaping/testLoopshapingConstraint.cpp
  test/loopshaping/testLoopshapingCost.cpp
  test/loopshaping/testLoopshapingDefinition.cpp
  test/loopshaping/testLoopshapingDynamics.cpp
  test/loopshaping/testLoopshapingFilterDynamics.cpp
)
target_link_libraries(${PROJECT_NAME}_loopshaping
  ${PROJECT_NAME}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  -lstdc++fs
  gtest_main
)

catkin_add_gtest(${PROJECT_NAME}_test_misc
  test/misc/testInterpolation.cpp
  test/misc/testLinearAlgebra.cpp
  test/misc/testSynchronized.cpp
  test/misc/testLookup.cpp
  test/misc/testThreadPool.cpp
)
target_link_libraries(${PROJECT_NAME}_test_misc
  ${PROJECT_NAME}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  gtest_main
)

catkin_add_gtest(test_dynamics
  test/dynamics/testSystemDynamicsLinearizer.cpp
)
target_link_libraries(test_dynamics
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  gtest_main
)

catkin_add_gtest(test_cost
  test/cost/testCostCollection.cpp
  test/cost/testRelaxedBarrierCost.cpp
  test/cost/testCostFunctionLinearCombination.cpp
  test/cost/testQuadraticCostFunction.cpp
)
target_link_libraries(test_cost
  ${PROJECT_NAME}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  gtest_main
)

catkin_add_gtest(test_constraint
  test/constraint/testConstraintCollection.cpp
)
target_link_libraries(test_constraint
  ${PROJECT_NAME}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  gtest_main
)

catkin_add_gtest(test_ModelDataBase
  test/model_data/testModelDataBase.cpp
)
target_link_libraries(test_ModelDataBase
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  gtest_main
)

catkin_add_gtest(test_Logging
  test/misc/testLogging.cpp
)
target_link_libraries(test_Logging
  ${PROJECT_NAME}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  gtest_main
)
