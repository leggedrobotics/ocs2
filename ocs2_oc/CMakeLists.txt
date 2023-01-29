cmake_minimum_required(VERSION 3.0.2)
project(ocs2_oc)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  ocs2_core
  # ocs2_qp_solver
)
find_package(Boost REQUIRED COMPONENTS
  system
  filesystem
)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS
    include
    test/include
    ${EIGEN3_INCLUDE_DIRS}
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ocs2_core
  DEPENDS
    Boost
)

###########
## Build ##
###########

include_directories(
  include
  test/include
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/approximate_model/ChangeOfInputVariables.cpp
  src/approximate_model/LinearQuadraticApproximator.cpp
  src/multiple_shooting/Helpers.cpp
  src/multiple_shooting/Initialization.cpp
  src/multiple_shooting/LagrangianEvaluation.cpp
  src/multiple_shooting/MetricsComputation.cpp
  src/multiple_shooting/PerformanceIndexComputation.cpp
  src/multiple_shooting/ProjectionMultiplierCoefficients.cpp
  src/multiple_shooting/Transcription.cpp
  src/oc_data/LoopshapingPrimalSolution.cpp
  src/oc_data/PerformanceIndex.cpp
  src/oc_data/TimeDiscretization.cpp
  src/oc_problem/OptimalControlProblem.cpp
  src/oc_problem/LoopshapingOptimalControlProblem.cpp
  src/oc_problem/OptimalControlProblemHelperFunction.cpp
  src/oc_problem/OcpSize.cpp
  src/oc_problem/OcpToKkt.cpp
  src/oc_solver/SolverBase.cpp
  src/precondition/Ruzi.cpp
  src/rollout/PerformanceIndicesRollout.cpp
  src/rollout/RolloutBase.cpp
  src/rollout/RootFinder.cpp
  src/rollout/InitializerRollout.cpp
  src/rollout/StateTriggeredRollout.cpp
  src/rollout/TimeTriggeredRollout.cpp
  src/rollout/RolloutSettings.cpp
  src/synchronized_module/ReferenceManager.cpp
  src/synchronized_module/LoopshapingReferenceManager.cpp
  src/synchronized_module/LoopshapingSynchronizedModule.cpp
  src/synchronized_module/SolverObserver.cpp
  src/search_strategy/FilterLinesearch.cpp
  src/trajectory_adjustment/TrajectorySpreading.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

add_executable(${PROJECT_NAME}_lintTarget
  src/lintTarget.cpp
)

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  message(STATUS "Run clang tooling for target " ${PROJECT_NAME}_lintTarget)
  add_clang_tooling(
    TARGETS ${PROJECT_NAME}_lintTarget
    SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/test/include
    CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
    CF_WERROR
)
endif(cmake_clang_tools_FOUND)

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
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY test/include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

#############
## Testing ##
#############
## Info ==============================
## to run tests, cd package folder and run
## $ catkin build -DCMAKE_BUILD_TYPE=RelWithDebInfo --this
## $ catkin run_tests --no-deps --this
## to see the summary of unit test results run
## $ catkin_test_results ../../../build/ocs2_oc

catkin_add_gtest(test_${PROJECT_NAME}_multiple_shooting
  test/multiple_shooting/testProjectionMultiplierCoefficients.cpp
  test/multiple_shooting/testTranscriptionMetrics.cpp
  test/multiple_shooting/testTranscriptionPerformanceIndex.cpp
)
add_dependencies(test_${PROJECT_NAME}_multiple_shooting
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(test_${PROJECT_NAME}_multiple_shooting
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  gtest_main
)

catkin_add_gtest(test_${PROJECT_NAME}_data
  test/oc_data/testTimeDiscretization.cpp
)
add_dependencies(test_${PROJECT_NAME}_data
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(test_${PROJECT_NAME}_data
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  gtest_main
)

catkin_add_gtest(test_${PROJECT_NAME}_rollout
   test/rollout/testTimeTriggeredRollout.cpp
   test/rollout/testStateTriggeredRollout.cpp
)
add_dependencies(test_${PROJECT_NAME}_rollout
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(test_${PROJECT_NAME}_rollout
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  gtest_main
)

catkin_add_gtest(test_change_of_variables
  test/testChangeOfInputVariables.cpp
)
add_dependencies(test_change_of_variables
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(test_change_of_variables
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  gtest_main
)

catkin_add_gtest(test_trajectory_spreading
  test/trajectory_adjustment/TrajectorySpreadingTest.cpp
)
add_dependencies(test_trajectory_spreading
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(test_trajectory_spreading
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  gtest_main
)

catkin_add_gtest(test_ocp_to_kkt
  test/oc_problem/testOcpToKkt.cpp
)
target_link_libraries(test_ocp_to_kkt
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  gtest_main
)

catkin_add_gtest(test_precondition
  test/precondition/testPrecondition.cpp
)
target_link_libraries(test_precondition
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  gtest_main
)
