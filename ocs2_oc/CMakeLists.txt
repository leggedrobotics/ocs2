cmake_minimum_required(VERSION 3.5)
project(ocs2_oc)

find_package(ament_cmake REQUIRED)
find_package(ocs2_core REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Boost REQUIRED COMPONENTS system filesystem log_setup log)
find_package(OpenMP REQUIRED)
find_package(Threads REQUIRED)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

###########
## Build ##
###########

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
  Boost::system
  Boost::filesystem
  OpenMP::OpenMP_CXX
  Threads::Threads
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)
ament_target_dependencies(${PROJECT_NAME}
  ocs2_core
)
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

#############
## Install ##
#############

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)
install(DIRECTORY test/include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  ocs2_core
  Boost
  Eigen3
  OpenMP
  Threads
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

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

ament_add_gtest(test_${PROJECT_NAME}_multiple_shooting
  test/multiple_shooting/testProjectionMultiplierCoefficients.cpp
  test/multiple_shooting/testTranscriptionMetrics.cpp
  test/multiple_shooting/testTranscriptionPerformanceIndex.cpp
)
target_link_libraries(test_${PROJECT_NAME}_multiple_shooting
  ${PROJECT_NAME}
  gtest_main
)

ament_add_gtest(test_${PROJECT_NAME}_data
  test/oc_data/testTimeDiscretization.cpp
)
target_link_libraries(test_${PROJECT_NAME}_data
  ${PROJECT_NAME}
  gtest_main
)

ament_add_gtest(test_${PROJECT_NAME}_rollout
   test/rollout/testTimeTriggeredRollout.cpp
   test/rollout/testStateTriggeredRollout.cpp
)
target_link_libraries(test_${PROJECT_NAME}_rollout
  ${PROJECT_NAME}
  gtest_main
)

ament_add_gtest(test_change_of_variables
  test/testChangeOfInputVariables.cpp
)
target_link_libraries(test_change_of_variables
  ${PROJECT_NAME}
  gtest_main
)

ament_add_gtest(test_trajectory_spreading
  test/trajectory_adjustment/TrajectorySpreadingTest.cpp
)
target_link_libraries(test_trajectory_spreading
  ${PROJECT_NAME}
  gtest_main
)

ament_add_gtest(test_ocp_to_kkt
  test/oc_problem/testOcpToKkt.cpp
)
target_link_libraries(test_ocp_to_kkt
  ${PROJECT_NAME}
  gtest_main
)

ament_add_gtest(test_precondition
  test/precondition/testPrecondition.cpp
)
target_link_libraries(test_precondition
  ${PROJECT_NAME}
  gtest_main
)
endif()

ament_package()
