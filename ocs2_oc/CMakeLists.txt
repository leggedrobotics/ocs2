cmake_minimum_required(VERSION 3.14.4)
project(ocs2_oc LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wno-extra -Wpedantic)
  add_compile_options(-Wno-sign-compare -Wno-range-loop-construct -Wno-ignored-qualifiers -Wno-unused-function)
  add_compile_options(-Wno-unused-parameter -Wno-unused-variable -Wno-unused-but-set-variable)
  add_compile_options(-Wno-implicit-fallthrough -Wno-reorder -Wno-redundant-move -Wno-missing-field-initializers)
endif()

## Find catkin macros and libraries
find_package(ament_cmake REQUIRED)
find_package(ocs2_thirdparty REQUIRED)
find_package(ocs2_core REQUIRED)
find_package(OpenMP REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

###########
## Build ##
###########

set(dependencies
    Boost
    Eigen3
    ocs2_core
)

add_library(${PROJECT_NAME} SHARED
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
set(includes_${PROJECT_NAME}
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_include_directories(${PROJECT_NAME} PUBLIC
  ${includes_${PROJECT_NAME}}
)
ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
)
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

add_executable(${PROJECT_NAME}_lintTarget
  src/lintTarget.cpp
)
target_include_directories(${PROJECT_NAME}_lintTarget PUBLIC
  ${includes_${PROJECT_NAME}}
)
ament_target_dependencies(${PROJECT_NAME}_lintTarget
  ${dependencies}
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
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(DIRECTORY test/include/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})

#############
## Testing ##
#############
## Info ==============================
## to run tests, cd package folder and run
## $ catkin build -DCMAKE_BUILD_TYPE=RelWithDebInfo --this
## $ catkin run_tests --no-deps --this
## to see the summary of unit test results run
## $ catkin_test_results ../../../build/ocs2_oc

find_package(gtest_vendor REQUIRED)
find_package(ament_cmake_gtest REQUIRED)

set(test_includes_${PROJECT_NAME}
  ${includes_${PROJECT_NAME}}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test/include>
)

function(ament_add_gtest_wrapper test_name)
  ament_add_gtest(${test_name} ${ARGN})
  target_include_directories(${test_name} PUBLIC
    ${test_includes_${PROJECT_NAME}}
  )
endfunction()

ament_add_gtest_wrapper(test_${PROJECT_NAME}_multiple_shooting
  test/multiple_shooting/testProjectionMultiplierCoefficients.cpp
  test/multiple_shooting/testTranscriptionMetrics.cpp
  test/multiple_shooting/testTranscriptionPerformanceIndex.cpp
)
ament_target_dependencies(test_${PROJECT_NAME}_multiple_shooting
  ${dependencies}
)
target_link_libraries(test_${PROJECT_NAME}_multiple_shooting
  ${PROJECT_NAME}
  -ldl
)

ament_add_gtest_wrapper(test_${PROJECT_NAME}_data
  test/oc_data/testTimeDiscretization.cpp
)
target_link_libraries(test_${PROJECT_NAME}_data
  ${PROJECT_NAME}
)

ament_add_gtest_wrapper(test_${PROJECT_NAME}_rollout
   test/rollout/testTimeTriggeredRollout.cpp
   test/rollout/testStateTriggeredRollout.cpp
)
target_link_libraries(test_${PROJECT_NAME}_rollout
  ${PROJECT_NAME} ${ocs2_core_TARGETS}
)

ament_add_gtest_wrapper(test_change_of_variables
  test/testChangeOfInputVariables.cpp
)
target_link_libraries(test_change_of_variables
  ${PROJECT_NAME}
)

ament_add_gtest_wrapper(test_trajectory_spreading
  test/trajectory_adjustment/TrajectorySpreadingTest.cpp
)
target_link_libraries(test_trajectory_spreading
  ${PROJECT_NAME}
)

ament_add_gtest_wrapper(test_ocp_to_kkt
  test/oc_problem/testOcpToKkt.cpp
)
target_link_libraries(test_ocp_to_kkt
  ${PROJECT_NAME}
)

ament_add_gtest_wrapper(test_precondition
  test/precondition/testPrecondition.cpp
)
target_link_libraries(test_precondition
  ${PROJECT_NAME}
)

ament_package()
