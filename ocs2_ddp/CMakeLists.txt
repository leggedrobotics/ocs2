cmake_minimum_required(VERSION 3.14.4)
project(ocs2_ddp LANGUAGES CXX)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter -Wno-sign-compare -Wno-return-type -Wno-unused-variable -Wno-unused-but-set-variable)
endif()

## Find ament macros and libraries
find_package(ament_cmake REQUIRED)
find_package(ocs2_core REQUIRED)
find_package(ocs2_oc REQUIRED)
find_package(ocs2_mpc REQUIRED)
find_package(ocs2_qp_solver REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

###########
## Build ##
###########

add_library(${PROJECT_NAME} SHARED
  src/riccati_equations/ContinuousTimeRiccatiEquations.cpp
  src/riccati_equations/DiscreteTimeRiccatiEquations.cpp
  src/riccati_equations/RiccatiModification.cpp
  src/search_strategy/LevenbergMarquardtStrategy.cpp
  src/search_strategy/LineSearchStrategy.cpp
  src/search_strategy/StrategySettings.cpp
  src/ContinuousTimeLqr.cpp
  src/GaussNewtonDDP.cpp
  src/HessianCorrection.cpp
  src/ILQR.cpp
  src/SLQ.cpp
  src/DDP_Settings.cpp
  src/DDP_HelperFunctions.cpp
)

target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

set(includes_${PROJECT_NAME}
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_include_directories(${PROJECT_NAME} PUBLIC
  ${includes_${PROJECT_NAME}}
)

# Link ament dependencies
ament_target_dependencies(${PROJECT_NAME} ocs2_core ocs2_oc ocs2_mpc ocs2_qp_solver Boost Eigen3)

add_executable(${PROJECT_NAME}_lintTarget
  src/lintTarget.cpp
)

target_include_directories(${PROJECT_NAME}_lintTarget PUBLIC
    ${includes_${PROJECT_NAME}}
)

ament_target_dependencies(${PROJECT_NAME}_lintTarget
  ocs2_core
  ocs2_oc
  ocs2_mpc
  ocs2_qp_solver
  Boost
  Eigen3
)

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  message(STATUS "Run clang tooling for target " ${PROJECT_NAME}_lintTarget)
  add_clang_tooling(
    TARGETS ${PROJECT_NAME}_lintTarget ${PROJECT_NAME}
    SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/test
    CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
    CF_WERROR
)
endif(cmake_clang_tools_FOUND)

#############
## Install ##
#############

# Install library
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers
install(
  DIRECTORY include/ test/include/
  DESTINATION include/${PROJECT_NAME}
)

# Export targets
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(ocs2_core ocs2_oc ocs2_mpc ocs2_qp_solver Boost Eigen3)

#############
## Testing ##
#############

if(BUILD_TESTING)
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

  # Test exp0
  ament_add_gtest(exp0_ddp_test
    test/Exp0Test.cpp
  )
  target_link_libraries(exp0_ddp_test
    ${PROJECT_NAME}
  )
  ament_target_dependencies(exp0_ddp_test ocs2_core ocs2_oc ocs2_mpc ocs2_qp_solver Boost)

  # Test exp1
  ament_add_gtest_wrapper(exp1_ddp_test
    test/Exp1Test.cpp
  )
  target_link_libraries(exp1_ddp_test
    ${PROJECT_NAME}
  )
  ament_target_dependencies(exp1_ddp_test ocs2_core ocs2_oc ocs2_mpc ocs2_qp_solver Boost)

  # Correctness test
  ament_add_gtest_wrapper(correctness_test
    test/CorrectnessTest.cpp
    TIMEOUT 120
  )

  target_link_libraries(correctness_test
    ${PROJECT_NAME}
  )
  ament_target_dependencies(correctness_test ocs2_core ocs2_oc ocs2_mpc ocs2_qp_solver Boost)

  # Riccati ODE test
  ament_add_gtest_wrapper(riccati_ode_test
    test/RiccatiTest.cpp
  )
  target_link_libraries(riccati_ode_test
    ${PROJECT_NAME}
  )
  ament_target_dependencies(riccati_ode_test ocs2_core ocs2_oc ocs2_mpc ocs2_qp_solver)

  # Circular kinematics test
  ament_add_gtest_wrapper(circular_kinematics_ddp_test
    test/CircularKinematicsTest.cpp
    TIMEOUT 120
  )
  target_link_libraries(circular_kinematics_ddp_test
    ${PROJECT_NAME}
  )
  ament_target_dependencies(circular_kinematics_ddp_test ocs2_core ocs2_oc ocs2_mpc ocs2_qp_solver Boost)

  # Hybrid SLQ test
  ament_add_gtest_wrapper(hybrid_slq_test
    test/HybridSlqTest.cpp
    TIMEOUT 120
  )
  target_link_libraries(hybrid_slq_test
    ${PROJECT_NAME}
  )
  ament_target_dependencies(hybrid_slq_test ocs2_core ocs2_oc ocs2_mpc ocs2_qp_solver Boost)

  # Bouncing mass test
  ament_add_gtest_wrapper(bouncing_mass_test
    test/bouncingmass/BouncingMassTest.cpp
    test/bouncingmass/OverallReference.cpp
    test/bouncingmass/Reference.cpp
  )
  target_link_libraries(bouncing_mass_test
    ${PROJECT_NAME}
  )
  ament_target_dependencies(bouncing_mass_test ocs2_core ocs2_oc ocs2_mpc ocs2_qp_solver Boost)

  # Continuous time LQR test
  ament_add_gtest_wrapper(testContinuousTimeLqr
    test/testContinuousTimeLqr.cpp
  )
  target_link_libraries(testContinuousTimeLqr
    ${PROJECT_NAME}
  )
  ament_target_dependencies(testContinuousTimeLqr ocs2_core ocs2_oc ocs2_mpc ocs2_qp_solver Boost)

  # DDP helper function test
  ament_add_gtest_wrapper(testDdpHelperFunction
    test/testDdpHelperFunction.cpp
  )
  target_link_libraries(testDdpHelperFunction
    ${PROJECT_NAME}
  )
  ament_target_dependencies(testDdpHelperFunction ocs2_core ocs2_oc ocs2_mpc ocs2_qp_solver Boost)

  # Reaching task test
  ament_add_gtest_wrapper(testReachingTask
    test/testReachingTask.cpp
  )
  target_link_libraries(testReachingTask
    ${PROJECT_NAME}
  )
  ament_target_dependencies(testReachingTask ocs2_core ocs2_oc ocs2_mpc ocs2_qp_solver Boost)
endif()

ament_package()
