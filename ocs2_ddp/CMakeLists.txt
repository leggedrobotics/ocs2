cmake_minimum_required(VERSION 3.5)
project(ocs2_ddp)

find_package(ament_cmake REQUIRED)
find_package(ocs2_core REQUIRED)
find_package(ocs2_oc REQUIRED)
find_package(ocs2_mpc REQUIRED)
find_package(ocs2_qp_solver REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Boost REQUIRED COMPONENTS system filesystem log_setup log)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_library(${PROJECT_NAME}
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
target_link_libraries(${PROJECT_NAME}
  ${Boost_LIBRARIES}
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
  ocs2_oc
  ocs2_mpc
  ocs2_qp_solver
)
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION include/${PROJECT_NAME})

ament_export_include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  ocs2_core
  ocs2_oc
  ocs2_mpc
  ocs2_qp_solver
  Boost
  Eigen3
)

#############
## Testing ##
#############
## Info ==============================
## to run tests, cd package folder and run
## $ catkin build -DCMAKE_BUILD_TYPE=RelWithDebInfo --this
## $ catkin run_tests --no-deps --this
## to see the summary of unit test results run
## $ catkin_test_results ../../../build/ocs2_ddp

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

ament_add_gtest(exp0_ddp_test
  test/Exp0Test.cpp
)
target_link_libraries(exp0_ddp_test
  ${PROJECT_NAME}
  ${Boost_LIBRARIES}
  gtest_main
)

ament_add_gtest(exp1_ddp_test
  test/Exp1Test.cpp
)
target_link_libraries(exp1_ddp_test
  ${PROJECT_NAME}
  ${Boost_LIBRARIES}
  gtest_main
)

ament_add_gtest(correctness_test
  test/CorrectnessTest.cpp
)
target_link_libraries(correctness_test
  ${PROJECT_NAME}
  ${Boost_LIBRARIES}
  gtest_main
)

ament_add_gtest(riccati_ode_test
  test/RiccatiTest.cpp
)
target_link_libraries(riccati_ode_test
  ${PROJECT_NAME}
  gtest_main
)

ament_add_gtest(circular_kinematics_ddp_test
  test/CircularKinematicsTest.cpp
)
target_link_libraries(circular_kinematics_ddp_test
  ${PROJECT_NAME}
  ${Boost_LIBRARIES}
  gtest_main
)

ament_add_gtest(hybrid_slq_test
  test/HybridSlqTest.cpp
)
target_link_libraries(hybrid_slq_test
  ${PROJECT_NAME}
  ${Boost_LIBRARIES}
  gtest_main
)

ament_add_gtest(bouncing_mass_test
  test/bouncingmass/BouncingMassTest.cpp
  test/bouncingmass/OverallReference.cpp
  test/bouncingmass/Reference.cpp
)
target_link_libraries(bouncing_mass_test
  ${PROJECT_NAME}
  ${Boost_LIBRARIES}
  gtest_main
)

ament_add_gtest(testContinuousTimeLqr
  test/testContinuousTimeLqr.cpp
)
target_link_libraries(testContinuousTimeLqr
  ${Boost_LIBRARIES}
  ${PROJECT_NAME}
  gtest_main
)

ament_add_gtest(testDdpHelperFunction
  test/testDdpHelperFunction.cpp
)
target_link_libraries(testDdpHelperFunction
  ${Boost_LIBRARIES}
  ${PROJECT_NAME}
  gtest_main
)

ament_add_gtest(testReachingTask
  test/testReachingTask.cpp
)
target_link_libraries(testReachingTask
  ${Boost_LIBRARIES}
  ${PROJECT_NAME}
  gtest_main
)
endif()

ament_package()
