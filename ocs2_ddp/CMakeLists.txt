cmake_minimum_required(VERSION 3.10.2)
project(ocs2_ddp)

find_package(ament_cmake)
find_package(ocs2_core REQUIRED)
find_package(ocs2_oc REQUIRED)
find_package(ocs2_mpc REQUIRED)
find_package(ocs2_qp_solver REQUIRED)
find_package(OpenMP REQUIRED)

find_package(Boost REQUIRED COMPONENTS
  system
  filesystem
  log
  log_setup
)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

set(AMENT_DEPENDENCIES
  ocs2_core
  ocs2_oc
  ocs2_qp_solver ) 

###########
## Build ##
###########

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

ament_target_dependencies(${PROJECT_NAME}
  ${AMENT_DEPENDENCIES}
)

target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)


add_executable(${PROJECT_NAME}_lintTarget
  src/lintTarget.cpp
)

ament_target_dependencies(${PROJECT_NAME}_lintTarget
  ${AMENT_DEPENDENCIES}
)

target_include_directories(${PROJECT_NAME}_lintTarget PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  message(STATUS "Run clang tooling for target " ${PROJECT_NAME}_lintTarget)
  add_clang_tooling(
    TARGETS ${PROJECT_NAME}_lintTarget ${PROJECT_NAME}
    SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
    CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
    CF_WERROR
)
endif(cmake_clang_tools_FOUND)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(DIRECTORY include/
        DESTINATION include
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

#catkin_add_gtest(exp0_ddp_test
#  test/Exp0Test.cpp
#)
#target_link_libraries(exp0_ddp_test
#  ${PROJECT_NAME}
#  ${catkin_LIBRARIES}
#  ${Boost_LIBRARIES}
#  gtest_main
#)

#catkin_add_gtest(exp1_ddp_test
#  test/Exp1Test.cpp
#)
#target_link_libraries(exp1_ddp_test
#  ${PROJECT_NAME}
#  ${catkin_LIBRARIES}
#  ${Boost_LIBRARIES}
#  gtest_main
#)

#catkin_add_gtest(correctness_test
#  test/CorrectnessTest.cpp
#)
#target_link_libraries(correctness_test
#  ${PROJECT_NAME}
#  ${catkin_LIBRARIES}
#  ${Boost_LIBRARIES}
#  gtest_main
#)

#catkin_add_gtest(riccati_ode_test
#  test/RiccatiTest.cpp
#)
#target_link_libraries(riccati_ode_test
#  ${PROJECT_NAME}
#  ${catkin_LIBRARIES}
#  gtest_main
#)

#catkin_add_gtest(circular_kinematics_ddp_test
#  test/CircularKinematicsTest.cpp
#)
#target_link_libraries(circular_kinematics_ddp_test
#  ${PROJECT_NAME}
#  ${catkin_LIBRARIES}
#  ${Boost_LIBRARIES}
#  gtest_main
#)

#catkin_add_gtest(hybrid_slq_test
#  test/HybridSlqTest.cpp
#)
#target_link_libraries(hybrid_slq_test
#  ${PROJECT_NAME}
#  ${catkin_LIBRARIES}
#  ${Boost_LIBRARIES}
#  gtest_main
#)

#catkin_add_gtest(bouncing_mass_test
#  test/bouncingmass/BouncingMassTest.cpp
#  test/bouncingmass/OverallReference.cpp
#  test/bouncingmass/Reference.cpp
#)
#target_link_libraries(bouncing_mass_test
#  ${PROJECT_NAME}
#  ${catkin_LIBRARIES}
#  ${Boost_LIBRARIES}
#  gtest_main
#)

#catkin_add_gtest(testContinuousTimeLqr
#  test/testContinuousTimeLqr.cpp
#)
#target_link_libraries(testContinuousTimeLqr
#  ${Boost_LIBRARIES}
#  ${catkin_LIBRARIES}
#  ${PROJECT_NAME}
#  gtest_main
#)


ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(
  export_${PROJECT_NAME}
)
ament_export_dependencies(ocs2_oc)
ament_export_dependencies(ocs2_qp_solver)
ament_export_dependencies(ocs2_core)

ament_package()
