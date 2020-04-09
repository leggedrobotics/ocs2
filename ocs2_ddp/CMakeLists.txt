cmake_minimum_required(VERSION 2.8.3)
project(ocs2_ddp)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  ocs2_core
  ocs2_oc
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
    ${EIGEN3_INCLUDE_DIRS}
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ocs2_core
    ocs2_oc
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
  src/StrategySettings.cpp
  src/HessianCorrection.cpp
  src/ContinuousTimeRiccatiEquations.cpp
  src/DiscreteTimeRiccatiEquations.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
target_compile_options(${PROJECT_NAME}_dynamic_size PUBLIC ${OCS2_CXX_FLAGS})

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
## $ catkin_test_results ../../../build/ocs2_ddp

catkin_add_gtest(exp0_slq_test
  test/exp0_slq_test.cpp
)
target_link_libraries(exp0_slq_test
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

catkin_add_gtest(exp1_slq_test
  test/exp1_slq_test.cpp
)
target_link_libraries(exp1_slq_test
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

catkin_add_gtest(riccati_ode_test
  test/riccati_ode_test.cpp
)
target_link_libraries(riccati_ode_test
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

catkin_add_gtest(hybrid_slq_test
  test/hybrid_slq_test.cpp
)
target_link_libraries(hybrid_slq_test
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

catkin_add_gtest(exp0_ilqr_test
  test/exp0_ilqr_test.cpp
)
target_link_libraries(exp0_ilqr_test
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

catkin_add_gtest(exp1_ilqr_test
  test/exp1_ilqr_test.cpp
)
target_link_libraries(exp1_ilqr_test
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

catkin_add_gtest(bouncing_mass_test
  test/bouncingmass/BouncingMass.cpp
  test/bouncingmass/OverallReference.cpp
  test/bouncingmass/Reference.cpp
)
target_link_libraries(bouncing_mass_test
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

catkin_add_gtest(circular_kinematics_ilqr_test
  test/circular_kinematics_ilqr_test.cpp
)
target_link_libraries(circular_kinematics_ilqr_test
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

catkin_add_gtest(circular_kinematics_slq_test
  test/circular_kinematics_slq_test.cpp
)
target_link_libraries(circular_kinematics_slq_test
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)
