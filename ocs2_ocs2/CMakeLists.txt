cmake_minimum_required(VERSION 3.5)
project(ocs2_ocs2)

find_package(ament_cmake REQUIRED)
find_package(ocs2_core REQUIRED)
find_package(ocs2_ddp REQUIRED)
find_package(ocs2_frank_wolfe REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Boost REQUIRED COMPONENTS system filesystem log_setup log)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)
ament_export_dependencies(
  ocs2_core
  ocs2_ddp
  ocs2_frank_wolfe
  Eigen3
  Boost
)

ament_package()

#############
## Testing ##
#############
## Info ==============================
## to run tests, cd package folder and run
## $ catkin build -DCMAKE_BUILD_TYPE=RelWithDebInfo --this
## $ catkin run_tests --no-deps --this
## to see the summary of unit test results run
## $ catkin_test_results ../../../build/ocs2_ocs2

#catkin_add_gtest(exp0_gddp_test
#  test/exp0_gddp_test.cpp
#)
#target_link_libraries(exp0_gddp_test
#  ${catkin_LIBRARIES}
#  ${Boost_LIBRARIES}
#)
#target_compile_options(exp0_gddp_test PRIVATE ${OCS2_CXX_FLAGS})

#catkin_add_gtest(exp1_gddp_test
#  test/exp1_gddp_test.cpp
#)
#target_link_libraries(exp1_gddp_test
#  ${catkin_LIBRARIES}
#  ${Boost_LIBRARIES}
#)
#target_compile_options(exp1_gddp_test PRIVATE ${OCS2_CXX_FLAGS})

#catkin_add_gtest(exp0_ocs2_test
#  test/exp0_ocs2_test.cpp
#)
#target_link_libraries(exp0_ocs2_test
#  ${catkin_LIBRARIES}
#  ${Boost_LIBRARIES}
#)
#target_compile_options(exp0_ocs2_test PRIVATE ${OCS2_CXX_FLAGS})

#catkin_add_gtest(exp1_ocs2_test
#  test/exp1_ocs2_test.cpp
#)
#target_link_libraries(exp1_ocs2_test
#  ${catkin_LIBRARIES}
#  ${Boost_LIBRARIES}
#)
#target_compile_options(exp1_ocs2_test PRIVATE ${OCS2_CXX_FLAGS})
