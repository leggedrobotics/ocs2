cmake_minimum_required(VERSION 3.5)
project(ocs2_slp)

find_package(ament_cmake REQUIRED)
find_package(ocs2_core REQUIRED)
find_package(ocs2_mpc REQUIRED)
find_package(ocs2_oc REQUIRED)
find_package(ocs2_qp_solver REQUIRED)
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
  src/pipg/PipgSettings.cpp
  src/pipg/PipgSolver.cpp
  src/pipg/SingleThreadPipg.cpp
  src/Helpers.cpp
  src/SlpSettings.cpp
  src/SlpSolver.cpp
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)
ament_target_dependencies(${PROJECT_NAME}
  ocs2_core
  ocs2_mpc
  ocs2_oc
  ocs2_qp_solver
)
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

#############
## Install ##
#############
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION include/${PROJECT_NAME})

install(
  EXPORT export_${PROJECT_NAME}
  DESTINATION share/${PROJECT_NAME}/cmake
)

ament_export_include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  ocs2_core
  ocs2_mpc
  ocs2_oc
  ocs2_qp_solver
  Eigen3
  Boost
  OpenMP
  Threads
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

ament_add_gtest(test_${PROJECT_NAME}
  test/testHelpers.cpp
  test/testPipgSolver.cpp
  test/testSlpSolver.cpp
)
target_link_libraries(test_${PROJECT_NAME}
  ${PROJECT_NAME}
  gtest_main
)
endif()

ament_package()
