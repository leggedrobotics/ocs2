cmake_minimum_required(VERSION 3.10.2)
project(ocs2_sqp)

find_package(ament_cmake REQUIRED)
find_package(ocs2_core REQUIRED)
find_package(ocs2_ddp REQUIRED)
find_package(ocs2_mpc REQUIRED)
find_package(ocs2_qp_solver REQUIRED)
find_package(blasfeo_catkin REQUIRED)
find_package(hpipm_catkin REQUIRED)
find_package(OpenMP REQUIRED)

set(AMENT_DEPENDENCIES
  ocs2_core
  ocs2_ddp
  ocs2_mpc
  ocs2_oc
  ocs2_qp_solver
  blasfeo_catkin
  hpipm_catkin
)


find_package(Boost REQUIRED COMPONENTS
  system
  filesystem
  log
  log_setup
)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

###########
## Build ##
###########

# Multiple shooting solver library
add_library(${PROJECT_NAME}
  src/ConstraintProjection.cpp
  src/MultipleShootingInitialization.cpp
  src/MultipleShootingSettings.cpp
  src/MultipleShootingSolver.cpp
  src/MultipleShootingSolverStatus.cpp
  src/MultipleShootingTranscription.cpp
  src/TimeDiscretization.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  ${AMENT_DEPENDENCIES}
)

target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  message(STATUS "Run clang tooling for target " ${PROJECT_NAME})
  add_clang_tooling(
    TARGETS
    ${PROJECT_NAME}
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

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_export_dependencies(ocs2_core)
ament_export_dependencies(ocs2_oc)
ament_export_dependencies(ocs2_mpc)
ament_export_dependencies(ocs2_ddp)
ament_export_dependencies(ocs2_qp_solver)
ament_export_dependencies(blasfeo_catkin)
ament_export_dependencies(hpipm_catkin)

#############
## Testing ##
#############
find_package(ament_lint_auto REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
ament_lint_auto_find_test_dependencies()

ament_add_gtest(test_${PROJECT_NAME}
  test/testCircularKinematics.cpp
  test/testDiscretization.cpp
  test/testProjection.cpp
  test/testSwitchedProblem.cpp
  test/testTranscription.cpp
  test/testUnconstrained.cpp
  test/testValuefunction.cpp
)

target_link_libraries(test_${PROJECT_NAME}
  ${PROJECT_NAME}
  gtest_main
)


ament_package()

