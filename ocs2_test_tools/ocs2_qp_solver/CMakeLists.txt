cmake_minimum_required(VERSION 3.14.4)
project(ocs2_qp_solver LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

###############
## Find Packages
###############

find_package(ament_cmake REQUIRED)
find_package(ocs2_core REQUIRED)
find_package(ocs2_oc REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)


######################
## Library Definition
######################

set(dependencies
  ocs2_core
  ocs2_oc
  Eigen3
)

add_library(${PROJECT_NAME} SHARED
  src/Ocs2QpSolver.cpp
  src/QpDiscreteTranscription.cpp
  src/QpSolver.cpp
  src/QpTrajectories.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
)

######################
## Clang Tooling (optional)
######################
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  message(STATUS "Run clang tooling for target ${PROJECT_NAME}")
  add_clang_tooling(
    TARGETS ${PROJECT_NAME}
    SOURCE_DIRS
      ${CMAKE_CURRENT_SOURCE_DIR}/src
      ${CMAKE_CURRENT_SOURCE_DIR}/include
      ${CMAKE_CURRENT_SOURCE_DIR}/test/include
    CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
    CF_WERROR
  )
endif()

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

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY test/include/
  DESTINATION include/${PROJECT_NAME}
)

#############
## Testing ##
#############

find_package(ament_cmake_gtest REQUIRED)

ament_add_gtest(test_${PROJECT_NAME}
  test/testDiscreteTranscription.cpp
  test/testQpSolver.cpp
  test/testOcs2QpSolver.cpp
)

target_include_directories(test_${PROJECT_NAME} 
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/test/include
)

ament_target_dependencies(test_${PROJECT_NAME}
  ${dependencies}
)

target_link_libraries(test_${PROJECT_NAME}
  ${PROJECT_NAME}
)

########################
## Export Information ##
########################

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})

ament_package()
