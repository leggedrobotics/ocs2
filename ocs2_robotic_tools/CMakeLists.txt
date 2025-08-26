cmake_minimum_required(VERSION 3.14.4)
project(ocs2_robotic_tools LANGUAGES CXX)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unknown-pragmas
    -Wno-unused-function -Wno-unused-parameter)
endif()

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

## Find ament macros and libraries
find_package(ament_cmake REQUIRED)
find_package(ocs2_core REQUIRED)
find_package(ocs2_oc REQUIRED)
find_package(Boost REQUIRED COMPONENTS
  system
  filesystem
  log_setup
  log
)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

###########
## Build ##
###########

set(includes_${PROJECT_NAME}
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

add_library(${PROJECT_NAME} SHARED
  src/common/RotationTransforms.cpp
  src/common/LoopshapingRobotInterface.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  ${includes_${PROJECT_NAME}}
)

# Link ament dependencies
ament_target_dependencies(${PROJECT_NAME} ocs2_core ocs2_oc Eigen3)


add_executable(${PROJECT_NAME}_lintTarget
  src/lintTarget.cpp
)
target_include_directories(${PROJECT_NAME}_lintTarget PUBLIC
    ${includes_${PROJECT_NAME}}
)

ament_target_dependencies(${PROJECT_NAME}_lintTarget
  ocs2_core
  ocs2_oc
  Eigen3
)
#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  message(STATUS "Run clang tooling for target " ${PROJECT_NAME}_lintTarget)
  add_clang_tooling(
    TARGETS ${PROJECT_NAME}_lintTarget
    SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
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
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# Export targets
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(ocs2_core ocs2_oc Eigen3)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Test rotation transforms
  ament_add_gtest(rotation_transform_tests
    test/common/TestRotationTransforms.cpp
    test/common/TestRotationDerivativesTransforms.cpp
  )
  target_link_libraries(rotation_transform_tests
    ${PROJECT_NAME}
  )
  target_compile_options(rotation_transform_tests PRIVATE ${OCS2_CXX_FLAGS})
  ament_target_dependencies(rotation_transform_tests ocs2_core ocs2_oc)
endif()

ament_package()
