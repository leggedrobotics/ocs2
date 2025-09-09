cmake_minimum_required(VERSION 3.14.4)
project(ocs2_self_collision_visualization LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter -Wno-unused-variable -Wno-return-type -Wno-sign-compare -Wno-deprecated-declarations -Wno-reorder)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(dependencies
  ocs2_core
  ocs2_robotic_tools
  ocs2_pinocchio_interface
  ocs2_self_collision
  ocs2_ros_interfaces
  urdf

  # visualization deps
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
  pinocchio
  hpp-fcl
)

find_package(ament_cmake REQUIRED)
find_package(ocs2_core REQUIRED)
find_package(ocs2_robotic_tools REQUIRED)
find_package(ocs2_pinocchio_interface REQUIRED)
find_package(ocs2_self_collision REQUIRED)
find_package(ocs2_ros_interfaces REQUIRED)
find_package(urdf REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(pinocchio REQUIRED pinocchio)
pkg_check_modules(hpp-fcl REQUIRED hpp-fcl)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(urdfdom REQUIRED)

set(FLAGS
  ${OCS2_CXX_FLAGS}
  ${pinocchio_CFLAGS_OTHER}
  -DPINOCCHIO_WITH_HPP_FCL
  -Wno-ignored-attributes
  -Wno-invalid-partial-specialization   # to silence warning with unsupported Eigen Tensor
  -DPINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR
  -DPINOCCHIO_URDFDOM_USE_STD_SHARED_PTR
)

link_directories(
  ${pinocchio_LIBRARY_DIRS}
)

# ocs2 pinocchio interface library
add_library(${PROJECT_NAME} SHARED
  src/GeometryInterfaceVisualization.cpp
)

set(includes_${PROJECT_NAME}
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_include_directories(${PROJECT_NAME} PUBLIC
  ${includes_${PROJECT_NAME}}
)

target_compile_options(${PROJECT_NAME} PUBLIC ${FLAGS})

ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
)

find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
    message(STATUS "Run clang tooling")
    add_clang_tooling(
      TARGETS ${PROJECT_NAME}
      SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
      CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
      CF_WERROR
    )
endif (cmake_clang_tools_FOUND)

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

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})

ament_package()
