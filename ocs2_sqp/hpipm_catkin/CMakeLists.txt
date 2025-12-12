cmake_minimum_required(VERSION 3.14)
project(hpipm_catkin)

find_package(ament_cmake REQUIRED)
find_package(blasfeo_catkin REQUIRED)
find_package(ocs2_core REQUIRED)
find_package(ocs2_oc REQUIRED)
find_package(ocs2_qp_solver REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Boost REQUIRED COMPONENTS system filesystem log_setup log)
find_package(OpenMP REQUIRED)
find_package(Threads REQUIRED)

include(FetchContent)

message(STATUS "BLASFEO_PATH: ${BLASFEO_PATH}")

# HPIPM settings
set(BLASFEO_PATH ${BLASFEO_PATH} CACHE STRING "BLASFEO installation path" FORCE)
set(BLASFEO_INCLUDE_DIR ${BLASFEO_PATH}/include CACHE STRING "Path to BLASFEO header files." FORCE)
set(BUILD_SHARED_LIBS ON CACHE BOOL "Build shared libraries" FORCE)
set(HPIPM_TESTING OFF CACHE BOOL "Examples enabled" FORCE)

FetchContent_Declare(hpipmdownload
  GIT_REPOSITORY https://github.com/giaf/hpipm
  GIT_TAG 255ffdf38d3a5e2c3285b29568ce65ae286e5faf
  UPDATE_COMMAND ""
)
FetchContent_MakeAvailable(hpipmdownload)

###########
## Build ##
###########

# Hpipm interface
add_library(${PROJECT_NAME}
  src/HpipmInterface.cpp
  src/HpipmInterfaceSettings.cpp
)
target_link_libraries(${PROJECT_NAME}
  hpipm
  blasfeo_catkin::blasfeo
)
target_link_directories(${PROJECT_NAME}
  PUBLIC
    ${BLASFEO_PATH}/lib
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${hpipmdownload_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(${PROJECT_NAME}
  blasfeo_catkin
  ocs2_core
  ocs2_oc
  ocs2_qp_solver
)
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

#############
## Install ##
#############
install(
  TARGETS ${PROJECT_NAME} hpipm
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  EXPORT export_${PROJECT_NAME}
  DESTINATION share/${PROJECT_NAME}/cmake
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY ${hpipmdownload_SOURCE_DIR}/include/
  DESTINATION include
)

ament_export_include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  blasfeo_catkin
  ocs2_core
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
  test/testHpipmInterface.cpp
)
target_link_libraries(test_${PROJECT_NAME}
  ${PROJECT_NAME}
  hpipm
  gtest_main
)
endif()

ament_package()
