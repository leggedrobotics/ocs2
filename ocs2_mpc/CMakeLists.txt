cmake_minimum_required(VERSION 3.5)
project(ocs2_mpc)

find_package(ament_cmake REQUIRED)
find_package(ocs2_core REQUIRED)
find_package(ocs2_oc REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Boost REQUIRED COMPONENTS system filesystem log_setup log)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_library(${PROJECT_NAME}
  src/LoopshapingSystemObservation.cpp
  src/MPC_BASE.cpp
  src/MPC_Settings.cpp
  src/SystemObservation.cpp
  src/MRT_BASE.cpp
  src/MPC_MRT_Interface.cpp
  # src/MPC_OCS2.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${Boost_LIBRARIES}
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
  ocs2_oc
)
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

#############
## Install ##
#############

install(
  TARGETS ${PROJECT_NAME}
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
  Boost
  Eigen3
)

ament_package()

#############
## Testing ##
#############

#catkin_add_gtest(testMPC_OCS2
#  test/testMPC_OCS2.cpp
#)
#target_link_libraries(testMPC_OCS2
#  ${catkin_LIBRARIES}
#  ${Boost_LIBRARIES}
#)
#target_compile_options(testMPC_OCS2 PRIVATE ${OCS2_CXX_FLAGS})
