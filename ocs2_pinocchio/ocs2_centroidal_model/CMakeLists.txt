cmake_minimum_required(VERSION 3.10.2)
project(ocs2_centroidal_model)

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(AMENT_DEPENDENCIES
  ocs2_core
  ocs2_robotic_tools
  ocs2_pinocchio_interface
)

find_package(ament_cmake REQUIRED)
find_package(ocs2_core REQUIRED)
find_package(ocs2_robotic_tools REQUIRED)
find_package(ocs2_pinocchio_interface REQUIRED)
find_package(OpenMP REQUIRED)

find_package(Boost REQUIRED COMPONENTS
  system
  filesystem
  log
  log_setup
)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

if ( EXISTS "/opt/openrobots/lib/libpinocchio.so")
else()
    message(FATAL_ERROR "Install Pinocchio first. Instructions: https://stack-of-tasks.github.io/pinocchio/download.html")
endif()

list(APPEND CMAKE_PREFIX_PATH "/opt/openrobots/")
find_package(pinocchio REQUIRED)

###########
## Build ##
###########

set(FLAGS
  ${OCS2_CXX_FLAGS}
  ${pinocchio_CFLAGS_OTHER}
  -Wno-ignored-attributes
  -Wno-invalid-partial-specialization   # to silence warning with unsupported Eigen Tensor
  -DPINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR
  -DPINOCCHIO_URDFDOM_USE_STD_SHARED_PTR
)

add_library(${PROJECT_NAME}
  src/PinocchioCentroidalDynamics.cpp
  src/PinocchioCentroidalDynamicsAD.cpp
  src/CentroidalModelRbdConversions.cpp
  src/CentroidalModelPinocchioMapping.cpp
  src/CentroidalModelInfo.cpp
  src/FactoryFunctions.cpp
  src/ModelHelperFunctions.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  pinocchio
  ocs2_pinocchio_interface
  ocs2_core
  ocs2_robotic_tools
)

set_target_properties(${PROJECT_NAME}
  PROPERTIES INSTALL_RPATH "/opt/openrobots/lib")

target_compile_options(${PROJECT_NAME} PUBLIC ${FLAGS})

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  message(STATUS "Run clang tooling for target " ${PROJECT_NAME})
  add_clang_tooling(
    TARGETS ${PROJECT_NAME}
    SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include  ${CMAKE_CURRENT_SOURCE_DIR}/test
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
ament_export_dependencies(ocs2_robotic_tools)
ament_export_dependencies(ocs2_thirdparty)
ament_export_dependencies(ocs2_pinocchio_interface)

#############
## Testing ##
#############

#catkin_add_gtest(${PROJECT_NAME}_test
#  # test/testAccessMethods.cpp
#  test/testAnymalCentroidalModel.cpp
#)
#target_include_directories(${PROJECT_NAME}_test PRIVATE
#  test/include
#)
#target_link_libraries(${PROJECT_NAME}_test
#  gtest_main
#  ${PROJECT_NAME}
#  ${Boost_LIBRARIES}
#  ${pinocchio_LIBRARIES}
#)
#target_compile_options(${PROJECT_NAME}_test PRIVATE ${FLAGS})

ament_package()

