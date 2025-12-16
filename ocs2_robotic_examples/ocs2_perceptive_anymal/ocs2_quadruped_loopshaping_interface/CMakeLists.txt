cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_STANDARD 17)
project(ocs2_quadruped_loopshaping_interface)

## ament_cmake Dependencies
set(dependencies
  ocs2_quadruped_interface
  )

find_package(ament_cmake REQUIRED)
find_package(ocs2_quadruped_interface REQUIRED)
## Eigen3
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(PCL REQUIRED)
find_package(Boost REQUIRED COMPONENTS
  system
  filesystem
  log_setup
  log
)




###########
## Build ##
###########

include_directories(
	include
  ${EIGEN3_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
)

# Declare a C++ library
add_library(${PROJECT_NAME}
  src/QuadrupedLoopshapingDummyNode.cpp
  src/QuadrupedLoopshapingInterface.cpp
  src/QuadrupedLoopshapingMpc.cpp
  src/QuadrupedLoopshapingMpcNode.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
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
  INCLUDES DESTINATION include/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

ament_export_dependencies(${dependencies})  
ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()
