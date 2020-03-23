cmake_minimum_required(VERSION 3.0)
project(ocs2_anymal_wheels_switched_model)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    ocs2_switched_model_interface
    ocs2_robotic_tools
)

## Eigen3
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fopenmp -std=c++11 -Wfatal-errors")

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    INCLUDE_DIRS
        include
        ${EIGEN_INCLUDE_DIRS}
    LIBRARIES
        ${PROJECT_NAME}
    CATKIN_DEPENDS
        ocs2_switched_model_interface
        ocs2_robotic_tools
#   DEPENDS
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${EIGEN_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/AnymalWheelsCom.cpp
  src/AnymalWheelsKinematics.cpp
)

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
   dl
)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(DIRECTORY
            include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

#############
## Testing ##
#############

# Build unit tests
catkin_add_gtest(${PROJECT_NAME}_switched_model_test
        test/TestAnymalWheelsSwitchedModel.cpp
        )

add_dependencies(${PROJECT_NAME}_switched_model_test ${catkin_EXPORTED_TARGETS})

target_include_directories(${PROJECT_NAME}_switched_model_test
  PRIVATE
  ${catkin_INCLUDE_DIRS}
  )

target_link_libraries(${PROJECT_NAME}_switched_model_test
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        )

#catkin_add_gtest(anymal_ad_switched_model_test
#        test/TestAnymalAdSwitchedModel.cpp
#        )
#
#add_dependencies(anymal_ad_switched_model_test ${catkin_EXPORTED_TARGETS})
#
#target_link_libraries(anymal_ad_switched_model_test
#        ${PROJECT_NAME}
#        ${catkin_LIBRARIES}
#        dl
#        )
