cmake_minimum_required(VERSION 2.8.3)
project(ocs2_slq)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS 
  cmake_modules 
  ocs2_core
  ocs2_oc
)
find_package(Boost REQUIRED COMPONENTS 
  system
  filesystem 
)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fopenmp -std=c++11 -Wfatal-errors -Wl,--no-as-needed")

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS 
    include
    ${EIGEN3_INCLUDE_DIRS}
  CATKIN_DEPENDS 
    ocs2_core
    ocs2_oc
  DEPENDS 
    Boost
)

###########
## Build ##
###########

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

#############
## Doxygen ##
#############

find_package(Doxygen)
if(DOXYGEN_FOUND)

    set(doxyfile_in ${CMAKE_CURRENT_SOURCE_DIR}/doc/ocs2_slq_doxyfile.in)
    set(doxyfile ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile)

    configure_file(${doxyfile_in} ${doxyfile} @ONLY)

    add_custom_target(doc
            COMMAND ${CMAKE_COMMAND} -E echo_append "Building OCS2 Documentation..."
            COMMAND ${DOXYGEN_EXECUTABLE} ${doxyfile}
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/doc
            COMMAND ${CMAKE_COMMAND} -E echo_append "OCS2 Documentation built in ${CMAKE_CURRENT_SOURCE_DIR}/doc"
            VERBATIM)
endif()

#############
## Testing ##
#############
## Info ==============================
## to run tests, cd package folder and run
## $ catkin build -DCMAKE_BUILD_TYPE=RelWithDebInfo --this
## $ catkin run_tests --no-deps --this
## to see the summary of unit test results run
## $ catkin_test_results ../../../build/ocs2_slq

catkin_add_gtest(exp0_slq_test 
  test/exp0_slq_test.cpp
)
target_link_libraries(exp0_slq_test
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

catkin_add_gtest(exp1_slq_test 
  test/exp1_slq_test.cpp
)
target_link_libraries(exp1_slq_test
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

catkin_add_gtest(exp2_slq_test 
  test/exp2_slq_test.cpp
)
target_link_libraries(exp2_slq_test
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

