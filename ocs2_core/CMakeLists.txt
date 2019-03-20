cmake_minimum_required(VERSION 3.0)
project(ocs2_core) 

find_package(catkin REQUIRED COMPONENTS 
  cmake_modules
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
    include/external
    ${EIGEN3_INCLUDE_DIRS}
  DEPENDS 
    Boost
)

###########
## Build ##
###########

include_directories(
  include
  include/external
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

#############
## Doxygen ##
#############

find_package(Doxygen)
if(DOXYGEN_FOUND)

    set(doxyfile_in ${CMAKE_CURRENT_SOURCE_DIR}/doc/ocs2_core_doxyfile.in)
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
## $ catkin_test_results ../../../build/ocs2_core

catkin_add_gtest(integration_unittest 
  test/IntegrationTest.cpp
)
target_link_libraries(integration_unittest
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
) 

catkin_add_gtest(findIndex_unittest 
  test/testFindIndex.cpp
)
target_link_libraries(findIndex_unittest
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

catkin_add_gtest(logic_rules_machin_unittest 
  test/testLogicRulesMachine.cpp
)
target_link_libraries(logic_rules_machin_unittest
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

catkin_add_gtest(cppadcg_unittest 
  test/testCppADCG.cpp
)
target_link_libraries(cppadcg_unittest
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES} -lm -ldl
)

catkin_add_gtest(test_transferfunctionbase
  test/testTransferfunctionBase.cpp
)
target_link_libraries(test_transferfunctionbase
  ${catkin_LIBRARIES}
)


