cmake_minimum_required(VERSION 3.0)
project(ocs2_core) 

find_package(catkin REQUIRED COMPONENTS 
  cmake_modules
) 
find_package(Boost REQUIRED COMPONENTS 
  system
  filesystem 
)
# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fopenmp -Wfatal-errors -Wl,--no-as-needed")
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS 
    include
    include/external
    ${EIGEN3_INCLUDE_DIRS}
  LIBRARIES
    ocs2_core_logic

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

# Declare a C++ library
add_library(ocs2_core_logic
        src/logic/machine/HybridLogicRulesMachine.cpp
        )
target_link_libraries(ocs2_core_logic
        ${catkin_LIBRARIES}
        )

add_executable(${PROJECT_NAME}_lintTarget
  src/lintTarget.cpp
)

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  message(STATUS "Run clang tooling for target " ${PROJECT_NAME}_lintTarget)
  add_clang_tooling(
    TARGETS ${PROJECT_NAME}_lintTarget ocs2_core_logic
    SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include/ocs2_core
    CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include/ocs2_core
    CF_FIX
    CF_WERROR
  )
endif(cmake_clang_tools_FOUND)

#############
## Install ##
#############

install(TARGETS
          ocs2_core_logic
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )


install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY include/external/
        DESTINATION "${CATKIN_PACKAGE_INCLUDE_DESTINATION}/../")

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
  test/integration/IntegrationTest.cpp
)
target_link_libraries(integration_unittest
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

catkin_add_gtest(findIndex_unittest
  test/misc/testFindIndex.cpp
)
target_link_libraries(findIndex_unittest
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

catkin_add_gtest(interpolation_unittest
  test/misc/testInterpolation.cpp
)
target_link_libraries(interpolation_unittest
  ${catkin_LIBRARIES}
)

catkin_add_gtest(logic_rules_machin_unittest
  test/logic/testLogicRulesMachine.cpp
)
target_link_libraries(logic_rules_machin_unittest
  ocs2_core_logic
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

catkin_add_gtest(cppadcg_unittest
  test/cppad_cg/testCppADCG.cpp
)
target_link_libraries(cppadcg_unittest
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES} -lm -ldl
)

catkin_add_gtest(test_transferfunctionbase
  test/dynamics/testTransferfunctionBase.cpp
)
target_link_libraries(test_transferfunctionbase
  ${catkin_LIBRARIES}
)

catkin_add_gtest(test_loopshapingDefinition
  test/loopshaping/testLoopshapingDefinition.cpp
)
target_link_libraries(test_loopshapingDefinition
   ${Boost_LIBRARIES}
   ${catkin_LIBRARIES}
   -lstdc++fs
)

catkin_add_gtest(test_loopshapingDynamics
  test/loopshaping/testLoopshapingDynamics.cpp
)
target_link_libraries(test_loopshapingDynamics
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  -lstdc++fs
)

catkin_add_gtest(test_loopshapingCost
  test/loopshaping/testLoopshapingCost.cpp
)
target_link_libraries(test_loopshapingCost
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  -lstdc++fs
)

catkin_add_gtest(test_loopshapingConstraint
  test/loopshaping/testLoopshapingConstraint.cpp
)
target_link_libraries(test_loopshapingConstraint
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  -lstdc++fs
)

catkin_add_gtest(test_loopshapingFilterDynamics
  test/loopshaping/testLoopshapingFilterDynamics.cpp
)
target_link_libraries(test_loopshapingFilterDynamics
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  -lstdc++fs
)

catkin_add_gtest(test_LinearAlgebra
  test/misc/testLinearAlgebra.cpp
)
target_link_libraries(test_LinearAlgebra
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)
