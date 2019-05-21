cmake_minimum_required(VERSION 3.0)
project(ocs2_double_slit_example)

find_package(catkin REQUIRED COMPONENTS
        roslib
        ocs2_core
        ocs2_slq
        ocs2_ocs2
        ocs2_mpc
        ocs2_comm_interfaces
        ocs2_robotic_tools
        )

find_package(Boost REQUIRED COMPONENTS
        system
        filesystem
        )

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fopenmp -Wfatal-errors -Wl,--no-as-needed")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

###################################
## catkin specific configuration ##
###################################

catkin_package(
        INCLUDE_DIRS
            include
            ${EIGEN3_INCLUDE_DIRS}
        CATKIN_DEPENDS
            ocs2_core
            ocs2_slq
            ocs2_ocs2
            ocs2_mpc
            ocs2_comm_interfaces
            ocs2_robotic_tools
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

add_library(double_slit_interface
        src/DoubleSlitInterface.cpp
        )
add_dependencies(double_slit_interface
        ${catkin_EXPORTED_TARGETS}
        )
target_link_libraries(double_slit_interface
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
        dl
        )

add_executable(double_slit_pi
  src/PI_Double_Slit.cpp
)
add_dependencies(double_slit_pi
  double_slit_interface
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(double_slit_pi
  double_slit_interface
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  dl
)

# Dummy node
add_executable(double_slit_dummy_test
        src/Dummy_Double_Slit_Node.cpp
        )
add_dependencies(double_slit_dummy_test
        ${catkin_EXPORTED_TARGETS}
        )
target_link_libraries(double_slit_dummy_test
        double_slit_interface
        ${catkin_LIBRARIES}
        )


#############
## Install ##
#############

install(TARGETS double_slit_interface
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS double_slit_pi double_slit_dummy_test
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY urdf launch config rviz
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )
