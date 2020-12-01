cmake_minimum_required(VERSION 3.0.2)
project(ocs2_thirdparty)

find_package(catkin)

catkin_package(
  INCLUDE_DIRS
    include
)

install(DIRECTORY include/
        DESTINATION "${CATKIN_PACKAGE_INCLUDE_DESTINATION}/../")
