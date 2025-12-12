cmake_minimum_required(VERSION 3.5)
project(ocs2_thirdparty)

find_package(ament_cmake REQUIRED)

ament_export_include_directories(include)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
