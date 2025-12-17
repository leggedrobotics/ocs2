cmake_minimum_required(VERSION 3.14.4)
project(ocs2_thirdparty)

find_package(ament_cmake REQUIRED)

ament_export_include_directories(include)

install(DIRECTORY include/
        DESTINATION include)

ament_package()
