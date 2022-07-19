cmake_minimum_required(VERSION 3.10.2)
project(ocs2_thirdparty)

find_package(ament_cmake REQUIRED)


install(DIRECTORY include/
        DESTINATION include
)
ament_export_include_directories(include)

ament_package()
