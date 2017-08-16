cmake_minimum_required(VERSION 2.8)
project(ocs2)
find_package(catkin REQUIRED)
catkin_metapackage()

add_custom_target(doc
	COMMAND ${CMAKE_COMMAND} -E echo_append "Generating OCS2 Documentation..."
	VERBATIM)
