cmake_minimum_required (VERSION 2.8)

project (ocs2_doc)

find_package(catkin REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS 
    ocs2_core 
    ocs2_frank_wolfe 
    ocs2_slq
    ocs2_ilqr 
    ocs2_ocs2
    ocs2_mpc
    ocs2_comm_interfaces
)

#############
## Doxygen ##
#############

find_package(Doxygen)
if(DOXYGEN_FOUND)

    set(doxyfile_in ${CMAKE_CURRENT_SOURCE_DIR}/doc/ocs2_doc_doxyfile.in)
    set(doxyfile ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile)

    configure_file(${doxyfile_in} ${doxyfile} @ONLY)

    add_custom_target(doc
	    COMMAND ${CMAKE_COMMAND} -E echo_append "Generating OCS2 Documentation..."
        COMMAND ${DOXYGEN_EXECUTABLE} ${doxyfile}
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/doc
	COMMAND ${CMAKE_COMMAND} -E echo_append "OCS2 Documentation built in ${CMAKE_CURRENT_SOURCE_DIR}/doc"
        COMMAND sensible-browser ${CMAKE_CURRENT_SOURCE_DIR}/doc/html/index.html
        VERBATIM)
endif()
