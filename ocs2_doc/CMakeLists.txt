cmake_minimum_required (VERSION 3.0)
project (ocs2_doc)

find_package(Doxygen)

IF(DOXYGEN_FOUND)

  list(APPEND ocs2_components "ocs2_core")
  list(APPEND ocs2_components "ocs2_frank_wolfe")
  list(APPEND ocs2_components "ocs2_slq")
  list(APPEND ocs2_components "ocs2_ocs2")
  list(APPEND ocs2_components "ocs2_mpc")
  list(APPEND ocs2_components "ocs2_comm_interfaces")
  list(APPEND ocs2_components "ocs2_ballbot_example")
  list(APPEND ocs2_components "ocs2_cart_pole_example")
  list(APPEND ocs2_components "ocs2_double_integrator_example")
  list(APPEND ocs2_components "ocs2_quadrotor_example")
  list(APPEND ocs2_components "ocs2_robotic_tools")

  find_package(catkin REQUIRED COMPONENTS
    ${ocs2_components}
  )

  catkin_package(
    INCLUDE_DIRS
    LIBRARIES
    CATKIN_DEPENDS
      ${ocs2_components}
    DEPENDS doxygen
  )

  # find ocs2_core
  find_path(SOURCE_OCS2_CORE_DIR NAMES ocs2_core/Dimensions.h PATHS ${ocs2_core_INCLUDE_DIRS}... NO_DEFAULT_PATH)
  if(EXISTS "${SOURCE_OCS2_CORE_DIR}")
    message(STATUS "Found ocs2_core at: ${SOURCE_OCS2_CORE_DIR}")
  else()
    message(FATAL_ERROR "Couldn't find ocs2_core")
  endif()

  # find ocs2_frank_wolfe
  find_path(SOURCE_OCS2_FRANK_WOLFE_DIR NAMES ocs2_frank_wolfe/GradientDescent.h PATHS ${ocs2_frank_wolfe_INCLUDE_DIRS}... NO_DEFAULT_PATH)
  if(EXISTS "${SOURCE_OCS2_FRANK_WOLFE_DIR}")
    message(STATUS "Found ocs2_frank_wolfe at: ${SOURCE_OCS2_FRANK_WOLFE_DIR}")
  else()
    message(FATAL_ERROR "Couldn't find ocs2_frank_wolfe")
  endif()

  # find ocs2_slq
  find_path(SOURCE_OCS2_SLQ_DIR NAMES ocs2_slq/SLQ.h PATHS ${ocs2_slq_INCLUDE_DIRS}... NO_DEFAULT_PATH)
  if(EXISTS "${SOURCE_OCS2_SLQ_DIR}")
    message(STATUS "Found ocs2_slq at: ${SOURCE_OCS2_SLQ_DIR}")
  else()
    message(FATAL_ERROR "Couldn't find ocs2_slq")
  endif()

  # find ocs2_ocs2
  find_path(SOURCE_OCS2_OCS2_DIR NAMES ocs2_ocs2/OCS2Projected.h PATHS ${ocs2_ocs2_INCLUDE_DIRS}... NO_DEFAULT_PATH)
  if(EXISTS "${SOURCE_OCS2_OCS2_DIR}")
    message(STATUS "Found ocs2_ocs2 at: ${SOURCE_OCS2_OCS2_DIR}")
  else()
    message(FATAL_ERROR "Couldn't find ocs2_ocs2")
  endif()

  # find ocs2_mpc
  find_path(SOURCE_OCS2_MPC_DIR NAMES ocs2_mpc/MPC_BASE.h PATHS ${ocs2_mpc_INCLUDE_DIRS}... NO_DEFAULT_PATH)
  if(EXISTS "${SOURCE_OCS2_MPC_DIR}")
    message(STATUS "Found ocs2_mpc at: ${SOURCE_OCS2_MPC_DIR}")
  else()
    message(FATAL_ERROR "Couldn't find ocs2_mpc")
  endif()

  # find ocs2_comm_interfaces
  find_path(SOURCE_OCS2_COMM_INTERFACES_DIR NAMES ocs2_comm_interfaces/SystemObservation.h PATHS ${ocs2_comm_interfaces_INCLUDE_DIRS}... NO_DEFAULT_PATH)
  if(EXISTS "${SOURCE_OCS2_COMM_INTERFACES_DIR}")
    message(STATUS "Found ocs2_comm_interfaces at: ${SOURCE_OCS2_COMM_INTERFACES_DIR}")
  else()
    message(FATAL_ERROR "Couldn't find ocs2_comm_interfaces")
  endif()

  # find ocs2_ballbot_example
  find_path(SOURCE_OCS2_BALLBOT_EXAMPLE_DIR NAMES ocs2_ballbot_example/definitions.h PATHS ${ocs2_ballbot_example_INCLUDE_DIRS}... NO_DEFAULT_PATH)
  if(EXISTS "${SOURCE_OCS2_BALLBOT_EXAMPLE_DIR}")
    message(STATUS "Found ocs2_ballbot_example at: ${SOURCE_OCS2_BALLBOT_EXAMPLE_DIR}")
  else()
    message(FATAL_ERROR "Couldn't find ocs2_ballbot_example")
  endif()

  # find cart_pole_example
  find_path(SOURCE_OCS2_CART_POLE_EXAMPLE_DIR NAMES ocs2_cart_pole_example/definitions.h PATHS ${ocs2_cart_pole_example_INCLUDE_DIRS}... NO_DEFAULT_PATH)
  if(EXISTS "${SOURCE_OCS2_CART_POLE_EXAMPLE_DIR}")
    message(STATUS "Found cart_pole_example at: ${SOURCE_OCS2_CART_POLE_EXAMPLE_DIR}")
  else()
    message(FATAL_ERROR "Couldn't find cart_pole_example")
  endif()

  # find ocs2_double_integrator_example
  find_path(SOURCE_OCS2_DOUBLE_INTEGRATOR_EXAMPLE_DIR NAMES ocs2_double_integrator_example/definitions.h PATHS ${ocs2_double_integrator_example_INCLUDE_DIRS}... NO_DEFAULT_PATH)
  if(EXISTS "${SOURCE_OCS2_DOUBLE_INTEGRATOR_EXAMPLE_DIR}")
    message(STATUS "Found ocs2_double_integrator_example at: ${SOURCE_OCS2_DOUBLE_INTEGRATOR_EXAMPLE_DIR}")
  else()
    message(FATAL_ERROR "Couldn't find ocs2_double_integrator_example")
  endif()

  # find ocs2_quadrotor_example
  find_path(SOURCE_OCS2_QUADROTOR_EXAMPLE_DIR NAMES ocs2_quadrotor_example/definitions.h PATHS ${ocs2_quadrotor_example_INCLUDE_DIRS}... NO_DEFAULT_PATH)
  if(EXISTS "${SOURCE_OCS2_QUADROTOR_EXAMPLE_DIR}")
    message(STATUS "Found ocs2_quadrotor_example at: ${SOURCE_OCS2_QUADROTOR_EXAMPLE_DIR}")
  else()
    message(FATAL_ERROR "Couldn't find ocs2_quadrotor_example")
  endif()


  # find ocs2_robotic_tools
  find_path(SOURCE_OCS2_ROBOTIC_TOOLS_DIR NAMES ocs2_robotic_tools/common/RobotInterfaceBase.h PATHS ${ocs2_robotic_tools_INCLUDE_DIRS}... NO_DEFAULT_PATH)
  if(EXISTS "${SOURCE_OCS2_ROBOTIC_TOOLS_DIR}")
    message(STATUS "Found ocs2_robotic_tools at: ${SOURCE_OCS2_ROBOTIC_TOOLS_DIR}")
  else()
    message(FATAL_ERROR "Couldn't find ocs2_robotic_tools")
  endif()

  # Set some environment variables
  Set(DOCSOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/doc)
  Set(DOXYGEN_IMAGE_DIRS "\"${CMAKE_CURRENT_SOURCE_DIR}/logo/\"" )
  Set(DOXYGEN_SOURCE_DIRS "${DOXYGEN_SOURCE_DIRS} \"${SOURCE_OCS2_CORE_DIR}/../\" \"${SOURCE_OCS2_FRANK_WOLFE_DIR}/../\" \"${SOURCE_OCS2_SLQ_DIR}/../\" \"${SOURCE_OCS2_OCS2_DIR}/../\" \"${SOURCE_OCS2_MPC_DIR}/../\" \"${SOURCE_OCS2_COMM_INTERFACES_DIR}/../\" \"${SOURCE_OCS2_BALLBOT_EXAMPLE_DIR}/../\" \"${SOURCE_OCS2_CART_POLE_EXAMPLE_DIR}/../\" \"${SOURCE_OCS2_DOUBLE_INTEGRATOR_EXAMPLE_DIR}/../\" \"${SOURCE_OCS2_QUADROTOR_EXAMPLE_DIR}/../\" \"${SOURCE_OCS2_ROBOTIC_TOOLS_DIR}/../\" \"${CMAKE_CURRENT_SOURCE_DIR}/\" ")
  Set(HTML_DIR ${CMAKE_CURRENT_SOURCE_DIR}/doc/html)
  MAKE_DIRECTORY(${HTML_DIR})

  # Add configure file
  CONFIGURE_FILE(${DOCSOURCE_DIR}/ocs2_doc_doxyfile.in ${DOCSOURCE_DIR}/ocs2_doc_doxyfile IMMEDIATE)

  # Build doxygen
  execute_process(COMMAND ${DOXYGEN} ${DOCSOURCE_DIR}/ocs2_doc_doxyfile)

  # Unset cache files
  unset(${SOURCE_OCS2_CORE_DIR} CACHE)

ELSE(DOXYGEN_FOUND)
  message(FATAL_ERROR "Doxygen not found")
ENDIF(DOXYGEN_FOUND)
