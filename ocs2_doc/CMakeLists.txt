cmake_minimum_required (VERSION 2.8)

project (ocs2_doc)

find_package(Doxygen REQUIRED)

set(CATKIN_PACKAGE_DEPENDENCIES
  ocs2_core
  ocs2_oc
  ocs2_frank_wolfe
  ocs2_ddp
  ocs2_ocs2
  ocs2_raisim
  ocs2_mpc
  ocs2_comm_interfaces
  ocs2_ballbot_example
  ocs2_cart_pole_example
  ocs2_cart_pole_raisim_example
  ocs2_double_integrator_example
  ocs2_quadrotor_example
  ocs2_robotic_tools
)

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_PACKAGE_DEPENDENCIES}
)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
  DEPENDS doxygen
)

# find ocs2_core
find_path(SOURCE_OCS2_CORE_DIR NAMES ocs2_core/Dimensions.h PATHS ${ocs2_core_INCLUDE_DIRS}... NO_DEFAULT_PATH)
if(EXISTS "${SOURCE_OCS2_CORE_DIR}")
  set(SOURCE_OCS2_CORE_DIR "${SOURCE_OCS2_CORE_DIR}/ocs2_core")
  message(STATUS "Found ocs2_core at: ${SOURCE_OCS2_CORE_DIR}")
else()
  message(FATAL_ERROR "Couldn't find ocs2_core")
endif()

# find ocs2_oc
find_path(SOURCE_OCS2_OC_DIR NAMES ocs2_oc/oc_solver/Solver_BASE.h PATHS ${ocs2_oc_INCLUDE_DIRS}... NO_DEFAULT_PATH)
if(EXISTS "${SOURCE_OCS2_OC_DIR}")
  set(SOURCE_OCS2_OC_DIR "${SOURCE_OCS2_OC_DIR}/ocs2_oc/")
  message(STATUS "Found ocs2_oc at: ${SOURCE_OCS2_OC_DIR}")
else()
  message(FATAL_ERROR "Couldn't find ocs2_oc")
endif()

# find ocs2_frank_wolfe
find_path(SOURCE_OCS2_FRANK_WOLFE_DIR NAMES ocs2_frank_wolfe/GradientDescent.h PATHS ${ocs2_frank_wolfe_INCLUDE_DIRS}... NO_DEFAULT_PATH)
if(EXISTS "${SOURCE_OCS2_FRANK_WOLFE_DIR}")
  set(SOURCE_OCS2_FRANK_WOLFE_DIR "${SOURCE_OCS2_FRANK_WOLFE_DIR}/ocs2_frank_wolfe")
  message(STATUS "Found ocs2_frank_wolfe at: ${SOURCE_OCS2_FRANK_WOLFE_DIR}")
else()
  message(FATAL_ERROR "Couldn't find ocs2_frank_wolfe")
endif()

# find ocs2_ddp
find_path(SOURCE_OCS2_DDP_BASE_DIR NAMES ocs2_ddp/DDP_BASE.h PATHS ${ocs2_ddp_INCLUDE_DIRS}... NO_DEFAULT_PATH)
if(EXISTS "${SOURCE_OCS2_DDP_BASE_DIR}")
  set(SOURCE_OCS2_DDP_BASE_DIR "${SOURCE_OCS2_DDP_BASE_DIR}/ocs2_ddp")
  message(STATUS "Found ocs2_ddp at: ${SOURCE_OCS2_DDP_BASE_DIR}")
else()
  message(FATAL_ERROR "Couldn't find ocs2_ddp")
endif()

# find ocs2_ocs2
find_path(SOURCE_OCS2_OCS2_DIR NAMES ocs2_ocs2/OCS2Projected.h PATHS ${ocs2_ocs2_INCLUDE_DIRS}... NO_DEFAULT_PATH)
if(EXISTS "${SOURCE_OCS2_OCS2_DIR}")
  set(SOURCE_OCS2_OCS2_DIR "${SOURCE_OCS2_OCS2_DIR}/ocs2_ocs2")
  message(STATUS "Found ocs2_ocs2 at: ${SOURCE_OCS2_OCS2_DIR}")
else()
  message(FATAL_ERROR "Couldn't find ocs2_ocs2")
endif()

# find ocs2_ocs2
find_path(SOURCE_OCS2_RAISIM_DIR NAMES ocs2_raisim/RaisimRollout.h PATHS ${ocs2_raisim_INCLUDE_DIRS}... NO_DEFAULT_PATH)
if(EXISTS "${SOURCE_OCS2_RAISIM_DIR}")
  set(SOURCE_OCS2_RAISIM_DIR "${SOURCE_OCS2_RAISIM_DIR}/ocs2_raisim")
  message(STATUS "Found ocs2_raisim at: ${SOURCE_OCS2_RAISIM_DIR}")
else()
  message(FATAL_ERROR "Couldn't find ocs2_raisim")
endif()

# find ocs2_mpc
find_path(SOURCE_OCS2_MPC_DIR NAMES ocs2_mpc/MPC_BASE.h PATHS ${ocs2_mpc_INCLUDE_DIRS}... NO_DEFAULT_PATH)
if(EXISTS "${SOURCE_OCS2_MPC_DIR}")
  set(SOURCE_OCS2_MPC_DIR "${SOURCE_OCS2_MPC_DIR}/ocs2_mpc")
  message(STATUS "Found ocs2_mpc at: ${SOURCE_OCS2_MPC_DIR}")
else()
  message(FATAL_ERROR "Couldn't find ocs2_mpc")
endif()

# find ocs2_comm_interfaces
find_path(SOURCE_OCS2_COMM_INTERFACES_DIR NAMES ocs2_comm_interfaces/SystemObservation.h PATHS ${ocs2_comm_interfaces_INCLUDE_DIRS}... NO_DEFAULT_PATH)
if(EXISTS "${SOURCE_OCS2_COMM_INTERFACES_DIR}")
  set(SOURCE_OCS2_COMM_INTERFACES_DIR "${SOURCE_OCS2_COMM_INTERFACES_DIR}/ocs2_comm_interfaces")
  message(STATUS "Found ocs2_comm_interfaces at: ${SOURCE_OCS2_COMM_INTERFACES_DIR}")
else()
  message(FATAL_ERROR "Couldn't find ocs2_comm_interfaces")
endif()

# find ocs2_ballbot_example
find_path(SOURCE_OCS2_BALLBOT_EXAMPLE_DIR NAMES ocs2_ballbot_example/definitions.h PATHS ${ocs2_ballbot_example_INCLUDE_DIRS}... NO_DEFAULT_PATH)
if(EXISTS "${SOURCE_OCS2_BALLBOT_EXAMPLE_DIR}")
  set(SOURCE_OCS2_BALLBOT_EXAMPLE_DIR "${SOURCE_OCS2_BALLBOT_EXAMPLE_DIR}/ocs2_ballbot_example")
  message(STATUS "Found ocs2_ballbot_example at: ${SOURCE_OCS2_BALLBOT_EXAMPLE_DIR}")
else()
  message(FATAL_ERROR "Couldn't find ocs2_ballbot_example")
endif()

# find cart_pole_example
find_path(SOURCE_OCS2_CART_POLE_EXAMPLE_DIR NAMES ocs2_cart_pole_example/definitions.h PATHS ${ocs2_cart_pole_example_INCLUDE_DIRS}... NO_DEFAULT_PATH)
if(EXISTS "${SOURCE_OCS2_CART_POLE_EXAMPLE_DIR}")
  set(SOURCE_OCS2_CART_POLE_EXAMPLE_DIR "${SOURCE_OCS2_CART_POLE_EXAMPLE_DIR}/ocs2_cart_pole_example")
  message(STATUS "Found cart_pole_example at: ${SOURCE_OCS2_CART_POLE_EXAMPLE_DIR}")
else()
  message(FATAL_ERROR "Couldn't find cart_pole_example")
endif()

# find ocs2_cart_pole_raisim_example
find_path(SOURCE_OCS2_CART_POLE_RAISIM_EXAMPLE_DIR NAMES ocs2_cart_pole_raisim_example/CartpoleRaisimConversions.h PATHS ${ocs2_cart_pole_raisim_example_INCLUDE_DIRS}... NO_DEFAULT_PATH)
if(EXISTS "${SOURCE_OCS2_CART_POLE_RAISIM_EXAMPLE_DIR}")
  set(SOURCE_OCS2_CART_POLE_RAISIM_EXAMPLE_DIR "${SOURCE_OCS2_CART_POLE_RAISIM_EXAMPLE_DIR}/ocs2_cart_pole_raisim_example")
  message(STATUS "Found cart_pole_raisim_example at: ${SOURCE_OCS2_CART_POLE_RAISIM_EXAMPLE_DIR}")
else()
  message(FATAL_ERROR "Couldn't find cart_pole_raisim_example")
endif()

# find ocs2_double_integrator_example
find_path(SOURCE_OCS2_DOUBLE_INTEGRATOR_EXAMPLE_DIR NAMES ocs2_double_integrator_example/definitions.h PATHS ${ocs2_double_integrator_example_INCLUDE_DIRS}... NO_DEFAULT_PATH)
if(EXISTS "${SOURCE_OCS2_DOUBLE_INTEGRATOR_EXAMPLE_DIR}")
  set(SOURCE_OCS2_DOUBLE_INTEGRATOR_EXAMPLE_DIR "${SOURCE_OCS2_DOUBLE_INTEGRATOR_EXAMPLE_DIR}/ocs2_double_integrator_example")
  message(STATUS "Found ocs2_double_integrator_example at: ${SOURCE_OCS2_DOUBLE_INTEGRATOR_EXAMPLE_DIR}")
else()
  message(FATAL_ERROR "Couldn't find ocs2_double_integrator_example")
endif()

# find ocs2_quadrotor_example
find_path(SOURCE_OCS2_QUADROTOR_EXAMPLE_DIR NAMES ocs2_quadrotor_example/definitions.h PATHS ${ocs2_quadrotor_example_INCLUDE_DIRS}... NO_DEFAULT_PATH)
if(EXISTS "${SOURCE_OCS2_QUADROTOR_EXAMPLE_DIR}")
  set(SOURCE_OCS2_QUADROTOR_EXAMPLE_DIR "${SOURCE_OCS2_QUADROTOR_EXAMPLE_DIR}/ocs2_quadrotor_example")
  message(STATUS "Found ocs2_quadrotor_example at: ${SOURCE_OCS2_QUADROTOR_EXAMPLE_DIR}")
else()
  message(FATAL_ERROR "Couldn't find ocs2_quadrotor_example")
endif()


# find ocs2_robotic_tools
find_path(SOURCE_OCS2_ROBOTIC_TOOLS_DIR NAMES ocs2_robotic_tools/common/RobotInterface.h PATHS ${ocs2_robotic_tools_INCLUDE_DIRS}... NO_DEFAULT_PATH)
if(EXISTS "${SOURCE_OCS2_ROBOTIC_TOOLS_DIR}")
  set(SOURCE_OCS2_ROBOTIC_TOOLS_DIR "${SOURCE_OCS2_ROBOTIC_TOOLS_DIR}/ocs2_robotic_tools")
  message(STATUS "Found ocs2_robotic_tools at: ${SOURCE_OCS2_ROBOTIC_TOOLS_DIR}")
else()
  message(FATAL_ERROR "Couldn't find ocs2_robotic_tools")
endif()

# Set some environment variables
Set(DOCSOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/doc)
Set(DOXYGEN_IMAGE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/doc/img")
Set(DOXYGEN_FILTER_DIR "${CMAKE_CURRENT_SOURCE_DIR}/doc")

Set(DOXYGEN_SOURCE_DIRS "${DOXYGEN_SOURCE_DIRS}\
  \"${SOURCE_OCS2_CORE_DIR}/\"\
  \"${SOURCE_OCS2_OC_DIR}/\"\
  \"${SOURCE_OCS2_FRANK_WOLFE_DIR}/\"\
  \"${SOURCE_OCS2_DDP_DIR}/\"\
  \"${SOURCE_OCS2_OCS2_DIR}/\"\
  \"${SOURCE_OCS2_RAISIM_DIR}/\"\
  \"${SOURCE_OCS2_MPC_DIR}/\"\
  \"${SOURCE_OCS2_COMM_INTERFACES_DIR}/\"\
  \"${SOURCE_OCS2_BALLBOT_EXAMPLE_DIR}/\"\
  \"${SOURCE_OCS2_CART_POLE_EXAMPLE_DIR}/\"\
  \"${SOURCE_OCS2_CART_POLE_RAISIM_EXAMPLE_DIR}/\"\
  \"${SOURCE_OCS2_DOUBLE_INTEGRATOR_EXAMPLE_DIR}/\"\
  \"${SOURCE_OCS2_QUADROTOR_EXAMPLE_DIR}/\"\
  \"${SOURCE_OCS2_ROBOTIC_TOOLS_DIR}/\"\
  \"${CMAKE_CURRENT_SOURCE_DIR}/\"\
  \"${DOCSOURCE_DIR}/\" ")

Set(HTML_DIR ${CMAKE_CURRENT_SOURCE_DIR}/doc/html)
MAKE_DIRECTORY(${HTML_DIR})

# Add configure file
CONFIGURE_FILE(${DOCSOURCE_DIR}/ocs2_doc_doxyfile.in ${DOCSOURCE_DIR}/ocs2_doc_doxyfile IMMEDIATE)

# Build doxygen
execute_process(COMMAND ${DOXYGEN} ${DOCSOURCE_DIR}/ocs2_doc_doxyfile)

# Unset cache files
unset(${SOURCE_OCS2_CORE_DIR} CACHE)
