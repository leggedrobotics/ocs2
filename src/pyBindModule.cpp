#include <ocs2_anymal_interface/AnymalPyBindings.h>

#define ROBOT_EQUAL_STATE_INPUT_DIMS
#include <ocs2_comm_interfaces/ocs2_interfaces/Pybind_Macros.h>

CREATE_ROBOT_PYTHON_BINDINGS(anymal::AnymalPyBindings, AnymalPyBindings)
#undef ROBOT_EQUAL_STATE_INPUT_DIMS
