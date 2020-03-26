#include <ocs2_anymal_croc/AnymalCrocPyBindings.h>

#define ROBOT_EQUAL_STATE_INPUT_DIMS
#include <ocs2_comm_interfaces/ocs2_interfaces/Pybind_Macros.h>

CREATE_ROBOT_PYTHON_BINDINGS(anymal::AnymalCrocPyBindings, AnymalCrocPyBindings)
#undef ROBOT_EQUAL_STATE_INPUT_DIMS
