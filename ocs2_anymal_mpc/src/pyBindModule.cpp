#include <ocs2_anymal_mpc/AnymalPyBindings.h>

#include <ocs2_python_interface/PybindMacros.h>

// TODO : Macro does not allow two instantiations.
// CREATE_ROBOT_PYTHON_BINDINGS(anymal::AnymalPyBindings<anymal::AnymalModel::Bear>, AnymalBearPyBindings);
CREATE_ROBOT_PYTHON_BINDINGS(anymal::AnymalPyBindings<anymal::AnymalModel::Croc>, AnymalCrocPyBindings);
