// python bindings
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <ocs2_double_integrator_noros_example/DoubleIntegratorPyBindings.hpp>

using namespace ocs2::double_integrator;
using namespace pybind11::literals;

// convenience macro to bind all kinds of std::vector-like types
#define VECTOR_TYPE_BINDING(VTYPE, NAME)\
  pybind11::class_<VTYPE>(m, NAME)\
      .def(pybind11::init<>())\
      .def("clear", &VTYPE::clear)\
      .def("pop_back", &VTYPE::pop_back)\
      .def("__getitem__",\
           [](const VTYPE& v, size_t i) {\
             if (i >= v.size()) throw pybind11::index_error();\
             return v[i];\
           })\
      .def("__setitem__",\
           [](VTYPE& v, size_t i, VTYPE::value_type val) {\
             if (i >= v.size()) throw pybind11::index_error();\
             v[i] = val;\
           })\
      .def("__len__", [](const VTYPE& v) { return v.size(); })\
      .def("__iter__", [](VTYPE& v) { return pybind11::make_iterator(v.begin(), v.end()); },\
           pybind11::keep_alive<0, 1>()); /* Keep vector alive while iterator is used */

PYBIND11_MAKE_OPAQUE(DoubleIntegratorPyBindings::scalar_array_t);
PYBIND11_MAKE_OPAQUE(DoubleIntegratorPyBindings::state_vector_array_t);
PYBIND11_MAKE_OPAQUE(DoubleIntegratorPyBindings::input_vector_array_t);
PYBIND11_MAKE_OPAQUE(DoubleIntegratorPyBindings::state_matrix_array_t);

PYBIND11_MODULE(DoubleIntegratorPyBindings3, m) {
  // bind the actual interface
  pybind11::class_<DoubleIntegratorPyBindings>(m, "mpc_interface")
      .def(pybind11::init<const std::string&, bool>())
      .def_property_readonly_static("STATE_DIM", [](pybind11::object) {return static_cast<int>(double_integrator_dims::STATE_DIM_);})
      .def_property_readonly_static("INPUT_DIM", [](pybind11::object) {return static_cast<int>(double_integrator_dims::INPUT_DIM_);})
      .def("setObservation", &DoubleIntegratorPyBindings::setObservation, "t"_a, "x"_a.noconvert())
      .def("advanceMpc", &DoubleIntegratorPyBindings::advanceMpc)
      .def("getMpcSolution", &DoubleIntegratorPyBindings::getMpcSolution, "t"_a.noconvert(), "x"_a.noconvert(), "u"_a.noconvert(), "sigmaX"_a.noconvert())
      .def("computeFlowMap", &DoubleIntegratorPyBindings::computeFlowMap, "t"_a, "x"_a.noconvert(), "u"_a.noconvert())
      .def("setFlowMapDerivativeStateAndControl", &DoubleIntegratorPyBindings::setFlowMapDerivativeStateAndControl, "t"_a, "x"_a.noconvert(), "u"_a.noconvert())
      .def("computeFlowMapDerivativeState", &DoubleIntegratorPyBindings::computeFlowMapDerivativeState)
      .def("computeFlowMapDerivativeInput", &DoubleIntegratorPyBindings::computeFlowMapDerivativeInput)
      .def("getRunningCost", &DoubleIntegratorPyBindings::getRunningCost, "t"_a, "x"_a.noconvert(), "u"_a.noconvert())
      .def("getRunningCostDerivativeState", &DoubleIntegratorPyBindings::getRunningCostDerivativeState, "t"_a, "x"_a.noconvert(), "u"_a.noconvert())
      .def("getRunningCostDerivativeInput", &DoubleIntegratorPyBindings::getRunningCostDerivativeInput, "t"_a, "x"_a.noconvert(), "u"_a.noconvert())
      .def("getValueFunctionStateDerivative", &DoubleIntegratorPyBindings::getValueFunctionStateDerivative, "t"_a, "x"_a.noconvert());

  // bind vector types so they can be used natively in python
  VECTOR_TYPE_BINDING(DoubleIntegratorPyBindings::scalar_array_t, "scalar_array")
  VECTOR_TYPE_BINDING(DoubleIntegratorPyBindings::state_vector_array_t, "state_vector_array")
  VECTOR_TYPE_BINDING(DoubleIntegratorPyBindings::input_vector_array_t, "input_vector_array_t")
  VECTOR_TYPE_BINDING(DoubleIntegratorPyBindings::state_matrix_array_t, "state_matrix_array_t")
}
