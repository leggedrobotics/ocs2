// python bindings
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <ocs2_double_integrator_noros_example/DoubleIntegratorPyBindings.hpp>

using namespace ocs2::double_integrator;
using namespace pybind11::literals;

PYBIND11_MAKE_OPAQUE(DoubleIntegratorPyBindings::scalar_array_t);
PYBIND11_MAKE_OPAQUE(DoubleIntegratorPyBindings::state_vector_array_t);
PYBIND11_MAKE_OPAQUE(DoubleIntegratorPyBindings::input_vector_array_t);

PYBIND11_MODULE(DoubleIntegratorPyBindings2, m) {
  // bind the actual interface
  pybind11::class_<DoubleIntegratorPyBindings>(m, "mpc_interface")
      .def(pybind11::init<const std::string&>())
      .def("setObservation", &DoubleIntegratorPyBindings::setObservation, "t"_a, "x"_a.noconvert())
      .def("advanceMpc", &DoubleIntegratorPyBindings::advanceMpc)
      .def("getMpcSolution", &DoubleIntegratorPyBindings::getMpcSolution, "t"_a.noconvert(), "x"_a.noconvert(), "u"_a.noconvert(), "Vx"_a.noconvert());

  // bind scalar_array_t so it can be used natively in python
  pybind11::class_<DoubleIntegratorPyBindings::scalar_array_t>(m, "scalar_array")
      .def(pybind11::init<>())
      .def("clear", &DoubleIntegratorPyBindings::scalar_array_t::clear)
      .def("pop_back", &DoubleIntegratorPyBindings::scalar_array_t::pop_back)
      .def("__getitem__",
           [](const DoubleIntegratorPyBindings::scalar_array_t& v, size_t i) {
             if (i >= v.size()) throw pybind11::index_error();
             return v[i];
           })
      .def("__setitem__",
           [](DoubleIntegratorPyBindings::scalar_array_t& v, size_t i, DoubleIntegratorPyBindings::scalar_array_t::value_type val) {
             if (i >= v.size()) throw pybind11::index_error();
             v[i] = val;
           })
      .def("__len__", [](const DoubleIntegratorPyBindings::scalar_array_t& v) { return v.size(); })
      .def("__iter__", [](DoubleIntegratorPyBindings::scalar_array_t& v) { return pybind11::make_iterator(v.begin(), v.end()); },
           pybind11::keep_alive<0, 1>()); /* Keep vector alive while iterator is used */

  // bind state_vector_array_t so it can be used natively in python
  pybind11::class_<DoubleIntegratorPyBindings::state_vector_array_t>(m, "state_vector_array")
      .def(pybind11::init<>())
      .def("clear", &DoubleIntegratorPyBindings::state_vector_array_t::clear)
      .def("pop_back", &DoubleIntegratorPyBindings::state_vector_array_t::pop_back)
      .def("__getitem__",
           [](const DoubleIntegratorPyBindings::state_vector_array_t& v, size_t i) {
             if (i >= v.size()) throw pybind11::index_error();
             return v[i];
           })
      .def("__setitem__",
           [](DoubleIntegratorPyBindings::state_vector_array_t& v, size_t i,
              DoubleIntegratorPyBindings::state_vector_array_t::value_type val) {
             if (i >= v.size()) throw pybind11::index_error();
             v[i] = val;
           })
      .def("__len__", [](const DoubleIntegratorPyBindings::state_vector_array_t& v) { return v.size(); })
      .def("__iter__", [](DoubleIntegratorPyBindings::state_vector_array_t& v) { return pybind11::make_iterator(v.begin(), v.end()); },
           pybind11::keep_alive<0, 1>()); /* Keep vector alive while iterator is used */

  // bind input_vector_array_t so it can be used natively in python
  pybind11::class_<DoubleIntegratorPyBindings::input_vector_array_t>(m, "input_vector_array_t")
      .def(pybind11::init<>())
      .def("clear", &DoubleIntegratorPyBindings::input_vector_array_t::clear)
      .def("pop_back", &DoubleIntegratorPyBindings::input_vector_array_t::pop_back)
      .def("__getitem__",
           [](const DoubleIntegratorPyBindings::input_vector_array_t& v, size_t i) {
             if (i >= v.size()) throw pybind11::index_error();
             return v[i];
           })
      .def("__setitem__",
           [](DoubleIntegratorPyBindings::input_vector_array_t& v, size_t i,
              DoubleIntegratorPyBindings::input_vector_array_t::value_type val) {
             if (i >= v.size()) throw pybind11::index_error();
             v[i] = val;
           })
      .def("__len__", [](const DoubleIntegratorPyBindings::input_vector_array_t& v) { return v.size(); })
      .def("__iter__", [](DoubleIntegratorPyBindings::input_vector_array_t& v) { return pybind11::make_iterator(v.begin(), v.end()); },
           pybind11::keep_alive<0, 1>()); /* Keep vector alive while iterator is used */
}
