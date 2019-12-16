/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#pragma once

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

using namespace pybind11::literals;

//! convenience macro to bind all kinds of std::vector-like types
#define VECTOR_TYPE_BINDING(VTYPE, NAME)                                                     \
  pybind11::class_<VTYPE>(m, NAME)                                                           \
      .def(pybind11::init<>())                                                               \
      .def("clear", &VTYPE::clear)                                                           \
      .def("pop_back", &VTYPE::pop_back)                                                     \
      .def("push_back", [](VTYPE& v, const VTYPE::value_type& val) { v.push_back(val); })    \
      .def("resize", [](VTYPE& v, size_t i) { v.resize(i); })                                \
      .def("__getitem__",                                                                    \
           [](const VTYPE& v, size_t i) {                                                    \
             if (i >= v.size()) throw pybind11::index_error();                               \
             return v[i];                                                                    \
           })                                                                                \
      .def("__setitem__",                                                                    \
           [](VTYPE& v, size_t i, VTYPE::value_type val) {                                   \
             if (i >= v.size()) throw pybind11::index_error();                               \
             v[i] = val;                                                                     \
           })                                                                                \
      .def("__len__", [](const VTYPE& v) { return v.size(); })                               \
      .def("__iter__", [](VTYPE& v) { return pybind11::make_iterator(v.begin(), v.end()); }, \
           pybind11::keep_alive<0, 1>()); /* Keep vector alive while iterator is used */

#ifdef ROBOT_EQUAL_STATE_INPUT_DIMS

#define MAKE_OPAQUE_ARRAY_TYPES(PY_INTERFACE)              \
  PYBIND11_MAKE_OPAQUE(PY_INTERFACE::scalar_array_t)       \
  PYBIND11_MAKE_OPAQUE(PY_INTERFACE::state_vector_array_t) \
  PYBIND11_MAKE_OPAQUE(PY_INTERFACE::state_matrix_array_t) \
  PYBIND11_MAKE_OPAQUE(PY_INTERFACE::dynamic_vector_array_t)
#define BIND_VECTOR_TYPES(PY_INTERFACE)                                         \
  VECTOR_TYPE_BINDING(PY_INTERFACE::scalar_array_t, "scalar_array")             \
  VECTOR_TYPE_BINDING(PY_INTERFACE::state_vector_array_t, "state_vector_array") \
  VECTOR_TYPE_BINDING(PY_INTERFACE::state_matrix_array_t, "state_matrix_array") \
  VECTOR_TYPE_BINDING(PY_INTERFACE::dynamic_vector_array_t, "dynamic_vector_array")

#else

#define MAKE_OPAQUE_ARRAY_TYPES(PY_INTERFACE)              \
  PYBIND11_MAKE_OPAQUE(PY_INTERFACE::scalar_array_t)       \
  PYBIND11_MAKE_OPAQUE(PY_INTERFACE::state_vector_array_t) \
  PYBIND11_MAKE_OPAQUE(PY_INTERFACE::input_vector_array_t) \
  PYBIND11_MAKE_OPAQUE(PY_INTERFACE::state_matrix_array_t) \
  PYBIND11_MAKE_OPAQUE(PY_INTERFACE::dynamic_vector_array_t)
#define BIND_VECTOR_TYPES(PY_INTERFACE)                                         \
  VECTOR_TYPE_BINDING(PY_INTERFACE::scalar_array_t, "scalar_array")             \
  VECTOR_TYPE_BINDING(PY_INTERFACE::state_vector_array_t, "state_vector_array") \
  VECTOR_TYPE_BINDING(PY_INTERFACE::input_vector_array_t, "input_vector_array") \
  VECTOR_TYPE_BINDING(PY_INTERFACE::state_matrix_array_t, "state_matrix_array") \
  VECTOR_TYPE_BINDING(PY_INTERFACE::dynamic_vector_array_t, "dynamic_vector_array")

#endif

/**
 * @brief Convenience macro to bind robot interface with all required vectors.
 * @note LIB_NAME must match target name in CMakeLists
 * @fixme(jcarius) redefinition error if state and input dim are the same -- solve this
 */
#define CREATE_ROBOT_PYTHON_BINDINGS(PY_INTERFACE, LIB_NAME)                                                                              \
  /* make vector types opaque so they are not converted to python lists */                                                                \
  MAKE_OPAQUE_ARRAY_TYPES(PY_INTERFACE)                                                                                                   \
  /* create a python module */                                                                                                            \
  PYBIND11_MODULE(LIB_NAME, m) {                                                                                                          \
    /* bind vector types so they can be used natively in python */                                                                        \
    BIND_VECTOR_TYPES(PY_INTERFACE)                                                                                                       \
    /* bind cost desired trajectories class */                                                                                            \
    pybind11::class_<ocs2::CostDesiredTrajectories>(m, "cost_desired_trajectories")                                                       \
        .def(pybind11::init<PY_INTERFACE::scalar_array_t, PY_INTERFACE::dynamic_vector_array_t, PY_INTERFACE::dynamic_vector_array_t>()); \
    /* bind the actual mpc interface */                                                                                                   \
    pybind11::class_<PY_INTERFACE>(m, "mpc_interface")                                                                                    \
        .def(pybind11::init<const std::string&, bool>())                                                                                  \
        .def_property_readonly_static("STATE_DIM",                                                                                        \
                                      [](pybind11::object) { return static_cast<int>(PY_INTERFACE::dim_t::DIMS::STATE_DIM_); })           \
        .def_property_readonly_static("INPUT_DIM",                                                                                        \
                                      [](pybind11::object) { return static_cast<int>(PY_INTERFACE::dim_t::DIMS::INPUT_DIM_); })           \
        .def("setObservation", &PY_INTERFACE::setObservation, "t"_a, "x"_a.noconvert())                                                   \
        .def("setTargetTrajectories", &PY_INTERFACE::setTargetTrajectories, "targetTrajectories"_a)                                       \
        .def("reset", &PY_INTERFACE::reset, "targetTrajectories"_a)                                                                       \
        .def("advanceMpc", &PY_INTERFACE::advanceMpc)                                                                                     \
        .def("getMpcSolution", &PY_INTERFACE::getMpcSolution, "t"_a.noconvert(), "x"_a.noconvert(), "u"_a.noconvert())                    \
        .def("getLinearFeedbackGain", &PY_INTERFACE::getLinearFeedbackGain, "t"_a.noconvert())                                            \
        .def("computeFlowMap", &PY_INTERFACE::computeFlowMap, "t"_a, "x"_a.noconvert(), "u"_a.noconvert())                                \
        .def("setFlowMapDerivativeStateAndControl", &PY_INTERFACE::setFlowMapDerivativeStateAndControl, "t"_a, "x"_a.noconvert(),         \
             "u"_a.noconvert())                                                                                                           \
        .def("computeFlowMapDerivativeState", &PY_INTERFACE::computeFlowMapDerivativeState)                                               \
        .def("computeFlowMapDerivativeInput", &PY_INTERFACE::computeFlowMapDerivativeInput)                                               \
        .def("getIntermediateCost", &PY_INTERFACE::getIntermediateCost, "t"_a, "x"_a.noconvert(), "u"_a.noconvert())                      \
        .def("getIntermediateCostDerivativeState", &PY_INTERFACE::getIntermediateCostDerivativeState, "t"_a, "x"_a.noconvert(),           \
             "u"_a.noconvert())                                                                                                           \
        .def("getIntermediateCostDerivativeInput", &PY_INTERFACE::getIntermediateCostDerivativeInput, "t"_a, "x"_a.noconvert(),           \
             "u"_a.noconvert())                                                                                                           \
        .def("getIntermediateCostSecondDerivativeInput", &PY_INTERFACE::getIntermediateCostSecondDerivativeInput, "t"_a,                  \
             "x"_a.noconvert(), "u"_a.noconvert())                                                                                        \
        .def("getValueFunction", &PY_INTERFACE::getValueFunction, "t"_a, "x"_a.noconvert())                                               \
        .def("getValueFunctionStateDerivative", &PY_INTERFACE::getValueFunctionStateDerivative, "t"_a, "x"_a.noconvert())                 \
        .def("getStateInputConstraint", &PY_INTERFACE::getStateInputConstraint, "t"_a, "x"_a.noconvert(), "u"_a.noconvert())              \
        .def("getStateInputConstraintDerivativeControl", &PY_INTERFACE::getStateInputConstraintDerivativeControl, "t"_a,                  \
             "x"_a.noconvert(), "u"_a.noconvert())                                                                                        \
        .def("getStateInputConstraintLagrangian", &PY_INTERFACE::getStateInputConstraintLagrangian, "t"_a, "x"_a.noconvert())             \
        .def("visualizeTrajectory", &PY_INTERFACE::visualizeTrajectory, "t"_a.noconvert(), "x"_a.noconvert(), "u"_a.noconvert(),          \
             "speed"_a);                                                                                                                  \
  }
