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

#include <ocs2_core/Types.h>

using namespace pybind11::literals;

//! convenience macro to bind all kinds of std::vector-like types
#define VECTOR_TYPE_BINDING(VTYPE, NAME)                                                    \
  pybind11::class_<VTYPE>(m, NAME)                                                          \
      .def(pybind11::init<>())                                                              \
      .def("clear", &VTYPE::clear)                                                          \
      .def("pop_back", &VTYPE::pop_back)                                                    \
      .def("push_back", [](VTYPE& v, const VTYPE::value_type& val) { v.push_back(val); })   \
      .def("resize", [](VTYPE& v, size_t i) { v.resize(i); })                               \
      .def("__getitem__",                                                                   \
           [](const VTYPE& v, size_t i) {                                                   \
             if (i >= v.size()) throw pybind11::index_error();                              \
             return v[i];                                                                   \
           })                                                                               \
      .def("__setitem__",                                                                   \
           [](VTYPE& v, size_t i, VTYPE::value_type val) {                                  \
             if (i >= v.size()) throw pybind11::index_error();                              \
             v[i] = val;                                                                    \
           })                                                                               \
      .def("__len__", [](const VTYPE& v) { return v.size(); })                              \
      .def(                                                                                 \
          "__iter__", [](VTYPE& v) { return pybind11::make_iterator(v.begin(), v.end()); }, \
          pybind11::keep_alive<0, 1>()); /* Keep vector alive while iterator is used */

/**
 * @brief Convenience macro to bind robot interface with all required vectors.
 * @note LIB_NAME must match target name in CMakeLists
 */
#define CREATE_ROBOT_PYTHON_BINDINGS(PY_INTERFACE, LIB_NAME)                                                                               \
  /* make vector types opaque so they are not converted to python lists */                                                                 \
  PYBIND11_MAKE_OPAQUE(ocs2::scalar_array_t)                                                                                               \
  PYBIND11_MAKE_OPAQUE(ocs2::vector_array_t)                                                                                               \
  PYBIND11_MAKE_OPAQUE(ocs2::matrix_array_t)                                                                                               \
  /* create a python module */                                                                                                             \
  PYBIND11_MODULE(LIB_NAME, m) {                                                                                                           \
    /* bind vector types so they can be used natively in python */                                                                         \
    VECTOR_TYPE_BINDING(ocs2::scalar_array_t, "scalar_array")                                                                              \
    VECTOR_TYPE_BINDING(ocs2::vector_array_t, "vector_array")                                                                              \
    VECTOR_TYPE_BINDING(ocs2::matrix_array_t, "matrix_array")                                                                              \
    /* bind approximation classes */                                                                                                       \
    pybind11::class_<ocs2::VectorFunctionLinearApproximation>(m, "VectorFunctionLinearApproximation")                                      \
        .def_readwrite("f", &ocs2::VectorFunctionLinearApproximation::f)                                                                   \
        .def_readwrite("dfdx", &ocs2::VectorFunctionLinearApproximation::dfdx)                                                             \
        .def_readwrite("dfdu", &ocs2::VectorFunctionLinearApproximation::dfdu);                                                            \
    pybind11::class_<ocs2::VectorFunctionQuadraticApproximation>(m, "VectorFunctionQuadraticApproximation")                                \
        .def_readwrite("f", &ocs2::VectorFunctionQuadraticApproximation::f)                                                                \
        .def_readwrite("dfdx", &ocs2::VectorFunctionQuadraticApproximation::dfdx)                                                          \
        .def_readwrite("dfdu", &ocs2::VectorFunctionQuadraticApproximation::dfdu)                                                          \
        .def_readwrite("dfdxx", &ocs2::VectorFunctionQuadraticApproximation::dfdxx)                                                        \
        .def_readwrite("dfdux", &ocs2::VectorFunctionQuadraticApproximation::dfdux)                                                        \
        .def_readwrite("dfduu", &ocs2::VectorFunctionQuadraticApproximation::dfduu);                                                       \
    pybind11::class_<ocs2::ScalarFunctionQuadraticApproximation>(m, "ScalarFunctionQuadraticApproximation")                                \
        .def_readwrite("f", &ocs2::ScalarFunctionQuadraticApproximation::f)                                                                \
        .def_readwrite("dfdx", &ocs2::ScalarFunctionQuadraticApproximation::dfdx)                                                          \
        .def_readwrite("dfdu", &ocs2::ScalarFunctionQuadraticApproximation::dfdu)                                                          \
        .def_readwrite("dfdxx", &ocs2::ScalarFunctionQuadraticApproximation::dfdxx)                                                        \
        .def_readwrite("dfdux", &ocs2::ScalarFunctionQuadraticApproximation::dfdux)                                                        \
        .def_readwrite("dfduu", &ocs2::ScalarFunctionQuadraticApproximation::dfduu);                                                       \
    /* bind TargetTrajectories class */                                                                                                    \
    pybind11::class_<ocs2::TargetTrajectories>(m, "TargetTrajectories")                                                                    \
        .def(pybind11::init<ocs2::scalar_array_t, ocs2::vector_array_t, ocs2::vector_array_t>());                                          \
    /* bind the actual mpc interface */                                                                                                    \
    pybind11::class_<PY_INTERFACE>(m, "mpc_interface")                                                                                     \
        .def(pybind11::init<const std::string&, const std::string&, const std::string&>(), "taskFile"_a, "libFolder"_a, "urdfFile"_a = "") \
        .def("getStateDim", &PY_INTERFACE::getStateDim)                                                                                    \
        .def("getInputDim", &PY_INTERFACE::getInputDim)                                                                                    \
        .def("setObservation", &PY_INTERFACE::setObservation, "t"_a, "x"_a.noconvert(), "u"_a.noconvert())                                 \
        .def("setTargetTrajectories", &PY_INTERFACE::setTargetTrajectories, "targetTrajectories"_a)                                        \
        .def("reset", &PY_INTERFACE::reset, "targetTrajectories"_a)                                                                        \
        .def("advanceMpc", &PY_INTERFACE::advanceMpc)                                                                                      \
        .def("getMpcSolution", &PY_INTERFACE::getMpcSolution, "t"_a.noconvert(), "x"_a.noconvert(), "u"_a.noconvert())                     \
        .def("getLinearFeedbackGain", &PY_INTERFACE::getLinearFeedbackGain, "t"_a.noconvert())                                             \
        .def("flowMap", &PY_INTERFACE::flowMap, "t"_a, "x"_a.noconvert(), "u"_a.noconvert())                                               \
        .def("flowMapLinearApproximation", &PY_INTERFACE::flowMapLinearApproximation, "t"_a, "x"_a.noconvert(), "u"_a.noconvert())         \
        .def("cost", &PY_INTERFACE::cost, "t"_a, "x"_a.noconvert(), "u"_a.noconvert())                                                     \
        .def("costQuadraticApproximation", &PY_INTERFACE::costQuadraticApproximation, "t"_a, "x"_a.noconvert(), "u"_a.noconvert())         \
        .def("valueFunction", &PY_INTERFACE::valueFunction, "t"_a, "x"_a.noconvert())                                                      \
        .def("valueFunctionStateDerivative", &PY_INTERFACE::valueFunctionStateDerivative, "t"_a, "x"_a.noconvert())                        \
        .def("stateInputEqualityConstraint", &PY_INTERFACE::stateInputEqualityConstraint, "t"_a, "x"_a.noconvert(), "u"_a.noconvert())     \
        .def("stateInputEqualityConstraintLinearApproximation", &PY_INTERFACE::stateInputEqualityConstraintLinearApproximation, "t"_a,     \
             "x"_a.noconvert(), "u"_a.noconvert())                                                                                         \
        .def("stateInputEqualityConstraintLagrangian", &PY_INTERFACE::stateInputEqualityConstraintLagrangian, "t"_a, "x"_a.noconvert(),    \
             "u"_a.noconvert())                                                                                                            \
        .def("visualizeTrajectory", &PY_INTERFACE::visualizeTrajectory, "t"_a.noconvert(), "x"_a.noconvert(), "u"_a.noconvert(),           \
             "speed"_a);                                                                                                                   \
  }
