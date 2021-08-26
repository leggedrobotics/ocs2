#pragma once

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <ocs2_core/Types.h>
#include <ocs2_python_interface/PybindMacros.h>

using namespace pybind11::literals;

/**
 * Convenience macro to bind general MPC-Net functionalities and other classes with all required vectors.
 */
#define CREATE_MPCNET_PYTHON_BINDINGS(MPCNET_INTERFACE, LIB_NAME)                                           \
  /* make vector types opaque so they are not converted to python lists */                                  \
  PYBIND11_MAKE_OPAQUE(ocs2::size_array_t)                                                                  \
  PYBIND11_MAKE_OPAQUE(ocs2::scalar_array_t)                                                                \
  PYBIND11_MAKE_OPAQUE(ocs2::vector_array_t)                                                                \
  PYBIND11_MAKE_OPAQUE(ocs2::matrix_array_t)                                                                \
  PYBIND11_MAKE_OPAQUE(std::vector<ocs2::SystemObservation>)                                                \
  PYBIND11_MAKE_OPAQUE(std::vector<ocs2::ModeSchedule>)                                                     \
  PYBIND11_MAKE_OPAQUE(std::vector<ocs2::TargetTrajectories>)                                               \
  PYBIND11_MAKE_OPAQUE(MPCNET_INTERFACE::data_array_t)                                                      \
  PYBIND11_MAKE_OPAQUE(MPCNET_INTERFACE::metrics_array_t)                                                   \
  /* create a python module */                                                                              \
  PYBIND11_MODULE(LIB_NAME, m) {                                                                            \
    /* bind vector types so they can be used natively in python */                                          \
    VECTOR_TYPE_BINDING(ocs2::size_array_t, "size_array")                                                   \
    VECTOR_TYPE_BINDING(ocs2::scalar_array_t, "scalar_array")                                               \
    VECTOR_TYPE_BINDING(ocs2::vector_array_t, "vector_array")                                               \
    VECTOR_TYPE_BINDING(ocs2::matrix_array_t, "matrix_array")                                               \
    VECTOR_TYPE_BINDING(std::vector<ocs2::SystemObservation>, "SystemObservationArray")                     \
    VECTOR_TYPE_BINDING(std::vector<ocs2::ModeSchedule>, "ModeScheduleArray")                               \
    VECTOR_TYPE_BINDING(std::vector<ocs2::TargetTrajectories>, "TargetTrajectoriesArray")                   \
    VECTOR_TYPE_BINDING(MPCNET_INTERFACE::data_array_t, "DataArray")                                        \
    VECTOR_TYPE_BINDING(MPCNET_INTERFACE::metrics_array_t, "MetricsArray")                                  \
    /* bind approximation classes */                                                                        \
    pybind11::class_<ocs2::ScalarFunctionQuadraticApproximation>(m, "ScalarFunctionQuadraticApproximation") \
        .def_readwrite("f", &ocs2::ScalarFunctionQuadraticApproximation::f)                                 \
        .def_readwrite("dfdx", &ocs2::ScalarFunctionQuadraticApproximation::dfdx)                           \
        .def_readwrite("dfdu", &ocs2::ScalarFunctionQuadraticApproximation::dfdu)                           \
        .def_readwrite("dfdxx", &ocs2::ScalarFunctionQuadraticApproximation::dfdxx)                         \
        .def_readwrite("dfdux", &ocs2::ScalarFunctionQuadraticApproximation::dfdux)                         \
        .def_readwrite("dfduu", &ocs2::ScalarFunctionQuadraticApproximation::dfduu);                        \
    /* bind system observation struct */                                                                    \
    pybind11::class_<ocs2::SystemObservation>(m, "SystemObservation")                                       \
        .def(pybind11::init<>())                                                                            \
        .def_readwrite("mode", &ocs2::SystemObservation::mode)                                              \
        .def_readwrite("time", &ocs2::SystemObservation::time)                                              \
        .def_readwrite("state", &ocs2::SystemObservation::state)                                            \
        .def_readwrite("input", &ocs2::SystemObservation::input);                                           \
    /* bind mode schedule struct */                                                                         \
    pybind11::class_<ocs2::ModeSchedule>(m, "ModeSchedule")                                                 \
        .def(pybind11::init<ocs2::scalar_array_t, ocs2::size_array_t>())                                    \
        .def_readwrite("event_times", &ocs2::ModeSchedule::eventTimes)                                      \
        .def_readwrite("mode_sequence", &ocs2::ModeSchedule::modeSequence);                                 \
    /* bind target trajectories class */                                                                    \
    pybind11::class_<ocs2::TargetTrajectories>(m, "TargetTrajectories")                                     \
        .def(pybind11::init<ocs2::scalar_array_t, ocs2::vector_array_t, ocs2::vector_array_t>())            \
        .def_readwrite("time_trajectory", &ocs2::TargetTrajectories::timeTrajectory)                        \
        .def_readwrite("state_trajectory", &ocs2::TargetTrajectories::stateTrajectory)                      \
        .def_readwrite("input_trajectory", &ocs2::TargetTrajectories::inputTrajectory);                     \
    /* bind data point struct */                                                                            \
    pybind11::class_<MPCNET_INTERFACE::data_point_t>(m, "DataPoint")                                        \
        .def(pybind11::init<>())                                                                            \
        .def_readwrite("t", &MPCNET_INTERFACE::data_point_t::t)                                             \
        .def_readwrite("x", &MPCNET_INTERFACE::data_point_t::x)                                             \
        .def_readwrite("u", &MPCNET_INTERFACE::data_point_t::u)                                             \
        .def_readwrite("mode", &MPCNET_INTERFACE::data_point_t::mode)                                       \
        .def_readwrite("generalized_time", &MPCNET_INTERFACE::data_point_t::generalizedTime)                \
        .def_readwrite("relative_state", &MPCNET_INTERFACE::data_point_t::relativeState)                    \
        .def_readwrite("hamiltonian", &MPCNET_INTERFACE::data_point_t::hamiltonian);                        \
    /* bind metrics struct */                                                                               \
    pybind11::class_<MPCNET_INTERFACE::metrics_t>(m, "Metrics")                                             \
        .def(pybind11::init<>())                                                                            \
        .def_readwrite("survival_time", &MPCNET_INTERFACE::metrics_t::survivalTime)                         \
        .def_readwrite("incurred_hamiltonian", &MPCNET_INTERFACE::metrics_t::incurredHamiltonian);          \
  }

/**
 * Convenience macro to bind robot MPC-Net interface.
 */
#define CREATE_ROBOT_MPCNET_PYTHON_BINDINGS(MPCNET_INTERFACE, LIB_NAME)                                                                    \
  /* create a python module */                                                                                                             \
  PYBIND11_MODULE(LIB_NAME, m) {                                                                                                           \
    /* import the general MPC-Net module */                                                                                                \
    pybind11::module::import("ocs2_mpcnet.MpcnetPybindings");                                                                              \
    /* bind actual MPC-Net interface for specific robot */                                                                                 \
    pybind11::class_<MPCNET_INTERFACE>(m, "MpcnetInterface")                                                                               \
        .def(pybind11::init<size_t, size_t>())                                                                                             \
        .def("startDataGeneration", &MPCNET_INTERFACE::startDataGeneration, "alpha"_a, "policyFilePath"_a, "timeStep"_a,                   \
             "dataDecimation"_a, "nSamples"_a, "samplingCovariance"_a.noconvert(), "initialObservations"_a, "modeSchedules"_a,             \
             "targetTrajectories"_a)                                                                                                       \
        .def("isDataGenerationDone", &MPCNET_INTERFACE::isDataGenerationDone)                                                              \
        .def("getGeneratedData", &MPCNET_INTERFACE::getGeneratedData)                                                                      \
        .def("startPolicyEvaluation", &MPCNET_INTERFACE::startPolicyEvaluation, "policyFilePath"_a, "timeStep"_a, "initialObservations"_a, \
             "modeSchedules"_a, "targetTrajectories"_a)                                                                                    \
        .def("isPolicyEvaluationDone", &MPCNET_INTERFACE::isPolicyEvaluationDone)                                                          \
        .def("getComputedMetrics", &MPCNET_INTERFACE::getComputedMetrics);                                                                 \
  }
