/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

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

#include <ocs2_core/Types.h>
#include <ocs2_core/integration/SensitivityIntegrator.h>
#include <ocs2_oc/oc_problem/OptimalControlProblem.h>
#include <ocs2_oc/oc_solver/PerformanceIndex.h>

namespace ocs2 {
namespace multiple_shooting {

/**
 * Results of the transcription at an intermediate node
 */
struct Transcription {
  PerformanceIndex performance;
  VectorFunctionLinearApproximation dynamics;
  ScalarFunctionQuadraticApproximation cost;
  VectorFunctionLinearApproximation constraints;
  VectorFunctionLinearApproximation constraintsProjection;
};

/**
 * Compute the multiple shooting transcription for a single intermediate node.
 *
 * @param optimalControlProblem : Definition of the optimal control problem
 * @param sensitivityDiscretizer : Integrator to use for creating the discrete dynamics.
 * @param projectStateInputEqualityConstraints
 * @param t : Start of the discrete interval
 * @param dt : Duration of the interval
 * @param x : State at start of the interval
 * @param x_next : State at the end of the interval
 * @param u : Input, taken to be constant across the interval.
 * @return multiple shooting transcription for this node.
 */
Transcription setupIntermediateNode(const OptimalControlProblem& optimalControlProblem,
                                    DynamicsSensitivityDiscretizer& sensitivityDiscretizer, bool projectStateInputEqualityConstraints,
                                    scalar_t t, scalar_t dt, const vector_t& x, const vector_t& x_next, const vector_t& u);

/**
 * Compute only the performance index for a single intermediate node.
 * Corresponds to the performance index returned by "setupIntermediateNode"
 */
PerformanceIndex computeIntermediatePerformance(const OptimalControlProblem& optimalControlProblem, DynamicsDiscretizer& discretizer,
                                                scalar_t t, scalar_t dt, const vector_t& x, const vector_t& x_next, const vector_t& u);

/**
 * Results of the transcription at a terminal node
 */
struct TerminalTranscription {
  PerformanceIndex performance;
  ScalarFunctionQuadraticApproximation cost;
  VectorFunctionLinearApproximation constraints;
};

/**
 * Compute the multiple shooting transcription the terminal node.
 *
 * @param optimalControlProblem : Definition of the optimal control problem
 * @param t : Time at the terminal node
 * @param x : Terminal state
 * @return multiple shooting transcription for the terminal node.
 */
TerminalTranscription setupTerminalNode(const OptimalControlProblem& optimalControlProblem, scalar_t t, const vector_t& x);

/**
 * Compute only the performance index for the terminal node.
 * Corresponds to the performance index returned by "setTerminalNode"
 */
PerformanceIndex computeTerminalPerformance(const OptimalControlProblem& optimalControlProblem, scalar_t t, const vector_t& x);

/**
 * Results of the transcription at an event
 */
struct EventTranscription {
  PerformanceIndex performance;
  VectorFunctionLinearApproximation dynamics;
  ScalarFunctionQuadraticApproximation cost;
  VectorFunctionLinearApproximation constraints;
};

/**
 * Compute the jump transcription at an event node.
 *
 * @param optimalControlProblem : Definition of the optimal control problem
 * @param t : Time at the event node
 * @param x : Pre-event state
 * @param x_next : Post-event state
 * @return multiple shooting transcription for the event node.
 */
EventTranscription setupEventNode(const OptimalControlProblem& optimalControlProblem, scalar_t t, const vector_t& x,
                                  const vector_t& x_next);

/**
 * Compute only the performance index for the event node.
 * Corresponds to the performance index returned by "setupEventNode"
 */
PerformanceIndex computeEventPerformance(const OptimalControlProblem& optimalControlProblem, scalar_t t, const vector_t& x,
                                         const vector_t& x_next);

}  // namespace multiple_shooting
}  // namespace ocs2