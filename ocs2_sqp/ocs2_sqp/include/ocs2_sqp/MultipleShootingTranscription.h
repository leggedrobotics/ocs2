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
#include <ocs2_core/constraint/ConstraintBase.h>
#include <ocs2_core/constraint/PenaltyBase.h>
#include <ocs2_core/cost/CostFunctionBase.h>
#include <ocs2_core/dynamics/SystemDynamicsBase.h>
#include <ocs2_core/integration/SensitivityIntegrator.h>
#include <ocs2_oc/oc_solver/PerformanceIndex.h>

namespace ocs2 {
namespace multiple_shooting {

/**
 * Decides on time discretization along the horizon. Tries to makes step of dt, but will also ensure that eventtimes are part of the
 * discretization.
 *
 *   A simple example here illustrates the mission of this function
 *
 *  Assume:
 *    eventTimes = {3.25, 3.4, 3.88, 4.02, 4.5}
 *    initTime = 3.0
 *    finalTime = 4.0
 *    dt = 0.1
 *    eps = eventDelta
 *
 *  Then the following variables will be:
 *    timeDiscretization = {3.0, 3.1, 3.2, 3.25 + eps, 3.35, 3.4 + eps, 3.5, 3.6, 3.7, 3.8, 3.88 + eps, 3.98, 4.0}
 *
 * @param initTime : start time.
 * @param finalTime : final time.
 * @param dt : desired descretization step.
 * @param eventTimes : Event times where a time discretization must be made.
 * @param eventDelta : Time added after a event time, to make lookup work.
 * @return vector of discrete time points
 */
scalar_array_t timeDiscretizationWithEvents(scalar_t initTime, scalar_t finalTime, scalar_t dt, const scalar_array_t& eventTimes,
                                            scalar_t eventDelta);

/**
 * Results of the transcription at an intermediate node
 */
struct Transcription {
  PerformanceIndex performance;
  VectorFunctionLinearApproximation dynamics;
  ScalarFunctionQuadraticApproximation cost;
  VectorFunctionLinearApproximation constraints;
};

/**
 * Compute the multiple shooting transcription for a single intermediate node.
 *
 * @param systemDynamics : Continuous time dynamics.
 * @param sensitivityDiscretizer : Integrator to use for creating the discrete dynamics.
 * @param costFunction : Continuous time cost function.
 * @param constraintPtr : (Optional) constraints to be evaluated at start of the interval
 * @param penaltyPtr : (Optional) penalty function to use in case of inequality constraints.
 * @param projectStateInputEqualityConstraints
 * @param t : Start of the discrete interval
 * @param dt : Duration of the interval
 * @param x : State at start of the interval
 * @param x_next : State at the end of the interval
 * @param u : Input, taken to be constant across the interval.
 * @return multiple shooting transcription for this node.
 */
Transcription setupIntermediateNode(SystemDynamicsBase& systemDynamics, DynamicsSensitivityDiscretizer& sensitivityDiscretizer,
                                    CostFunctionBase& costFunction, ConstraintBase* constraintPtr, PenaltyBase* penaltyPtr,
                                    bool projectStateInputEqualityConstraints, scalar_t t, scalar_t dt, const vector_t& x,
                                    const vector_t& x_next, const vector_t& u);

/**
 * Compute only the performance index for a single intermediate node.
 * Corresponds to the performance index returned by "setupIntermediateNode"
 */
PerformanceIndex computeIntermediatePerformance(SystemDynamicsBase& systemDynamics, DynamicsDiscretizer& discretizer,
                                                CostFunctionBase& costFunction, ConstraintBase* constraintPtr, PenaltyBase* penaltyPtr,
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
 * @param terminalCostFunctionPtr : (Optional) terminal cost function
 * @param constraintPtr : (Optional) terminal constraints
 * @param t : Time at the terminal node
 * @param x : Terminal state
 * @return multiple shooting transcription for the terminal node.
 */
TerminalTranscription setupTerminalNode(CostFunctionBase* terminalCostFunctionPtr, ConstraintBase* constraintPtr, scalar_t t,
                                        const vector_t& x);

/**
 * Compute only the performance index for the terminal node.
 * Corresponds to the performance index returned by "setTerminalNode"
 */
PerformanceIndex computeTerminalPerformance(CostFunctionBase* terminalCostFunctionPtr, ConstraintBase* constraintPtr, scalar_t t,
                                            const vector_t& x);

}  // namespace multiple_shooting
}  // namespace ocs2