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
#include <ocs2_core/model_data/Metrics.h>
#include <ocs2_oc/multiple_shooting/Transcription.h>
#include <ocs2_oc/oc_data/PerformanceIndex.h>

namespace ocs2 {
namespace ipm {

/**
 * Compute the performance index from the transcription for a single intermediate node.
 *
 * @param transcription : transcription computed by "multiple_shooting::setupIntermediateNode"
 * @param dt : Duration of the interval
 * @param barrierParam : Barrier parameter of the interior point method
 * @param slackStateIneq : Slack variable of the state inequality constraints
 * @param slackStateInputIneq : Slack variable of the state-input inequality constraints
 * @return Performance index for a single intermediate node
 */
PerformanceIndex computePerformanceIndex(const multiple_shooting::Transcription& transcription, scalar_t dt, scalar_t barrierParam,
                                         const vector_t& slackStateIneq, const vector_t& slackStateInputIneq);

/**
 * Compute the performance index from the transcription for the terminal node.
 *
 * @param transcription : transcription computed by "multiple_shooting::setupTerminalNode"
 * @param barrierParam : Barrier parameter of the interior point method
 * @param slackIneq : Slack variable of the inequality constraints
 * @return Performance index for the terminal node
 */
PerformanceIndex computePerformanceIndex(const multiple_shooting::TerminalTranscription& transcription, scalar_t barrierParam,
                                         const vector_t& slackIneq);

/**
 * Compute the performance index from the transcription for the event node.
 *
 * @param transcription : transcription computed by "multiple_shooting::setupEventNode"
 * @param barrierParam : Barrier parameter of the interior point method
 * @param slackIneq : Slack variable of the inequality constraints
 * @return Performance index for the event node
 */
PerformanceIndex computePerformanceIndex(const multiple_shooting::EventTranscription& transcription, scalar_t barrierParam,
                                         const vector_t& slackIneq);

/**
 * Compute only the performance index for a single intermediate node.
 * Corresponds to the performance index computed from the multiple_shooting::Transcription returned by
 * "multiple_shooting::setupIntermediateNode"
 *
 * @param optimalControlProblem : Definition of the optimal control problem
 * @param discretizer : Integrator to use for creating the discrete dynamics
 * @param t : Start of the discrete interval
 * @param dt : Duration of the interval
 * @param x : State at start of the interval
 * @param x_next : State at the end of the interval
 * @param u : Input, taken to be constant across the interval
 * @param barrierParam : Barrier parameter of the interior point method
 * @param slackStateIneq : Slack variable of the state inequality constraints
 * @param slackStateInputIneq : Slack variable of the state-input inequality constraints
 * @return Performance index for a single intermediate node
 */
PerformanceIndex computeIntermediatePerformance(OptimalControlProblem& optimalControlProblem, DynamicsDiscretizer& discretizer, scalar_t t,
                                                scalar_t dt, const vector_t& x, const vector_t& x_next, const vector_t& u,
                                                scalar_t barrierParam, const vector_t& slackStateIneq, const vector_t& slackStateInputIneq,
                                                bool enableStateInequalityConstraints);

/**
 * Compute only the performance index for the terminal node.
 * Corresponds to the performance index computed from multiple_shooting::TerminalTranscription returned by
 * "multiple_shooting::setTerminalNode"
 *
 * @param optimalControlProblem : Definition of the optimal control problem
 * @param t : Time at the terminal node
 * @param x : Terminal state
 * @param barrierParam : Barrier parameter of the interior point method
 * @param slackIneq : Slack variable of the inequality constraints
 * @return Performance index for the terminal node
 */
PerformanceIndex computeTerminalPerformance(OptimalControlProblem& optimalControlProblem, scalar_t t, const vector_t& x,
                                            scalar_t barrierParam, const vector_t& slackIneq);

/**
 * Compute only the performance index for the event node.
 * Corresponds to the performance index computed from multiple_shooting::EventTranscription returned by
 * "multiple_shooting::setEventNode"
 *
 * @param optimalControlProblem : Definition of the optimal control problem
 * @param t : Time at the event node
 * @param x : Pre-event state
 * @param x_next : Post-event state
 * @param barrierParam : Barrier parameter of the interior point method
 * @param slackIneq : Slack variable of the inequality constraints
 * @return Performance index for the event node
 */
PerformanceIndex computeEventPerformance(OptimalControlProblem& optimalControlProblem, scalar_t t, const vector_t& x,
                                         const vector_t& x_next, scalar_t barrierParam, const vector_t& slackIneq);

/**
 * Computes the PerformanceIndex of the interior point method based on a given Metrics at an intermediate node.
 *
 * @param metrics : Metrics at an intermediate node
 * @param dt : Duration of the interval
 * @param barrierParam : Barrier parameter of the interior point method
 * @param slackStateIneq : Slack variable of the state inequality constraints
 * @param slackStateInputIneq : Slack variable of the state-input inequality constraints
 * @return Performance index of the interior point method
 */
PerformanceIndex toPerformanceIndex(const Metrics& metrics, scalar_t dt, scalar_t barrierParam, const vector_t& slackStateIneq,
                                    const vector_t& slackStateInputIneq);

/**
 * Computes the PerformanceIndex of the interior point method based on a given Metrics at the event or terminal node.
 *
 * @param metrics : Metrics at the event or terminal node
 * @param barrierParam : Barrier parameter of the interior point method
 * @param slackIneq : Slack variable of the inequality constraints
 * @return Performance index of the interior point method
 */
PerformanceIndex toPerformanceIndex(const Metrics& metrics, scalar_t barrierParam, const vector_t& slackIneq);

}  // namespace ipm
}  // namespace ocs2
