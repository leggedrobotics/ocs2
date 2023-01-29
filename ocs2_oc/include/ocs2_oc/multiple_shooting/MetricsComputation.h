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
#include <ocs2_core/model_data/Metrics.h>

#include "ocs2_oc/multiple_shooting/Transcription.h"
#include "ocs2_oc/oc_problem/OptimalControlProblem.h"

namespace ocs2 {
namespace multiple_shooting {

/**
 * Compute the Metrics for a single intermediate node.
 * @param transcription: multiple shooting transcription for an intermediate node.
 * @return Metrics for a single intermediate node.
 */
Metrics computeMetrics(const Transcription& transcription);

/**
 * Compute the Metrics for the event node.
 * @param transcription: multiple shooting transcription for event node.
 * @return Metrics for a event node.
 */
Metrics computeMetrics(const EventTranscription& transcription);

/**
 * Compute the Metrics for the terminal node.
 * @param transcription: multiple shooting transcription for terminal node.
 * @return Metrics for a terminal node.
 */
Metrics computeMetrics(const TerminalTranscription& transcription);

/**
 * Compute the Metrics for a single intermediate node.
 * @param optimalControlProblem : Definition of the optimal control problem
 * @param discretizer : Integrator to use for creating the discrete dynamics.
 * @param t : Start of the discrete interval
 * @param dt : Duration of the interval
 * @param x : State at start of the interval
 * @param x_next : State at the end of the interval
 * @param u : Input, taken to be constant across the interval.
 * @return Metrics for a single intermediate node.
 */
Metrics computeIntermediateMetrics(OptimalControlProblem& optimalControlProblem, DynamicsDiscretizer& discretizer, scalar_t t, scalar_t dt,
                                   const vector_t& x, const vector_t& x_next, const vector_t& u);

/**
 * Compute the Metrics for the event node.
 * @param optimalControlProblem : Definition of the optimal control problem
 * @param t : Time at the event node
 * @param x : Pre-event state
 * @param x_next : Post-event state
 * @return Metrics for the event node.
 */
Metrics computeEventMetrics(OptimalControlProblem& optimalControlProblem, scalar_t t, const vector_t& x, const vector_t& x_next);

/**
 * Compute the Metrics for the terminal node.
 * @param optimalControlProblem : Definition of the optimal control problem
 * @param t : Time at the terminal node
 * @param x : Terminal state
 * @return Metrics for the terminal node.
 */
Metrics computeTerminalMetrics(OptimalControlProblem& optimalControlProblem, scalar_t t, const vector_t& x);

}  // namespace multiple_shooting
}  // namespace ocs2
