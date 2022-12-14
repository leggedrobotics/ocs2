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

#include "ocs2_oc/multiple_shooting/PerformanceIndexComputation.h"

#include <ocs2_core/model_data/Metrics.h>

#include "ocs2_oc/approximate_model/LinearQuadraticApproximator.h"
#include "ocs2_oc/multiple_shooting/MetricsComputation.h"

namespace ocs2 {
namespace multiple_shooting {

PerformanceIndex computePerformanceIndex(const Transcription& transcription, scalar_t dt) {
  PerformanceIndex performance;

  // Dynamics
  performance.dynamicsViolationSSE = dt * transcription.dynamics.f.squaredNorm();

  // Costs
  performance.cost = transcription.cost.f;

  // State-input equality constraints
  performance.equalityConstraintsSSE =
      dt * (getEqConstraintsSSE(transcription.stateEqConstraints.f) + getEqConstraintsSSE(transcription.stateInputEqConstraints.f));

  // Inequality constraints.
  performance.inequalityConstraintsSSE =
      dt * (getIneqConstraintsSSE(transcription.stateIneqConstraints.f) + getIneqConstraintsSSE(transcription.stateInputIneqConstraints.f));

  return performance;
}

PerformanceIndex computePerformanceIndex(const EventTranscription& transcription) {
  PerformanceIndex performance;

  // Dynamics
  performance.dynamicsViolationSSE = transcription.dynamics.f.squaredNorm();

  performance.cost = transcription.cost.f;

  // Equality constraints
  performance.equalityConstraintsSSE = getEqConstraintsSSE(transcription.eqConstraints.f);

  // State inequality constraints.
  performance.inequalityConstraintsSSE = getIneqConstraintsSSE(transcription.ineqConstraints.f);

  return performance;
}

PerformanceIndex computePerformanceIndex(const TerminalTranscription& transcription) {
  PerformanceIndex performance;

  performance.cost = transcription.cost.f;

  // Equality constraints
  performance.equalityConstraintsSSE = getEqConstraintsSSE(transcription.eqConstraints.f);

  // State inequality constraints.
  performance.inequalityConstraintsSSE = getIneqConstraintsSSE(transcription.ineqConstraints.f);

  return performance;
}

PerformanceIndex computeIntermediatePerformance(OptimalControlProblem& optimalControlProblem, DynamicsDiscretizer& discretizer, scalar_t t,
                                                scalar_t dt, const vector_t& x, const vector_t& x_next, const vector_t& u) {
  const auto metrics = computeIntermediateMetrics(optimalControlProblem, discretizer, t, dt, x, x_next, u);
  return toPerformanceIndex(metrics, dt);
}

PerformanceIndex computeEventPerformance(OptimalControlProblem& optimalControlProblem, scalar_t t, const vector_t& x,
                                         const vector_t& x_next) {
  const auto metrics = computeEventMetrics(optimalControlProblem, t, x, x_next);
  return toPerformanceIndex(metrics);
}

PerformanceIndex computeTerminalPerformance(OptimalControlProblem& optimalControlProblem, scalar_t t, const vector_t& x) {
  const auto metrics = computeTerminalMetrics(optimalControlProblem, t, x);
  return toPerformanceIndex(metrics);
}

}  // namespace multiple_shooting
}  // namespace ocs2
