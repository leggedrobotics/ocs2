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

#include "ocs2_ipm/IpmPerformanceIndexComputation.h"

#include <ocs2_core/model_data/Metrics.h>

#include <ocs2_oc/approximate_model/LinearQuadraticApproximator.h>
#include <ocs2_oc/multiple_shooting/MetricsComputation.h>
#include <ocs2_oc/multiple_shooting/PerformanceIndexComputation.h>

namespace ocs2 {
namespace ipm {

PerformanceIndex computePerformanceIndex(const multiple_shooting::Transcription& transcription, scalar_t dt, scalar_t barrierParam,
                                         const vector_t& slackStateIneq, const vector_t& slackStateInputIneq) {
  auto performance = multiple_shooting::computePerformanceIndex(transcription, dt);

  if (slackStateIneq.size() > 0) {
    performance.cost -= dt * barrierParam * slackStateIneq.array().log().sum();
  }
  if (slackStateInputIneq.size() > 0) {
    performance.cost -= dt * barrierParam * slackStateInputIneq.array().log().sum();
  }

  if (transcription.stateIneqConstraints.f.size() > 0) {
    performance.equalityConstraintsSSE += dt * (transcription.stateIneqConstraints.f - slackStateIneq).squaredNorm();
  }
  if (transcription.stateInputIneqConstraints.f.size() > 0) {
    performance.equalityConstraintsSSE += dt * (transcription.stateInputIneqConstraints.f - slackStateInputIneq).squaredNorm();
  }

  return performance;
}

PerformanceIndex computePerformanceIndex(const multiple_shooting::TerminalTranscription& transcription, scalar_t barrierParam,
                                         const vector_t& slackIneq) {
  auto performance = multiple_shooting::computePerformanceIndex(transcription);

  if (slackIneq.size() > 0) {
    performance.cost -= barrierParam * slackIneq.array().log().sum();
  }

  if (transcription.ineqConstraints.f.size() > 0) {
    performance.equalityConstraintsSSE += (transcription.ineqConstraints.f - slackIneq).squaredNorm();
  }

  return performance;
}

PerformanceIndex computePerformanceIndex(const multiple_shooting::EventTranscription& transcription, scalar_t barrierParam,
                                         const vector_t& slackIneq) {
  auto performance = multiple_shooting::computePerformanceIndex(transcription);

  if (slackIneq.size() > 0) {
    performance.cost -= barrierParam * slackIneq.array().log().sum();
  }

  if (transcription.ineqConstraints.f.size() > 0) {
    performance.equalityConstraintsSSE += (transcription.ineqConstraints.f - slackIneq).squaredNorm();
  }

  return performance;
}

PerformanceIndex computeIntermediatePerformance(OptimalControlProblem& optimalControlProblem, DynamicsDiscretizer& discretizer, scalar_t t,
                                                scalar_t dt, const vector_t& x, const vector_t& x_next, const vector_t& u,
                                                scalar_t barrierParam, const vector_t& slackStateIneq, const vector_t& slackStateInputIneq,
                                                bool enableStateInequalityConstraints) {
  auto metrics = multiple_shooting::computeIntermediateMetrics(optimalControlProblem, discretizer, t, dt, x, x_next, u);
  if (!enableStateInequalityConstraints) {
    metrics.stateIneqConstraint.clear();
  }
  return toPerformanceIndex(metrics, dt, barrierParam, slackStateIneq, slackStateInputIneq);
}

PerformanceIndex computeEventPerformance(OptimalControlProblem& optimalControlProblem, scalar_t t, const vector_t& x,
                                         const vector_t& x_next, scalar_t barrierParam, const vector_t& slackIneq) {
  const auto metrics = multiple_shooting::computeEventMetrics(optimalControlProblem, t, x, x_next);
  return toPerformanceIndex(metrics, barrierParam, slackIneq);
}

PerformanceIndex computeTerminalPerformance(OptimalControlProblem& optimalControlProblem, scalar_t t, const vector_t& x,
                                            scalar_t barrierParam, const vector_t& slackIneq) {
  const auto metrics = multiple_shooting::computeTerminalMetrics(optimalControlProblem, t, x);
  return toPerformanceIndex(metrics, barrierParam, slackIneq);
}

PerformanceIndex toPerformanceIndex(const Metrics& metrics, scalar_t dt, scalar_t barrierParam, const vector_t& slackStateIneq,
                                    const vector_t& slackStateInputIneq) {
  PerformanceIndex performance = toPerformanceIndex(metrics, dt);

  if (slackStateIneq.size() > 0) {
    performance.cost -= dt * barrierParam * slackStateIneq.array().log().sum();
    performance.equalityConstraintsSSE += dt * (toVector(metrics.stateIneqConstraint) - slackStateIneq).squaredNorm();
  }

  if (slackStateInputIneq.size() > 0) {
    performance.cost -= dt * barrierParam * slackStateInputIneq.array().log().sum();
    performance.equalityConstraintsSSE += dt * (toVector(metrics.stateInputIneqConstraint) - slackStateInputIneq).squaredNorm();
  }

  return performance;
}

PerformanceIndex toPerformanceIndex(const Metrics& metrics, scalar_t barrierParam, const vector_t& slackIneq) {
  PerformanceIndex performance = toPerformanceIndex(metrics);

  if (slackIneq.size() > 0) {
    performance.cost -= barrierParam * slackIneq.array().log().sum();
    performance.equalityConstraintsSSE += (toVector(metrics.stateIneqConstraint) - slackIneq).squaredNorm();
  }

  return performance;
}

}  // namespace ipm
}  // namespace ocs2
