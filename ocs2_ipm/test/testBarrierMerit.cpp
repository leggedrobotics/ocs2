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

#include <gtest/gtest.h>
#include <ocs2_core/model_data/Metrics.h>
#include <ocs2_ipm/IpmHelpers.h>
#include <ocs2_ipm/IpmPerformanceIndexComputation.h>

using namespace ocs2;

TEST(IpmBarrierMerit, IntermediateMeritMatchesCondensedGradientAndHessian) {
  constexpr double dt = 0.05, mu = 0.01;
  const Eigen::Vector2d point(0.2, 0.1);
  const auto makeTranscription = [](const Eigen::Vector2d &xu) {
    multiple_shooting::Transcription node;
    node.cost = ScalarFunctionQuadraticApproximation::Zero(1, 1);
    node.dynamics = VectorFunctionLinearApproximation::Zero(1, 1, 1);
    node.stateEqConstraints = VectorFunctionLinearApproximation::Zero(0, 1, 0);
    node.stateInputEqConstraints =
        VectorFunctionLinearApproximation::Zero(0, 1, 1);
    node.stateIneqConstraints =
        VectorFunctionLinearApproximation::Zero(2, 1, 0);
    node.stateIneqConstraints.f << 2.0 + xu[0], 3.0 - 2.0 * xu[0];
    node.stateIneqConstraints.dfdx << 1.0, -2.0;
    node.stateInputIneqConstraints =
        VectorFunctionLinearApproximation::Zero(2, 1, 1);
    node.stateInputIneqConstraints.f << 1.0 + xu[0] + 2.0 * xu[1],
        4.0 - 0.3 * xu[0] - xu[1];
    node.stateInputIneqConstraints.dfdx << 1.0, -0.3;
    node.stateInputIneqConstraints.dfdu << 2.0, -1.0;
    return node;
  };
  const auto merit = [&](const Eigen::Vector2d &xu) {
    const auto node = makeTranscription(xu);
    const auto &stateSlack = node.stateIneqConstraints.f;
    const auto &mixedSlack = node.stateInputIneqConstraints.f;
    const auto fromTranscription =
        ipm::computePerformanceIndex(node, dt, mu, stateSlack, mixedSlack);
    Metrics metrics;
    metrics.cost = 0.0;
    metrics.dynamicsViolation = vector_t::Zero(1);
    metrics.stateIneqConstraint = {stateSlack};
    metrics.stateInputIneqConstraint = {mixedSlack};
    const auto fromMetrics =
        ipm::toPerformanceIndex(metrics, dt, mu, stateSlack, mixedSlack);
    EXPECT_NEAR(fromTranscription.cost, fromMetrics.cost, 1e-14);
    return fromTranscription.cost;
  };
  const auto node = makeTranscription(point);
  auto condensed = node.cost;
  for (const auto *constraint :
       {&node.stateIneqConstraints, &node.stateInputIneqConstraints}) {
    const vector_t slack = constraint->f;
    const vector_t dual = mu * slack.cwiseInverse();
    ipm::condenseIneqConstraints(mu, slack, dual, *constraint, condensed);
  }
  Eigen::Vector2d gradient(condensed.dfdx[0], condensed.dfdu[0]);
  Eigen::Matrix2d hessian;
  hessian << condensed.dfdxx(0, 0), condensed.dfdux(0, 0),
      condensed.dfdux(0, 0), condensed.dfduu(0, 0);
  constexpr double eps = 1e-4;
  for (int i = 0; i < 2; ++i) {
    const Eigen::Vector2d ei = eps * Eigen::Vector2d::Unit(i);
    EXPECT_NEAR((merit(point + ei) - merit(point - ei)) / (2.0 * eps),
                gradient[i], 1e-8);
    for (int j = 0; j < 2; ++j) {
      const Eigen::Vector2d ej = eps * Eigen::Vector2d::Unit(j);
      const double fd = (merit(point + ei + ej) - merit(point + ei - ej) -
                         merit(point - ei + ej) + merit(point - ei - ej)) /
                        (4.0 * eps * eps);
      EXPECT_NEAR(fd, hessian(i, j), 1e-7);
    }
  }
}
