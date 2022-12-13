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

#include <gtest/gtest.h>

#include <ocs2_oc/multiple_shooting/MetricsComputation.h>
#include <ocs2_oc/multiple_shooting/Transcription.h>

#include "ocs2_oc/test/circular_kinematics.h"
#include "ocs2_oc/test/testProblemsGeneration.h"

using namespace ocs2;

TEST(test_transcription_metrics, intermediate) {
  constexpr int nx = 2;
  constexpr int nu = 2;

  // optimal control problem
  OptimalControlProblem problem = createCircularKinematicsProblem("/tmp/sqp_test_generated");

  // equality constraints
  problem.equalityConstraintPtr->add("equalityConstraint", getOcs2Constraints(getRandomConstraints(nx, nu, 2)));
  problem.stateEqualityConstraintPtr->add("stateEqualityConstraint", getOcs2StateOnlyConstraints(getRandomConstraints(nx, 0, 1)));
  // inequality constraints
  problem.inequalityConstraintPtr->add("inequalityConstraint", getOcs2Constraints(getRandomConstraints(nx, nu, 3)));
  problem.stateInequalityConstraintPtr->add("stateInequalityConstraint", getOcs2StateOnlyConstraints(getRandomConstraints(nx, 0, 4)));

  auto discretizer = selectDynamicsDiscretization(SensitivityIntegratorType::RK4);
  auto sensitivityDiscretizer = selectDynamicsSensitivityDiscretization(SensitivityIntegratorType::RK4);

  const scalar_t t = 0.5;
  const scalar_t dt = 0.1;
  const vector_t x = (vector_t(nx) << 1.0, 0.1).finished();
  const vector_t x_next = (vector_t(nx) << 1.1, 0.2).finished();
  const vector_t u = (vector_t(nu) << 0.1, 1.3).finished();
  const auto transcription = multiple_shooting::setupIntermediateNode(problem, sensitivityDiscretizer, t, dt, x, x_next, u);
  const auto metrics = multiple_shooting::computeIntermediateMetrics(problem, discretizer, t, dt, x, x_next, u);

  ASSERT_TRUE(metrics.isApprox(multiple_shooting::computeMetrics(transcription), 1e-12));
}

TEST(test_transcription_metrics, event) {
  constexpr int nx = 2;

  // optimal control problem
  OptimalControlProblem problem;

  // dynamics
  const auto dynamics = getRandomDynamics(nx, 0);
  const auto jumpMap = matrix_t::Random(nx, nx);
  problem.dynamicsPtr.reset(new LinearSystemDynamics(dynamics.dfdx, dynamics.dfdu, jumpMap));

  // cost
  problem.preJumpCostPtr->add("eventCost", getOcs2StateCost(getRandomCost(nx, 0)));

  // constraints
  problem.preJumpEqualityConstraintPtr->add("preJumpEqualityConstraint", getOcs2StateOnlyConstraints(getRandomConstraints(nx, 0, 3)));
  problem.preJumpInequalityConstraintPtr->add("preJumpInequalityConstraint", getOcs2StateOnlyConstraints(getRandomConstraints(nx, 0, 4)));

  const TargetTrajectories targetTrajectories({0.0}, {vector_t::Random(nx)}, {vector_t::Random(0)});
  problem.targetTrajectoriesPtr = &targetTrajectories;

  const scalar_t t = 0.5;
  const vector_t x = (vector_t(nx) << 1.0, 0.1).finished();
  const vector_t x_next = (vector_t(nx) << 1.1, 0.2).finished();
  const auto transcription = multiple_shooting::setupEventNode(problem, t, x, x_next);
  const auto metrics = multiple_shooting::computeEventMetrics(problem, t, x, x_next);

  ASSERT_TRUE(metrics.isApprox(multiple_shooting::computeMetrics(transcription), 1e-12));
}

TEST(test_transcription_metrics, terminal) {
  constexpr int nx = 3;

  // optimal control problem
  OptimalControlProblem problem;

  // cost
  problem.finalCostPtr->add("finalCost", getOcs2StateCost(getRandomCost(nx, 0)));
  problem.finalSoftConstraintPtr->add("finalSoftCost", getOcs2StateCost(getRandomCost(nx, 0)));

  // constraints
  problem.finalEqualityConstraintPtr->add("finalEqualityConstraint", getOcs2StateOnlyConstraints(getRandomConstraints(nx, 0, 3)));
  problem.finalInequalityConstraintPtr->add("finalInequalityConstraint", getOcs2StateOnlyConstraints(getRandomConstraints(nx, 0, 4)));

  const TargetTrajectories targetTrajectories({0.0}, {vector_t::Random(nx)}, {vector_t::Random(0)});
  problem.targetTrajectoriesPtr = &targetTrajectories;

  const scalar_t t = 0.5;
  const vector_t x = vector_t::Random(nx);
  const auto transcription = multiple_shooting::setupTerminalNode(problem, t, x);
  const auto metrics = multiple_shooting::computeTerminalMetrics(problem, t, x);

  ASSERT_TRUE(metrics.isApprox(multiple_shooting::computeMetrics(transcription), 1e-12));
}
