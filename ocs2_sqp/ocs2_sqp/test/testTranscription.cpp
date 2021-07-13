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

#include "ocs2_sqp/MultipleShootingTranscription.h"

#include <ocs2_oc/test/circular_kinematics.h>
#include <ocs2_qp_solver/test/testProblemsGeneration.h>

namespace {
/** Helper to compare if two performance indices are identical */
bool areIdentical(const ocs2::PerformanceIndex& lhs, const ocs2::PerformanceIndex& rhs) {
  return lhs.merit == rhs.merit && lhs.totalCost == rhs.totalCost && lhs.stateEqConstraintISE == rhs.stateEqConstraintISE &&
         lhs.stateEqFinalConstraintSSE == rhs.stateEqFinalConstraintSSE && lhs.stateInputEqConstraintISE == rhs.stateInputEqConstraintISE &&
         lhs.inequalityConstraintISE == rhs.inequalityConstraintISE && lhs.inequalityConstraintPenalty == rhs.inequalityConstraintPenalty;
}
}  // namespace

using namespace ocs2;
using namespace ocs2::multiple_shooting;

TEST(test_transcription, intermediate_performance) {
  ocs2::CircularKinematicsSystem system;
  ocs2::CircularKinematicsCost cost;
  ocs2::CircularKinematicsConstraints constraint;
  auto discretizer = selectDynamicsDiscretization(SensitivityIntegratorType::RK4);
  auto sensitivityDiscretizer = selectDynamicsSensitivityDiscretization(SensitivityIntegratorType::RK4);

  scalar_t t = 0.5;
  scalar_t dt = 0.1;
  const ocs2::vector_t x = (ocs2::vector_t(2) << 1.0, 0.1).finished();
  const ocs2::vector_t x_next = (ocs2::vector_t(2) << 1.1, 0.2).finished();
  const ocs2::vector_t u = (ocs2::vector_t(2) << 0.1, 1.3).finished();
  const auto transcription = setupIntermediateNode(system, sensitivityDiscretizer, cost, &constraint, nullptr, true, t, dt, x, x_next, u);

  const auto performance = computeIntermediatePerformance(system, discretizer, cost, &constraint, nullptr, t, dt, x, x_next, u);

  ASSERT_TRUE(areIdentical(performance, transcription.performance));
}

TEST(test_transcription, terminal_performance) {
  int nx = 3;
  int nu = 2;

  // cost
  auto costPtr = ocs2::qp_solver::getOcs2Cost(ocs2::qp_solver::getRandomCost(nx, nu), ocs2::qp_solver::getRandomCost(nx, nu));
  const ocs2::TargetTrajectories targetTrajectories({0.0}, {ocs2::vector_t::Random(nx)}, {ocs2::vector_t::Random(nu)});
  costPtr->setTargetTrajectoriesPtr(&targetTrajectories);

  scalar_t t = 0.5;
  const ocs2::vector_t x = ocs2::vector_t::Random(nx);
  const auto transcription = setupTerminalNode(costPtr.get(), nullptr, t, x);
  const auto performance = computeTerminalPerformance(costPtr.get(), nullptr, t, x);

  ASSERT_TRUE(areIdentical(performance, transcription.performance));
}

TEST(test_transcription, event_performance) {
  ocs2::CircularKinematicsSystem system;

  const scalar_t t = 0.5;
  const ocs2::vector_t x = (ocs2::vector_t(2) << 1.0, 0.1).finished();
  const ocs2::vector_t x_next = (ocs2::vector_t(2) << 1.1, 0.2).finished();
  const auto transcription = setupEventNode(system, nullptr, nullptr, t, x, x_next);
  const auto performance = computeEventPerformance(system, nullptr, nullptr, t, x, x_next);

  ASSERT_TRUE(areIdentical(performance, transcription.performance));
}
