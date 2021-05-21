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

#include <ocs2_core/cost/QuadraticStateInputCost.h>
#include <ocs2_core/dynamics/LinearSystemDynamics.h>
#include <ocs2_oc/oc_problem/OptimalControlProblem.h>
#include <ocs2_oc/oc_problem/ProblemData.h>

using namespace ocs2;

TEST(testOptimalControlProblem, evaluateCost) {
  using Request = ProblemData::Request;

  LinearSystemDynamics dynamics(matrix_t::Random(2, 2), matrix_t::Random(2, 1));

  QuadraticStateInputCost cost(matrix_t::Identity(2, 2), matrix_t::Identity(1, 1));
  CostDesiredTrajectories desiredTrajectory({0.0}, {vector_t::Zero(2)}, {vector_t::Zero(1)});

  OptimalControlProblem problem;
  problem.dynamics.reset(dynamics.clone());
  problem.cost.add("QuadraticCost", std::unique_ptr<StateInputCost>(cost->clone()));
  problem.costDesiredTrajectories = &desiredTrajectory;

  scalar_t t = 0.0;
  const vector_t x = vector_t::Random(2);
  const vector_t u = vector_t::Random(1);
  const auto costValue = cost.getValue(t, x, u, desiredTrajectory, PreComputation());
  const auto costApproximation = cost.getQuadraticApproximation(t, x, u, desiredTrajectory, PreComputation());

  ModelData modelData;
  LinearQuadraticApproximator lqapprox(problem);

  lqapprox.approximateLQProblem(t, x, u, modelData);
  EXPECT_FLOAT_EQ(modelData.cost_.f, costValue);
  EXPECT_FLOAT_EQ(modelData.cost_.f, costApproximation.f);
  EXPECT_TRUE(modelData.cost_.dfdx.isApprox(costApproximation.dfdx));
  EXPECT_TRUE(modelData.cost_.dfdu.isApprox(costApproximation.dfdu));
  EXPECT_TRUE(modelData.cost_.dfdxx.isApprox(costApproximation.dfdxx));
  EXPECT_TRUE(modelData.cost_.dfduu.isApprox(costApproximation.dfduu));
  EXPECT_TRUE(modelData.cost_.dfdux.isApprox(costApproximation.dfdux));

  lqapprox.approximateLQProblemAtEventTime(request, t, x, modelData);
  EXPECT_FLOAT_EQ(modelData.cost_.f, 0.0);

  lqapprox.approximateLQProblemAtFinalTime(request, t, x, modelData);
  EXPECT_FLOAT_EQ(modelData.cost_.f, 0.0);
}
