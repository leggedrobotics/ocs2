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

#include "ocs2_qp_solver/QpSolver.h"
#include "ocs2_qp_solver/test/testProblemsGeneration.h"

class QpSolverTest : public testing::Test {
 protected:
  static constexpr int N_ = 5;
  static constexpr int nx_ = 3;
  static constexpr int nu_ = 2;
  static constexpr int numDecisionVariables = N_ * (nx_ + nu_) + nx_;
  static constexpr int numConstraints = (N_ + 1) * nx_;

  QpSolverTest() {
    srand(0);
    lqProblem = ocs2::qp_solver::generateRandomLqProblem(N_, nx_, nu_);
    x0 = ocs2::vector_t::Random(nx_);

    cost = ocs2::qp_solver::getCostMatrices(lqProblem, numDecisionVariables);
    constraints = ocs2::qp_solver::getConstraintMatrices(lqProblem, x0, numConstraints, numDecisionVariables);
    std::tie(primalSolution, dualSolution) = ocs2::qp_solver::solveDenseQp(cost, constraints);
  }

  std::vector<ocs2::qp_solver::LinearQuadraticStage> lqProblem;
  ocs2::vector_t x0;
  ocs2::ScalarFunctionQuadraticApproximation cost;
  ocs2::VectorFunctionLinearApproximation constraints;
  ocs2::vector_t primalSolution;
  ocs2::vector_t dualSolution;
};

constexpr int QpSolverTest::N_;
constexpr int QpSolverTest::nx_;
constexpr int QpSolverTest::nu_;
constexpr int QpSolverTest::numDecisionVariables;
constexpr int QpSolverTest::numConstraints;

TEST_F(QpSolverTest, constraintSatisfaction) {
  ASSERT_TRUE(constraints.f.isApprox(-constraints.dfdx * primalSolution));
}

TEST_F(QpSolverTest, firstOrderOptimality) {
  ASSERT_TRUE(cost.dfdx.isApprox(-cost.dfdxx * primalSolution - constraints.dfdx.transpose() * dualSolution));
}

TEST_F(QpSolverTest, costMatrixFullRank) {
  ASSERT_TRUE(cost.dfdxx.fullPivLu().rank() == cost.dfdxx.rows());
}

TEST_F(QpSolverTest, constraintMatrixFullRank) {
  ASSERT_TRUE(constraints.dfdx.fullPivLu().rank() == constraints.dfdx.rows());
}
