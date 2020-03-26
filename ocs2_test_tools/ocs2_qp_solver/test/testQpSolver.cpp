//
// Created by rgrandia on 26.02.20.
//

#include <gtest/gtest.h>

#include "ocs2_qp_solver/QpSolver.h"
#include "testProblemsGeneration.h"

class QpSolverTest : public testing::Test {
 protected:
  static constexpr int N_ = 5;
  static constexpr int nx_ = 3;
  static constexpr int nu_ = 2;
  static constexpr int numDecisionVariables = N_ * (nx_ + nu_) + nx_;
  static constexpr int numConstraints = (N_ + 1) * nx_;

  QpSolverTest() {
    srand(0);
    lqProblem = ocs2::qp_solver::generateRandomProblem(N_, nx_, nu_);
    x0 = ocs2::dynamic_vector_t::Random(nx_);

    cost = ocs2::qp_solver::getCostMatrices(lqProblem, numDecisionVariables);
    constraints = ocs2::qp_solver::getConstraintMatrices(lqProblem, x0, numConstraints, numDecisionVariables);
    std::tie(primalSolution, dualSolution) = ocs2::qp_solver::solveDenseQp(cost, constraints);
  }

  std::vector<ocs2::qp_solver::LinearQuadraticStage> lqProblem;
  ocs2::dynamic_vector_t x0;
  ocs2::qp_solver::ScalarFunctionQuadraticApproximation cost;
  ocs2::qp_solver::VectorFunctionLinearApproximation constraints;
  ocs2::dynamic_vector_t primalSolution;
  ocs2::dynamic_vector_t dualSolution;
};

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