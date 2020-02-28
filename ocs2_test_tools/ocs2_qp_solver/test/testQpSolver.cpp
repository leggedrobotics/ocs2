//
// Created by rgrandia on 26.02.20.
//

#include <gtest/gtest.h>

#include "ocs2_qp_solver/QpSolver.h"
#include "testProblemsGeneration.h"

TEST(qpSolver, lqproblem) {
  // Set up problem
  ocs2_qp_solver::ProblemDimensions problemSize(5, 3, 2);
  srand(0);
  const auto lqProblem = generateRandomProblem(problemSize);
  Eigen::VectorXd x0 = Eigen::VectorXd::Random(problemSize.numStates[0]);

  // Construct QP
  const auto Hg = ocs2_qp_solver::getCostMatrices(problemSize, lqProblem);
  const auto Ab = ocs2_qp_solver::getConstraintMatrices(problemSize, lqProblem, x0);

  // Solve QP
  const auto xl = ocs2_qp_solver::solveDenseQp(Hg, Ab);

  // Check constraint satisfaction
  const Eigen::VectorXd constraintViolation = Ab.first * xl.first - Ab.second;
  ASSERT_LE(constraintViolation.norm(), 1e-6);

  // Check first order optimality
  const Eigen::VectorXd firstOrderOptimality = Hg.first * xl.first + Hg.second + Ab.first.transpose() * xl.second;
  ASSERT_LE(firstOrderOptimality.norm(), 1e-6);
}

TEST(qpSolver, costMatrix) {
  // Set up problem
  ocs2_qp_solver::ProblemDimensions problemSize(5, 3, 2);
  srand(0);
  const auto lqProblem = generateRandomProblem(problemSize);

  // Construct cost matrix
  const auto Hg = ocs2_qp_solver::getCostMatrices(problemSize, lqProblem);
  ASSERT_TRUE(Hg.first.fullPivLu().rank() == Hg.first.rows());
}

TEST(qpSolver, constraintMatrix) {
  // Set up problem
  ocs2_qp_solver::ProblemDimensions problemSize(5, 3, 2);
  srand(0);
  const auto lqProblem = generateRandomProblem(problemSize);
  Eigen::VectorXd x0 = Eigen::VectorXd::Random(problemSize.numStates[0]);

  // Construct cost matrix
  const auto Ab = ocs2_qp_solver::getConstraintMatrices(problemSize, lqProblem, x0);
  ASSERT_TRUE(Ab.first.fullPivLu().rank() == Ab.first.rows());
}