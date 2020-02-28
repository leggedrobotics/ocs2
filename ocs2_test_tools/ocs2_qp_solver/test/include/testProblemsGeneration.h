//
// Created by rgrandia on 26.02.20.
//

#pragma once

#include "ocs2_qp_solver/QpSolverTypes.h"
#include "ocs2_qp_solver/QpTrajectories.h"

#include <ocs2_core/cost/QuadraticCostFunction.h>
#include <ocs2_core/dynamics/LinearSystemDynamics.h>

namespace ocs2_qp_solver {

/** Get random positive definite costs of n states and m inputs */
inline QuadraticCost getRandomCost(int n, int m) {
  Eigen::MatrixXd QPPR = Eigen::MatrixXd::Random(n + m, n + m);
  QPPR = QPPR.transpose() * QPPR;
  QuadraticCost cost;
  cost.Q = QPPR.topLeftCorner(n, n);
  cost.P = QPPR.bottomLeftCorner(m, n);
  cost.R = QPPR.bottomRightCorner(m, m);
  cost.q = Eigen::VectorXd::Random(n);
  cost.r = Eigen::VectorXd::Random(m);
  cost.c = std::rand() / static_cast<double>(RAND_MAX);
  return cost;
}

template <size_t STATE_DIM, size_t INPUT_DIM>
std::unique_ptr<ocs2::QuadraticCostFunction<STATE_DIM, INPUT_DIM>> getOcs2Cost(const QuadraticCost& costMat,
                                                                               const QuadraticCost& costFinalMat,
                                                                               const Eigen::VectorXd& xNominalIntermediate,
                                                                               const Eigen::VectorXd& uNominalIntermediate,
                                                                               const Eigen::VectorXd& xNominalFinal) {
  return std::unique_ptr<ocs2::QuadraticCostFunction<STATE_DIM, INPUT_DIM>>(new ocs2::QuadraticCostFunction<STATE_DIM, INPUT_DIM>(
      costMat.Q, costMat.R, xNominalIntermediate, uNominalIntermediate, costFinalMat.Q, xNominalFinal, costMat.P));
}

/** Get random linear dynamics of n states and m inputs */
inline LinearDynamics getRandomDynamics(int n, int m) {
  LinearDynamics dynamics;
  dynamics.A = Eigen::MatrixXd::Random(n, n);
  dynamics.B = Eigen::MatrixXd::Random(n, m);
  dynamics.b = Eigen::VectorXd::Random(n);
  return dynamics;
}

template <size_t STATE_DIM, size_t INPUT_DIM>
std::unique_ptr<ocs2::LinearSystemDynamics<STATE_DIM, INPUT_DIM>> getOcs2Dynamics(const LinearDynamics& dynamics) {
  return std::unique_ptr<ocs2::LinearSystemDynamics<STATE_DIM, INPUT_DIM>>(
      new ocs2::LinearSystemDynamics<STATE_DIM, INPUT_DIM>(dynamics.A, dynamics.B));
}

inline ContinuousTrajectory getRandomTrajectory(int N, int n, int m) {
  ContinuousTrajectory trajectory;
  double t = 0.0;
  for (int k = 0; k < N; ++k) {
    t += std::rand() / static_cast<double>(RAND_MAX);
    trajectory.timeTrajectory.push_back(t);  // some nonconstant time spacing
    trajectory.inputTrajectory.emplace_back(Eigen::VectorXd::Random(m));
    trajectory.stateTrajectory.emplace_back(Eigen::VectorXd::Random(n));
  }
  // Terminal
  t += std::rand() / static_cast<double>(RAND_MAX);
  trajectory.timeTrajectory.push_back(t);
  trajectory.stateTrajectory.emplace_back(Eigen::VectorXd::Random(n));
  return trajectory;
}

inline std::vector<LinearQuadraticStage> generateRandomProblem(const ProblemDimensions& problemDimensions) {
  const auto N = problemDimensions.numStages;
  std::vector<LinearQuadraticStage> lqProblem;
  lqProblem.reserve(N);

  for (int k = 0; k < N; ++k) {
    const auto nx_k = problemDimensions.numStates[k];
    const auto nu_k = problemDimensions.numInputs[k];
    lqProblem.emplace_back(getRandomCost(nx_k, nu_k), getRandomDynamics(nx_k, nu_k));
  }

  // Terminal Cost
  LinearQuadraticStage lq_stage;
  const auto nx_N = problemDimensions.numStates[N];
  lqProblem.emplace_back(getRandomCost(nx_N, 0), LinearDynamics());

  return lqProblem;
}

/**
 * Compares to Eigen vectors on equality.
 * @param tol : tolerance (default value is 1e-12, which is the default of isApprox().
 */
inline bool isEqual(const Eigen::VectorXd& lhs, const Eigen::VectorXd& rhs, double tol = Eigen::NumTraits<double>::dummy_precision()) {
  if (lhs.norm() > tol && rhs.norm() > tol) {
    return lhs.isApprox(rhs, tol);
  } else {
    return (lhs - rhs).norm() < tol;
  }
}

/**
 * Compares two trajectories on element-wise approximate equality
 * @param tol : tolerance (default value is 1e-12, which is the default of isApprox().
 * @return Vectors are of equal length and equal values.
 */
inline bool isEqual(const std::vector<Eigen::VectorXd>& v0, const std::vector<Eigen::VectorXd>& v1,
                    double tol = Eigen::NumTraits<double>::dummy_precision()) {
  return (v0.size() == v1.size()) &&
         std::equal(v0.begin(), v0.end(), v1.begin(),
                    [tol](const Eigen::VectorXd& lhs, const Eigen::VectorXd& rhs) { return isEqual(lhs, rhs, tol); });
};

/**
 * Prints trajectory as formatted string:
 * [0]  :  v[0][0], .. v[0][j]
 * [i]  :  v[i][0], .. v[i][j]
 */
inline std::string print(const std::vector<Eigen::VectorXd>& v) {
  Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "", ";");
  std::stringstream out;
  for (int k = 0; k < v.size(); k++) {
    out << "[" << k << "] \t: " << v[k].format(CommaInitFmt) << "\n";
  }
  return out.str();
};

}  // namespace ocs2_qp_solver