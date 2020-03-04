//
// Created by rgrandia on 26.02.20.
//

#pragma once

#include "ocs2_qp_solver/QpSolverTypes.h"

#include <ocs2_core/cost/QuadraticCostFunction.h>
#include <ocs2_core/dynamics/LinearSystemDynamics.h>

namespace ocs2 {
namespace qp_solver {

/** Get random positive definite costs of n states and m inputs */
inline ScalarFunctionQuadraticApproximation getRandomCost(int n, int m) {
  Eigen::MatrixXd QPPR = Eigen::MatrixXd::Random(n + m, n + m);
  QPPR = QPPR.transpose() * QPPR;
  ScalarFunctionQuadraticApproximation cost;
  cost.dfdxx = QPPR.topLeftCorner(n, n);
  cost.dfdux = QPPR.bottomLeftCorner(m, n);
  cost.dfduu = QPPR.bottomRightCorner(m, m);
  cost.dfdx = Eigen::VectorXd::Random(n);
  cost.dfdu = Eigen::VectorXd::Random(m);
  cost.f = std::rand() / static_cast<double>(RAND_MAX);
  return cost;
}

template <size_t STATE_DIM, size_t INPUT_DIM>
std::unique_ptr<ocs2::QuadraticCostFunction<STATE_DIM, INPUT_DIM>> getOcs2Cost(const ScalarFunctionQuadraticApproximation& cost,
                                                                               const ScalarFunctionQuadraticApproximation& costFinal,
                                                                               const Eigen::VectorXd& xNominalIntermediate,
                                                                               const Eigen::VectorXd& uNominalIntermediate,
                                                                               const Eigen::VectorXd& xNominalFinal) {
  return std::unique_ptr<ocs2::QuadraticCostFunction<STATE_DIM, INPUT_DIM>>(new ocs2::QuadraticCostFunction<STATE_DIM, INPUT_DIM>(
      cost.dfdxx, cost.dfduu, xNominalIntermediate, uNominalIntermediate, costFinal.dfdxx, xNominalFinal, cost.dfdux));
}

/** Get random linear dynamics of n states and m inputs */
inline VectorFunctionLinearApproximation getRandomDynamics(int n, int m) {
  VectorFunctionLinearApproximation dynamics;
  dynamics.dfdx = Eigen::MatrixXd::Random(n, n);
  dynamics.dfdu = Eigen::MatrixXd::Random(n, m);
  dynamics.f = Eigen::VectorXd::Random(n);
  return dynamics;
}

template <size_t STATE_DIM, size_t INPUT_DIM>
std::unique_ptr<ocs2::LinearSystemDynamics<STATE_DIM, INPUT_DIM>> getOcs2Dynamics(const VectorFunctionLinearApproximation& dynamics) {
  return std::unique_ptr<ocs2::LinearSystemDynamics<STATE_DIM, INPUT_DIM>>(
      new ocs2::LinearSystemDynamics<STATE_DIM, INPUT_DIM>(dynamics.dfdx, dynamics.dfdu));
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
}

}  // namespace qp_solver
}  // namespace ocs2
