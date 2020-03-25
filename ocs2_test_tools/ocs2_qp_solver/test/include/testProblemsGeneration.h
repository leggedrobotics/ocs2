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
