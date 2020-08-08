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
#include "ocs2_qp_solver/QpTrajectories.h"

#include <ocs2_core/cost/QuadraticCostFunction.h>
#include <ocs2_core/dynamics/LinearSystemDynamics.h>

namespace ocs2 {
namespace qp_solver {

/** Get random positive definite costs of n states and m inputs */
inline ScalarFunctionQuadraticApproximation getRandomCost(int n, int m) {
  matrix_t QPPR = matrix_t::Random(n + m, n + m);
  QPPR = QPPR.transpose() * QPPR;
  ScalarFunctionQuadraticApproximation cost;
  cost.dfdxx = QPPR.topLeftCorner(n, n);
  cost.dfdux = QPPR.bottomLeftCorner(m, n);
  cost.dfduu = QPPR.bottomRightCorner(m, m);
  cost.dfdx = vector_t::Random(n);
  cost.dfdu = vector_t::Random(m);
  cost.f = std::rand() / static_cast<scalar_t>(RAND_MAX);
  return cost;
}

inline std::unique_ptr<ocs2::QuadraticCostFunction> getOcs2Cost(const ScalarFunctionQuadraticApproximation& cost,
                                                                const ScalarFunctionQuadraticApproximation& costFinal) {
  return std::unique_ptr<ocs2::QuadraticCostFunction>(new ocs2::QuadraticCostFunction(cost.dfdxx, cost.dfduu, costFinal.dfdxx, cost.dfdux));
}

/** Get random linear dynamics of n states and m inputs */
inline VectorFunctionLinearApproximation getRandomDynamics(int n, int m) {
  VectorFunctionLinearApproximation dynamics;
  dynamics.dfdx = matrix_t::Random(n, n);
  dynamics.dfdu = matrix_t::Random(n, m);
  dynamics.f = vector_t::Random(n);
  return dynamics;
}

inline std::unique_ptr<ocs2::LinearSystemDynamics> getOcs2Dynamics(const VectorFunctionLinearApproximation& dynamics) {
  return std::unique_ptr<ocs2::LinearSystemDynamics>(new ocs2::LinearSystemDynamics(dynamics.dfdx, dynamics.dfdu));
}

inline ContinuousTrajectory getRandomTrajectory(int N, int n, int m, scalar_t dt) {
  ContinuousTrajectory trajectory = {
      .timeTrajectory = scalar_array_t(N + 1), .stateTrajectory = vector_array_t(N + 1), .inputTrajectory = vector_array_t(N)};
  auto t = -dt;
  std::generate(trajectory.timeTrajectory.begin(), trajectory.timeTrajectory.end(), [&t, dt]() {
    t += dt;
    return t;
  });
  std::generate(trajectory.stateTrajectory.begin(), trajectory.stateTrajectory.end(), [n]() { return vector_t::Random(n); });
  std::generate(trajectory.inputTrajectory.begin(), trajectory.inputTrajectory.end(), [m]() { return vector_t::Random(m); });
  return trajectory;
}

inline std::vector<LinearQuadraticStage> generateRandomLqProblem(int N, int nx, int nu) {
  std::vector<LinearQuadraticStage> lqProblem;
  lqProblem.reserve(N);

  for (int k = 0; k < N; ++k) {
    lqProblem.emplace_back(getRandomCost(nx, nu), getRandomDynamics(nx, nu));
  }

  // Terminal Cost
  lqProblem.emplace_back(getRandomCost(nx, 0), VectorFunctionLinearApproximation());

  return lqProblem;
}

/**
 * Compares to Eigen vectors on equality.
 * @param tol : tolerance (default value is 1e-12, which is the default of isApprox().
 */
inline bool isEqual(const vector_t& lhs, const vector_t& rhs, scalar_t tol = Eigen::NumTraits<scalar_t>::dummy_precision()) {
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
inline bool isEqual(const vector_array_t& v0, const vector_array_t& v1, scalar_t tol = Eigen::NumTraits<scalar_t>::dummy_precision()) {
  return (v0.size() == v1.size()) &&
         std::equal(v0.begin(), v0.end(), v1.begin(), [tol](const vector_t& lhs, const vector_t& rhs) { return isEqual(lhs, rhs, tol); });
}

}  // namespace qp_solver
}  // namespace ocs2
