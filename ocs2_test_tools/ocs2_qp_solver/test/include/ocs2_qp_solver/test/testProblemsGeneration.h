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

#include <ocs2_core/constraint/LinearConstraint.h>
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

/** Get random nc linear constraints of n states, and m inputs */
inline VectorFunctionLinearApproximation getRandomConstraints(int n, int m, int nc) {
  VectorFunctionLinearApproximation constraints;
  constraints.dfdx = matrix_t::Random(nc, n);
  constraints.dfdu = matrix_t::Random(nc, m);
  constraints.f = vector_t::Random(nc);
  return constraints;
}

inline std::unique_ptr<ocs2::LinearConstraint> getOcs2Constraints(const VectorFunctionLinearApproximation& stateInputConstraints,
                                                                  const VectorFunctionLinearApproximation& stateOnlyConstraints,
                                                                  const VectorFunctionLinearApproximation& finalStateOnlyConstraints) {
  return std::unique_ptr<ocs2::LinearConstraint>(
      new ocs2::LinearConstraint(stateInputConstraints.f, stateInputConstraints.dfdx, stateInputConstraints.dfdu, stateOnlyConstraints.f,
                                 stateOnlyConstraints.dfdx, finalStateOnlyConstraints.f, finalStateOnlyConstraints.dfdx));
}

inline ContinuousTrajectory getRandomTrajectory(int N, int n, int m, scalar_t dt) {
  ContinuousTrajectory trajectory;
  trajectory.timeTrajectory = scalar_array_t(N + 1);
  trajectory.stateTrajectory = vector_array_t(N + 1);
  trajectory.inputTrajectory = vector_array_t(N);
  auto t = -dt;
  std::generate(trajectory.timeTrajectory.begin(), trajectory.timeTrajectory.end(), [&t, dt]() {
    t += dt;
    return t;
  });
  std::generate(trajectory.stateTrajectory.begin(), trajectory.stateTrajectory.end(), [n]() { return vector_t::Random(n); });
  std::generate(trajectory.inputTrajectory.begin(), trajectory.inputTrajectory.end(), [m]() { return vector_t::Random(m); });
  return trajectory;
}

inline std::vector<LinearQuadraticStage> generateRandomLqProblem(int N, int nx, int nu, int nc) {
  std::vector<LinearQuadraticStage> lqProblem;
  lqProblem.reserve(N + 1);

  for (int k = 0; k < N; ++k) {
    lqProblem.emplace_back(getRandomCost(nx, nu), getRandomDynamics(nx, nu), getRandomConstraints(nx, nu, nc));
  }

  // Terminal Cost
  lqProblem.emplace_back(getRandomCost(nx, 0), VectorFunctionLinearApproximation(), getRandomConstraints(nx, nu, nc));

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
 * Compares to Eigen matrices on equality.
 * @param tol : tolerance (default value is 1e-12, which is the default of isApprox().
 */
inline bool isEqual(const matrix_t& lhs, const matrix_t& rhs, scalar_t tol = Eigen::NumTraits<scalar_t>::dummy_precision()) {
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

/**
 * Compares two trajectories on element-wise approximate equality
 * @param tol : tolerance (default value is 1e-12, which is the default of isApprox().
 * @return Matrices are of equal length and equal values.
 */
inline bool isEqual(const matrix_array_t& v0, const matrix_array_t& v1, scalar_t tol = Eigen::NumTraits<scalar_t>::dummy_precision()) {
  return (v0.size() == v1.size()) &&
         std::equal(v0.begin(), v0.end(), v1.begin(), [tol](const matrix_t& lhs, const matrix_t& rhs) { return isEqual(lhs, rhs, tol); });
}

/** Checks QP feasibility and numerical conditioning */
inline bool isQpFeasible(const ocs2::ScalarFunctionQuadraticApproximation& qpCost,
                         const ocs2::VectorFunctionLinearApproximation& qpConstraints) {
  const auto& H = qpCost.dfdxx;
  const auto& A = qpConstraints.dfdx;

  // Cost must be convex
  Eigen::LDLT<ocs2::matrix_t> ldlt(H);
  if (!(H.ldlt().vectorD().array() > 0.0).all()) {
    std::cerr << "H is not positive definite\n";
    return false;
  }

  // Constraints feasibility
  Eigen::JacobiSVD<ocs2::matrix_t> svd(A);
  const auto conditionNumber = svd.singularValues()(0) / svd.singularValues().tail(1)(0);
  if (svd.rank() != A.rows()) {
    std::cerr << "A is not full row-rank\n";
    return false;
  } else if (conditionNumber > 1e6) {
    std::cerr << "A is ill-conditioned\n";
    return false;
  }

  return true;
}

}  // namespace qp_solver
}  // namespace ocs2
