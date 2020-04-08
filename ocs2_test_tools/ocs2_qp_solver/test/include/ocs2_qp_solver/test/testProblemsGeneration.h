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
  dynamic_matrix_t QPPR = dynamic_matrix_t::Random(n + m, n + m);
  QPPR = QPPR.transpose() * QPPR;
  ScalarFunctionQuadraticApproximation cost;
  cost.dfdxx = QPPR.topLeftCorner(n, n);
  cost.dfdux = QPPR.bottomLeftCorner(m, n);
  cost.dfduu = QPPR.bottomRightCorner(m, m);
  cost.dfdx = dynamic_vector_t::Random(n);
  cost.dfdu = dynamic_vector_t::Random(m);
  cost.f = std::rand() / static_cast<scalar_t>(RAND_MAX);
  return cost;
}

template <size_t STATE_DIM, size_t INPUT_DIM>
std::unique_ptr<ocs2::QuadraticCostFunction<STATE_DIM, INPUT_DIM>> getOcs2Cost(const ScalarFunctionQuadraticApproximation& cost,
                                                                               const ScalarFunctionQuadraticApproximation& costFinal,
                                                                               const dynamic_vector_t& xNominalIntermediate,
                                                                               const dynamic_vector_t& uNominalIntermediate,
                                                                               const dynamic_vector_t& xNominalFinal) {
  return std::unique_ptr<ocs2::QuadraticCostFunction<STATE_DIM, INPUT_DIM>>(new ocs2::QuadraticCostFunction<STATE_DIM, INPUT_DIM>(
      cost.dfdxx, cost.dfduu, xNominalIntermediate, uNominalIntermediate, costFinal.dfdxx, xNominalFinal, cost.dfdux));
}

/** Get random linear dynamics of n states and m inputs */
inline VectorFunctionLinearApproximation getRandomDynamics(int n, int m) {
  VectorFunctionLinearApproximation dynamics;
  dynamics.dfdx = dynamic_matrix_t::Random(n, n);
  dynamics.dfdu = dynamic_matrix_t::Random(n, m);
  dynamics.f = dynamic_vector_t::Random(n);
  return dynamics;
}

template <size_t STATE_DIM, size_t INPUT_DIM>
std::unique_ptr<ocs2::LinearSystemDynamics<STATE_DIM, INPUT_DIM>> getOcs2Dynamics(const VectorFunctionLinearApproximation& dynamics) {
  return std::unique_ptr<ocs2::LinearSystemDynamics<STATE_DIM, INPUT_DIM>>(
      new ocs2::LinearSystemDynamics<STATE_DIM, INPUT_DIM>(dynamics.dfdx, dynamics.dfdu));
}

/** Get random nc linear constraints of n states, and m inputs */
inline VectorFunctionLinearApproximation getRandomConstraints(int n, int m, int nc) {
  VectorFunctionLinearApproximation constraints;
  constraints.dfdx = dynamic_matrix_t::Random(nc, n);
  constraints.dfdu = dynamic_matrix_t::Random(nc, m);
  constraints.f = dynamic_vector_t::Random(nc);
  return constraints;
}

template <size_t STATE_DIM, size_t INPUT_DIM>
std::unique_ptr<ocs2::LinearConstraint<STATE_DIM, INPUT_DIM>> getOcs2Constraints(
    const VectorFunctionLinearApproximation& stateInputConstraints, const VectorFunctionLinearApproximation& stateOnlyConstraints,
    const VectorFunctionLinearApproximation& finalStateOnlyConstraints) {
  using constraint_t = ocs2::LinearConstraint<STATE_DIM, INPUT_DIM>;

  const auto numStateInputConstraint = stateInputConstraints.f.size();
  typename constraint_t::constraint1_vector_t e;
  e.head(numStateInputConstraint) = stateInputConstraints.f;
  typename constraint_t::constraint1_state_matrix_t C;
  C.topRows(numStateInputConstraint) = stateInputConstraints.dfdx;
  typename constraint_t::constraint1_input_matrix_t D;
  D.topRows(numStateInputConstraint) = stateInputConstraints.dfdu;

  const auto numStateOnlyConstraint = stateOnlyConstraints.f.size();
  typename constraint_t::constraint2_vector_t h;
  h.head(numStateOnlyConstraint) = stateOnlyConstraints.f;
  typename constraint_t::constraint2_state_matrix_t F;
  F.topRows(numStateOnlyConstraint) = stateOnlyConstraints.dfdx;

  const auto numStateOnlyFinalConstraint = finalStateOnlyConstraints.f.size();
  typename constraint_t::constraint2_vector_t h_f;
  h_f.head(numStateOnlyFinalConstraint) = finalStateOnlyConstraints.f;
  typename constraint_t::constraint2_state_matrix_t F_f;
  F_f.topRows(numStateOnlyFinalConstraint) = finalStateOnlyConstraints.dfdx;

  return std::unique_ptr<constraint_t>(
      new constraint_t(numStateInputConstraint, e, C, D, numStateOnlyConstraint, h, F, numStateOnlyFinalConstraint, h_f, F_f));
}

inline ContinuousTrajectory getRandomTrajectory(int N, int n, int m, scalar_t dt) {
  ContinuousTrajectory trajectory = {.timeTrajectory = scalar_array_t(N + 1),
                                     .stateTrajectory = dynamic_vector_array_t(N + 1),
                                     .inputTrajectory = dynamic_vector_array_t(N)};
  auto t = -dt;
  std::generate(trajectory.timeTrajectory.begin(), trajectory.timeTrajectory.end(), [&t, dt]() {
    t += dt;
    return t;
  });
  std::generate(trajectory.stateTrajectory.begin(), trajectory.stateTrajectory.end(), [n]() { return dynamic_vector_t::Random(n); });
  std::generate(trajectory.inputTrajectory.begin(), trajectory.inputTrajectory.end(), [m]() { return dynamic_vector_t::Random(m); });
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
inline bool isEqual(const dynamic_vector_t& lhs, const dynamic_vector_t& rhs,
                    scalar_t tol = Eigen::NumTraits<scalar_t>::dummy_precision()) {
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
inline bool isEqual(const dynamic_vector_array_t& v0, const dynamic_vector_array_t& v1,
                    scalar_t tol = Eigen::NumTraits<scalar_t>::dummy_precision()) {
  return (v0.size() == v1.size()) &&
         std::equal(v0.begin(), v0.end(), v1.begin(),
                    [tol](const dynamic_vector_t& lhs, const dynamic_vector_t& rhs) { return isEqual(lhs, rhs, tol); });
}

}  // namespace qp_solver
}  // namespace ocs2
