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

#include <ocs2_core/constraint/LinearStateConstraint.h>
#include <ocs2_core/constraint/LinearStateInputConstraint.h>
#include <ocs2_core/cost/QuadraticStateCost.h>
#include <ocs2_core/cost/QuadraticStateInputCost.h>
#include <ocs2_core/dynamics/LinearSystemDynamics.h>

namespace ocs2 {

/** Get random positive definite costs of n states and m inputs */
inline ScalarFunctionQuadraticApproximation getRandomCost(int n, int m) {
  matrix_t QPPR = matrix_t::Random(n + m, n + m);
  QPPR = QPPR.transpose() * QPPR;
  ScalarFunctionQuadraticApproximation cost;
  cost.dfdxx = QPPR.topLeftCorner(n, n);
  cost.dfdx = vector_t::Random(n);
  if (m >= 0) {
    cost.dfdux = QPPR.bottomLeftCorner(m, n);
    cost.dfduu = QPPR.bottomRightCorner(m, m);
    cost.dfdu = vector_t::Random(m);
  }
  cost.f = std::rand() / static_cast<scalar_t>(RAND_MAX);
  return cost;
}

inline std::unique_ptr<ocs2::StateInputCost> getOcs2Cost(const ScalarFunctionQuadraticApproximation& cost) {
  return std::make_unique<ocs2::QuadraticStateInputCost>(cost.dfdxx, cost.dfduu, cost.dfdux);
}

inline std::unique_ptr<ocs2::StateCost> getOcs2StateCost(const ScalarFunctionQuadraticApproximation& costFinal) {
  return std::make_unique<ocs2::QuadraticStateCost>(costFinal.dfdxx);
}

/** Get random linear dynamics of n states and m inputs */
inline VectorFunctionLinearApproximation getRandomDynamics(int n, int m) {
  VectorFunctionLinearApproximation dynamics;
  dynamics.dfdx = matrix_t::Random(n, n);
  if (m >= 0) {
    dynamics.dfdu = matrix_t::Random(n, m);
  }
  dynamics.f = vector_t::Random(n);
  return dynamics;
}

inline std::unique_ptr<ocs2::LinearSystemDynamics> getOcs2Dynamics(const VectorFunctionLinearApproximation& dynamics) {
  return std::make_unique<ocs2::LinearSystemDynamics>(dynamics.dfdx, dynamics.dfdu);
}

/** Get random nc linear constraints of n states, and m inputs */
inline VectorFunctionLinearApproximation getRandomConstraints(int n, int m, int nc) {
  VectorFunctionLinearApproximation constraints;
  constraints.dfdx = matrix_t::Random(nc, n);
  if (m >= 0) {
    constraints.dfdu = matrix_t::Random(nc, m);
  }
  constraints.f = vector_t::Random(nc);
  return constraints;
}

inline std::unique_ptr<ocs2::StateInputConstraint> getOcs2Constraints(const VectorFunctionLinearApproximation& stateInputConstraints) {
  return std::make_unique<ocs2::LinearStateInputConstraint>(stateInputConstraints.f, stateInputConstraints.dfdx,
                                                            stateInputConstraints.dfdu);
}

inline std::unique_ptr<ocs2::StateConstraint> getOcs2StateOnlyConstraints(const VectorFunctionLinearApproximation& stateOnlyConstraints) {
  return std::make_unique<ocs2::LinearStateConstraint>(stateOnlyConstraints.f, stateOnlyConstraints.dfdx);
}

}  // namespace ocs2
