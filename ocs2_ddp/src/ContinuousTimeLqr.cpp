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

#include "ocs2_ddp/ContinuousTimeLqr.h"

#include <ocs2_core/misc/LinearAlgebra.h>
#include <ocs2_oc/approximate_model/LinearQuadraticApproximator.h>

namespace ocs2 {
namespace continuous_time_lqr {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
solution solve(OptimalControlProblem& problem, scalar_t time, const vector_t& state, const vector_t& input, const Settings& settings) {
  // OCP check
  if (!problem.equalityLagrangianPtr->empty() || !problem.stateEqualityLagrangianPtr->empty()) {
    throw std::runtime_error("[getLinearQuadraticApproximation] equalityLagrangianPtr and stateEqualityLagrangianPtr should be empty!");
  }
  if (!problem.inequalityLagrangianPtr->empty() || !problem.stateInequalityLagrangianPtr->empty()) {
    throw std::runtime_error("[getLinearQuadraticApproximation] inequalityLagrangianPtr and stateInequalityLagrangianPtr should be empty!");
  }

  const size_t stateDim = state.size();

  // --- Form the Linear quadratic approximation ---
  // Obtain model data at the provided reference
  const auto modelData = approximateIntermediateLQ(problem, time, state, input, MultiplierCollection());

  // checking the numerical properties
  if (settings.checkNumericalCharacteristics) {
    const std::string err = checkDynamicsProperties(modelData) + checkCostProperties(modelData) + checkConstraintProperties(modelData);
    if (!err.empty()) {
      throw std::runtime_error("[continuous_time_lqr::solve] Ill-posed problem at intermediate time: " + std::to_string(time) + "\n" + err);
    }
  }

  // --- Contstruct hamiltonian ---
  // Compute terms containing inv(R)
  matrix_t RinvU;
  LinearAlgebra::computeInverseMatrixUUT(modelData.cost.dfduu, RinvU);  // Rinv = RinvU * RinvU.transpose()
  const matrix_t B_RinvU = modelData.dynamics.dfdu * RinvU;
  matrix_t PT_RinvU = modelData.cost.dfdux.transpose() * RinvU;
  const matrix_t B_Rinv_BT = B_RinvU * B_RinvU.transpose();
  const matrix_t B_Rinv_P = B_RinvU * PT_RinvU.transpose();
  const matrix_t PT_Rinv_P = PT_RinvU * PT_RinvU.transpose();

  // Correct for state-input matrix
  const matrix_t A = modelData.dynamics.dfdx - B_Rinv_P;
  const matrix_t Q = modelData.cost.dfdxx - PT_Rinv_P;

  // The permuted Hamiltonian is the initial point of the iterative algorithm
  matrix_t W = (matrix_t(2 * stateDim, 2 * stateDim) << -Q, -A.transpose(), -A, B_Rinv_BT).finished();

  // --- Solve CARE ---

  // Temporary variables used in the iteration
  matrix_t Winv(2 * stateDim, 2 * stateDim);
  matrix_t dW(2 * stateDim, 2 * stateDim);
  matrix_t JWinvJ(2 * stateDim, 2 * stateDim);

  const scalar_t exponent = 0.5 / stateDim;
  scalar_t Wnorm;
  size_t iter = 0;
  do {
    // Store scale of current iteration
    Wnorm = W.norm();

    // Prepare update: W(k+1) = 1/(2c) * (W(k) + c^2 * J *  inv(W(k)) * J)
    // W(k+1) - W(k) = c1 * W(k) + c2 *  J *  inv(W(k)) * J
    const scalar_t c = std::pow(std::abs(W.determinant()), exponent);
    const scalar_t c1 = 0.5 / c - 1.0;
    const scalar_t c2 = 0.5 * c;
    Winv = W.partialPivLu().inverse();
    JWinvJ << -Winv.bottomRightCorner(stateDim, stateDim), Winv.bottomLeftCorner(stateDim, stateDim),
        Winv.topRightCorner(stateDim, stateDim), -Winv.topLeftCorner(stateDim, stateDim);  // Implement J * Winv * J, with J = [0, I; -I 0]
    dW = c1 * W + c2 * JWinvJ;

    // Apply update
    W += dW;
    iter++;
  } while ((dW.norm() / Wnorm) > settings.tolerance && iter < settings.maxIter);

  // Solve for value function through system of equations M * S = N
  matrix_t lhsM(2 * stateDim, stateDim);
  lhsM << W.bottomRightCorner(stateDim, stateDim), W.topRightCorner(stateDim, stateDim) + matrix_t::Identity(stateDim, stateDim);
  matrix_t rhsN(2 * stateDim, stateDim);
  rhsN << matrix_t::Identity(stateDim, stateDim) - W.bottomLeftCorner(stateDim, stateDim), -W.topLeftCorner(stateDim, stateDim);
  const matrix_t S = lhsM.colPivHouseholderQr().solve(rhsN);

  // Compute feedback gains
  PT_RinvU.noalias() += S.transpose() * B_RinvU;
  const matrix_t K = -RinvU * PT_RinvU.transpose();

  return {K, S};
}

}  // namespace continuous_time_lqr
}  // namespace ocs2
