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

#include "ocs2_sqp/ConstraintProjection.h"

namespace ocs2 {

VectorFunctionLinearApproximation qrConstraintProjection(const VectorFunctionLinearApproximation& constraint) {
  // Constraint Projectors are based on the QR decomposition
  const auto numConstraints = constraint.dfdu.rows();
  const auto numInputs = constraint.dfdu.cols();
  const Eigen::HouseholderQR<matrix_t> QRof_DT(constraint.dfdu.transpose());

  const auto RT = QRof_DT.matrixQR().topRows(numConstraints).triangularView<Eigen::Upper>().transpose();
  const matrix_t RTinvC = RT.solve(constraint.dfdx);  // inv(R^T) * C
  const matrix_t RTinve = RT.solve(constraint.f);     // inv(R^T) * e

  const matrix_t Q = QRof_DT.householderQ();
  const auto Q1 = Q.leftCols(numConstraints);

  VectorFunctionLinearApproximation projectionTerms;
  projectionTerms.dfdu = Q.rightCols(numInputs - numConstraints);
  projectionTerms.dfdx.noalias() = -Q1 * RTinvC;
  projectionTerms.f.noalias() = -Q1 * RTinve;

  return projectionTerms;
}

VectorFunctionLinearApproximation luConstraintProjection(const VectorFunctionLinearApproximation& constraint) {
  // Constraint Projectors are based on the LU decomposition
  const Eigen::FullPivLU<matrix_t> lu(constraint.dfdu);

  VectorFunctionLinearApproximation projectionTerms;
  projectionTerms.dfdu = lu.kernel();
  projectionTerms.dfdx.noalias() = -lu.solve(constraint.dfdx);
  projectionTerms.f.noalias() = -lu.solve(constraint.f);

  return projectionTerms;
}

}  // namespace ocs2