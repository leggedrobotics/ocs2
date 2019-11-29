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

#ifndef OCS2_CTRL_LINEARALGEBRA_H
#define OCS2_CTRL_LINEARALGEBRA_H

#include <Eigen/Dense>

namespace ocs2 {
namespace LinearAlgebra {

/**
 * Makes the input matrix PSD.
 *
 * @tparam Derived type.
 * @param [out] squareMatrix: The matrix to become PSD.
 * @return true if the matrix had negative eigen values.
 */
bool makePSD(Eigen::MatrixXd& squareMatrix);

template <typename Derived>
bool makePSD(Eigen::MatrixBase<Derived>& squareMatrix) {
  bool hasNegativeEigenValue;

  Eigen::MatrixXd mat = squareMatrix;
  hasNegativeEigenValue = makePSD(mat);
  squareMatrix = mat;

  return hasNegativeEigenValue;
}

/**
 * Compute upper-triangular Cholesky decomposition of inv(A)
 * A = L * L^T = U^T * U
 * inv(A) = inv(L^T) * inv(L) = inv(U) * inv(U^T)
 * inv(L^T) = inv(U)
 * @tparam Derived type.
 * @param [in] A: A symmetric square positive definite matrix
 * @param [out] LinvT: LT_L decomposition of inv(A), upper-triangular
 */
template <typename Derived>
void computeLinvTLinv(const Derived& A, Derived& LinvT) {
  // L is lower triangular, U is upper triangular --> inv(L^T) = inv(U) is upper triangular
  Eigen::LLT<Derived> lltOfA(A);
  LinvT.setIdentity();
  lltOfA.matrixU().solveInPlace(LinvT);
}

/**
 * Computes constraint projection for linear constraints  C*x + D*u - e = 0,
 * The input cost matrix R should be already inverted and decomposed such that inv(R) = RinvChol * RinvChol^T
 * @param [in] D: A full row rank constraint matrix
 * @param [in] RinvChol: Cholesky decomposition of inv(R)
 * @param [out] Ddagger: Weighted pseudo inverse of D, Ddagger = inv(R)*D' * inv(D*inv(R)*D')
 * @param [out] DdaggerT_R_Ddagger_Chol: Cholesky decomposition of DdaggerT_R_Ddagger
 * @param [out] RinvConstrainedChol: Decomposition of inv(R)^T * (I-Ddagger*D)^T * R * (I-Ddagger*D) * inv(R)
 */
void computeConstraintProjection(const Eigen::MatrixXd& D, const Eigen::MatrixXd& RinvChol, Eigen::MatrixXd& Ddagger,
                                 Eigen::MatrixXd& DdaggerT_R_Ddagger_Chol, Eigen::MatrixXd& RinvConstrainedChol);

template <typename DerivedInputMatrix>
void computeConstraintProjection(const Eigen::MatrixXd& D, const DerivedInputMatrix& RinvChol, Eigen::MatrixXd& Ddagger,
                                 Eigen::MatrixXd& DdaggerT_R_Ddagger_Chol, Eigen::MatrixXd& RinvConstrainedChol) {
  computeConstraintProjection(D, Eigen::MatrixXd(RinvChol), Ddagger, DdaggerT_R_Ddagger_Chol, RinvConstrainedChol);
}

/**
 * Compute the rank of a matrix
 * @param [in] A: Matrix
 * @return rank of A
 */
int rank(const Eigen::MatrixXd& A);

template <typename Derived>
int rank(const Derived& A) {
  return rank(Eigen::MatrixXd(A));
}

/**
 * Compute the eigenvalues of A
 * @param [in] A: Matrix
 * @return Vector of complex eigenvalues
 */
Eigen::VectorXcd eigenvalues(const Eigen::MatrixXd& A);

template <typename Derived>
Eigen::VectorXcd eigenvalues(const Derived& A) {
  return eigenvalues(Eigen::MatrixXd(A));
}

}  // namespace LinearAlgebra
}  // namespace ocs2

#endif  // OCS2_CTRL_LINEARALGEBRA_H
