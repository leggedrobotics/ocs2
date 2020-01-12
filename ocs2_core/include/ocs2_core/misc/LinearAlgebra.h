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

#pragma once

#include <Eigen/Dense>
#include <Eigen/IterativeLinearSolvers>

#include "ocs2_core/OCS2NumericTraits.h"

namespace ocs2 {
namespace LinearAlgebra {

// forward declarations
void makePsdEigenvalue(Eigen::MatrixXd& squareMatrix, double minEigenvalue);

void makePsdCholesky(Eigen::MatrixXd& A, double minEigenvalue);

void computeConstraintProjection(const Eigen::MatrixXd& D, const Eigen::MatrixXd& RinvChol, Eigen::MatrixXd& Ddagger,
                                 Eigen::MatrixXd& DdaggerT_R_Ddagger_Chol, Eigen::MatrixXd& RinvConstrainedChol);

int rank(const Eigen::MatrixXd& A);

Eigen::VectorXcd eigenvalues(const Eigen::MatrixXd& A);

/**
 * Makes the input matrix PSD using a eigenvalue decomposition.
 *
 * @tparam Derived type.
 * @param squareMatrix: The matrix to become PSD.
 * @param [in] minEigenvalue: minimum eigenvalue.
 */
template <typename Derived>
void makePsdEigenvalue(Eigen::MatrixBase<Derived>& squareMatrix, double minEigenvalue = OCS2NumericTraits<double>::limitEpsilon()) {
  Eigen::MatrixXd mat = squareMatrix;
  makePsdEigenvalue(mat, minEigenvalue);
  squareMatrix = mat;
}

/**
 * Makes the input matrix PSD based on Gershgorin circle theorem. If the input matrix is positive definite and diagonally dominant,
 * the method will not modify the matrix.
 *
 * How it works:
 * Assume that the Ri is the sum of the absolute values of the non-diagonal entries in the i-th row (Gershgorin radius).
 * This methods updates the diagonal elements as Aii = max(Aii, minEigenvalue + Ri).
 *
 * To understand this update rule note that the Gershgorin radius (Ri) of the new matrix is the same as original one.
 * Now if we use the Gershgorin circle theorem we have:
 * | lambda - max(Aii, minEigenvalue + Ri) | < Ri ==> max(Aii, minEigenvalue + Ri) - Ri < lambda < max(Aii, minEigenvalue + Ri) + Ri
 * Two cases are possible:
 * (1) Aii < minEigenvalue + Ri ==> minEigenvalue < lambda < minEigenvalue + 2 Ri
 * (2) Aii > minEigenvalue + Ri ==> minEigenvalue < Aii - Ri < lambda < Aii + Ri
 *
 * @tparam Derived type.
 * @param squareMatrix: The matrix to become PSD.
 * @param [in] minEigenvalue: minimum eigenvalue.
 */
template <typename Derived>
void makePsdGershgorin(Eigen::MatrixBase<Derived>& squareMatrix, double minEigenvalue = OCS2NumericTraits<double>::limitEpsilon()) {
  assert(squareMatrix.rows() == squareMatrix.cols());
  squareMatrix = 0.5 * (squareMatrix + squareMatrix.transpose()).eval();
  for (size_t i = 0; i < squareMatrix.rows(); i++) {
    // Gershgorin radius: since the matrix is symmetric we use column sum instead of row sum
    auto Ri = squareMatrix.col(i).cwiseAbs().sum() - std::abs(squareMatrix(i, i));
    squareMatrix(i, i) = std::max(squareMatrix(i, i), Ri + minEigenvalue);
  }
}

/**
 * Makes the input matrix PSD based on modified Cholesky decomposition.
 *
 * S P' (A + E) P S = L L'
 * where P is a permutation matrix, E is a diagonal perturbation matrix, L is unit lower triangular, and D is diagonal.
 * If A is sufficiently positive definite, then the perturbation matrix E will be zero and this method is equivalent to
 * the pivoted Cholesky algorithm. For indefinite matrices, the perturbation matrix E is computed to ensure that A + E
 * is positive definite and well conditioned.
 *
 * References : C-J. Lin and J. J. Moré, Incomplete Cholesky Factorizations with Limited memory, SIAM J. Sci. Comput.
 * 21(1), pp. 24-45, 1999
 *
 * @tparam Derived type.
 * @param A: The matrix to become PSD.
 * @param [in] minEigenvalue: minimum eigenvalue.
 */
template <typename Derived>
void makePsdCholesky(Eigen::MatrixBase<Derived>& A, double minEigenvalue = OCS2NumericTraits<double>::limitEpsilon()) {
  Eigen::MatrixXd mat = A;
  makePsdCholesky(mat, minEigenvalue);
  A = mat;
}

/**
 * Compute upper-triangular Cholesky decomposition of inv(A)
 * A = L * L^T = U^T * U
 * inv(A) = inv(L^T) * inv(L) = inv(U) * inv(U^T)
 * inv(L^T) = inv(U)
 * @tparam Derived type.
 * @param [in] A: A symmetric square positive definite matrix
 * @param [in] U: UUT of A matrix.
 * @param [out] LinvT: LT_L decomposition of inv(A), upper-triangular
 */
template <typename Derived>
void computeLinvTLinv(const Derived& A, Derived& U, Derived& LinvT) {
  // L is lower triangular, U is upper triangular --> inv(L^T) = inv(U) is upper triangular
  Eigen::LLT<Derived> lltOfA(A);
  LinvT.setIdentity();
  U = lltOfA.matrixU();
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
template <typename Derived>
int rank(const Derived& A) {
  return rank(Eigen::MatrixXd(A));
}

/**
 * Compute the eigenvalues of A
 * @param [in] A: Matrix
 * @return Vector of complex eigenvalues
 */
template <typename Derived>
Eigen::VectorXcd eigenvalues(const Derived& A) {
  return eigenvalues(Eigen::MatrixXd(A));
}

}  // namespace LinearAlgebra
}  // namespace ocs2
