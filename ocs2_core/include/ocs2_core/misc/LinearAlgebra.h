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

#pragma once

#include <Eigen/Dense>
#include <Eigen/IterativeLinearSolvers>

#include <ocs2_core/NumericTraits.h>
#include <ocs2_core/Types.h>

namespace ocs2 {
namespace LinearAlgebra {

/**
 *  Set the eigenvalues of a triangular matrix to a minimum magnitude (maintaining the sign).
 */
void setTriangularMinimumEigenvalues(matrix_t& Lr, scalar_t minEigenValue = numeric_traits::weakEpsilon<scalar_t>());

/**
 * Makes the input matrix PSD using a eigenvalue decomposition.
 *
 * @param [in, out] squareMatrix: The matrix to become PSD.
 * @param [in] minEigenvalue: minimum eigenvalue.
 */
void makePsdEigenvalue(matrix_t& squareMatrix, scalar_t minEigenvalue = numeric_traits::limitEpsilon<scalar_t>());

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
 * @param [in, out] squareMatrix: The matrix to become PSD.
 * @param [in] minEigenvalue: minimum eigenvalue.
 */
void makePsdGershgorin(matrix_t& squareMatrix, scalar_t minEigenvalue = numeric_traits::limitEpsilon<scalar_t>());

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
 * @param [in, out] A: The matrix to become PSD.
 * @param [in] minEigenvalue: minimum eigenvalue.
 */
void makePsdCholesky(matrix_t& A, scalar_t minEigenvalue = numeric_traits::limitEpsilon<scalar_t>());

/**
 * Computes the U*U^T decomposition associated to the inverse of the input matrix, where U is an upper triangular
 * matrix. Note that the U*U^T decomposition is different from the Cholesky decomposition (U^T*U).
 *
 * @param [in] Am: A symmetric square positive definite matrix
 * @param [out] AmInvUmUmT: The upper-triangular matrix associated to the UUT decomposition of inv(Am) matrix.
 */
void computeInverseMatrixUUT(const matrix_t& Am, matrix_t& AmInvUmUmT);

/**
 * Computes constraint projection for linear constraints  C*x + D*u - e = 0, with the weighting inv(Rm)
 * The input cost matrix Rm should be already inverted and decomposed such that inv(Rm) = RmInvUmUmT * RmInvUmUmT^T.
 *
 * @param [in] Dm: A full row rank constraint matrix
 * @param [in] RmInvUmUmT: The upper-triangular matrix associated to the UUT decomposition of inv(Rm) matrix.
 * @param [out] DmDagger: The right weighted pseudo inverse of Dm, DmDagger = inv(Rm)*Dm'*inv(Dm*inv(Rm)*Dm')
 * @param [out] DmDaggerTRmDmDaggerUUT: The UUT decomposition of DmDagger^T * Rm * DmDagger = Um * Um^T
 * @param [out] RmInvConstrainedUUT: The VVT decomposition of (I-DmDagger*Dm) * inv(Rm) * (I-DmDagger*Dm)^T where V is of
 * the dimension n_u*(n_u-n_c) with n_u = Rm.rows() and n_c = Dm.rows().
 */
void computeConstraintProjection(const matrix_t& Dm, const matrix_t& RmInvUmUmT, matrix_t& DmDagger, matrix_t& DmDaggerTRmDmDaggerUUT,
                                 matrix_t& RmInvConstrainedUUT);

/**
 * Returns the linear projection
 *  u = Pu * \tilde{u} + Px * x + Pe
 *
 * s.t. C*x + D*u + e = 0 is satisfied for any \tilde{u}
 *
 * Implementation based on the QR decomposition
 *
 * @param [in] constraint : C = dfdx, D = dfdu, e = f;
 * @return Projection terms Px = dfdx, Pu = dfdu, Pe = f (first) and left pseudo-inverse of D^T (second);
 */
std::pair<VectorFunctionLinearApproximation, matrix_t> qrConstraintProjection(const VectorFunctionLinearApproximation& constraint);

/**
 * Returns the linear projection
 *  u = Pu * \tilde{u} + Px * x + Pe
 *
 * s.t. C*x + D*u + e = 0 is satisfied for any \tilde{u}
 *
 * Implementation based on the LU decomposition
 *
 * @param [in] constraint : C = dfdx, D = dfdu, e = f;
 * @param [in] extractPseudoInverse : If true, left pseudo-inverse of D^T is returned. If false, an empty matrix is returned;
 * @return Projection terms Px = dfdx, Pu = dfdu, Pe = f (first) and left pseudo-inverse of D^T (second);
 */
std::pair<VectorFunctionLinearApproximation, matrix_t> luConstraintProjection(const VectorFunctionLinearApproximation& constraint,
                                                                              bool extractPseudoInverse = false);

/** Computes the rank of a matrix */
template <typename Derived>
int rank(const Derived& A) {
  return A.colPivHouseholderQr().rank();
}

/** Computes the complex eigenvalues of A */
template <typename Derived>
Eigen::VectorXcd eigenvalues(const Derived& A) {
  return A.eigenvalues();
}

/** Computes the eigenvalues for a symmetric matrix A */
template <typename Derived>
vector_t symmetricEigenvalues(const Derived& A) {
  return A.template selfadjointView<Eigen::Lower>().eigenvalues();
}

// Declaring explicit instantiations for dynamic sized matrices
extern template int rank(const matrix_t& A);
extern template Eigen::VectorXcd eigenvalues(const matrix_t& A);
extern template vector_t symmetricEigenvalues(const matrix_t& A);

}  // namespace LinearAlgebra
}  // namespace ocs2
