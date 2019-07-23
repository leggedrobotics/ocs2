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
template <typename Derived>
bool makePSD(Eigen::MatrixBase<Derived>& squareMatrix) {
  if (squareMatrix.rows() != squareMatrix.cols()) throw std::runtime_error("Not a square matrix: makePSD() method is for square matrix.");

  Eigen::SelfAdjointEigenSolver<Derived> eig(squareMatrix, Eigen::EigenvaluesOnly);
  Eigen::VectorXd lambda = eig.eigenvalues();

  bool hasNegativeEigenValue = false;
  for (size_t j = 0; j < lambda.size(); j++)
    if (lambda(j) < 0.0) {
      hasNegativeEigenValue = true;
      lambda(j) = 1e-6;
    }

  if (hasNegativeEigenValue) {
    eig.compute(squareMatrix, Eigen::ComputeEigenvectors);
    squareMatrix = eig.eigenvectors() * lambda.asDiagonal() * eig.eigenvectors().inverse();
  } else {
    squareMatrix = 0.5 * (squareMatrix + squareMatrix.transpose()).eval();
  }

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
template <typename DerivedInputMatrix>
void computeConstraintProjection(const Eigen::MatrixXd& D, const DerivedInputMatrix& RinvChol, Eigen::MatrixXd& Ddagger,
                                 Eigen::MatrixXd& DdaggerT_R_Ddagger_Chol, Eigen::MatrixXd& RinvConstrainedChol) {
  const auto numConstraints = D.rows();
  const auto numInputs = D.cols();

  // Constraint Projectors are based on the QR decomposition
  Eigen::HouseholderQR<Eigen::MatrixXd> QRof_RinvCholT_DmT(RinvChol.transpose() * D.transpose());
  Eigen::MatrixXd QRof_RinvCholT_DmT_Q = QRof_RinvCholT_DmT.householderQ();
  Eigen::MatrixXd QRof_RinvCholT_DmT_Qu = QRof_RinvCholT_DmT_Q.rightCols(numInputs - numConstraints);
  Eigen::MatrixXd QRof_RinvCholT_DmT_Qc = QRof_RinvCholT_DmT_Q.leftCols(numConstraints);
  Eigen::MatrixXd QRof_RinvCholT_DmT_Rc =
      QRof_RinvCholT_DmT.matrixQR().topLeftCorner(numConstraints, numConstraints).template triangularView<Eigen::Upper>();

  // Computes the inverse of Rc with an efficient in-place forward-backward substitution
  // Turns out that this is equal to the cholesky decomposition of Ddagger^T * R * Ddagger after simplification
  DdaggerT_R_Ddagger_Chol.setIdentity(numConstraints, numConstraints);
  QRof_RinvCholT_DmT_Rc.template triangularView<Eigen::Upper>().solveInPlace(DdaggerT_R_Ddagger_Chol);

  // Compute Weighted Pseudo Inverse, brackets used to compute the smaller, right-side product first
  Ddagger.noalias() = RinvChol * (QRof_RinvCholT_DmT_Qc * DdaggerT_R_Ddagger_Chol.transpose());

  // Constraint input cost cholesky decomposition
  RinvConstrainedChol.noalias() = RinvChol * QRof_RinvCholT_DmT_Qu;
}

}  // namespace LinearAlgebra
}  // namespace ocs2

#endif  // OCS2_CTRL_LINEARALGEBRA_H
