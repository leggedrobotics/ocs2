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

#include <ocs2_core/misc/LinearAlgebra.h>

namespace ocs2 {
namespace LinearAlgebra {

bool makePSD(Eigen::MatrixXd& squareMatrix) {
  if (squareMatrix.rows() != squareMatrix.cols()) {
    throw std::runtime_error("Not a square matrix: makePSD() method is for square matrix.");
  }

  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(squareMatrix, Eigen::EigenvaluesOnly);
  Eigen::VectorXd lambda = eig.eigenvalues();

  bool hasNegativeEigenValue = false;
  for (size_t j = 0; j < lambda.size(); j++) {
    if (lambda(j) < 0.0) {
      hasNegativeEigenValue = true;
      lambda(j) = 1e-6;
    }
  }

  if (hasNegativeEigenValue) {
    eig.compute(squareMatrix, Eigen::ComputeEigenvectors);
    squareMatrix = eig.eigenvectors() * lambda.asDiagonal() * eig.eigenvectors().inverse();
  } else {
    squareMatrix = 0.5 * (squareMatrix + squareMatrix.transpose()).eval();
  }

  return hasNegativeEigenValue;
}

void computeConstraintProjection(const Eigen::MatrixXd& D, const Eigen::MatrixXd& RinvChol, Eigen::MatrixXd& Ddagger,
                                 Eigen::MatrixXd& DdaggerT_R_Ddagger_Chol, Eigen::MatrixXd& RinvConstrainedChol) {
  const auto numConstraints = D.rows();
  const auto numInputs = D.cols();

  // Constraint Projectors are based on the QR decomposition
  Eigen::HouseholderQR<Eigen::MatrixXd> QRof_RinvCholT_DmT(RinvChol.transpose() * D.transpose());

  Eigen::MatrixXd QRof_RinvCholT_DmT_Rc =
      QRof_RinvCholT_DmT.matrixQR().topLeftCorner(numConstraints, numConstraints).template triangularView<Eigen::Upper>();

  // Computes the inverse of Rc with an efficient in-place forward-backward substitution
  // Turns out that this is equal to the cholesky decomposition of Ddagger^T * R * Ddagger after simplification
  DdaggerT_R_Ddagger_Chol.setIdentity(numConstraints, numConstraints);
  QRof_RinvCholT_DmT_Rc.template triangularView<Eigen::Upper>().solveInPlace(DdaggerT_R_Ddagger_Chol);

  Eigen::MatrixXd QRof_RinvCholT_DmT_Q = QRof_RinvCholT_DmT.householderQ();
  // Auto take reference to the column view here without making a temporary
  auto QRof_RinvCholT_DmT_Qc = QRof_RinvCholT_DmT_Q.leftCols(numConstraints);
  auto QRof_RinvCholT_DmT_Qu = QRof_RinvCholT_DmT_Q.rightCols(numInputs - numConstraints);

  // Compute Weighted Pseudo Inverse, brackets used to compute the smaller, right-side product first
  Ddagger.noalias() = RinvChol * (QRof_RinvCholT_DmT_Qc * DdaggerT_R_Ddagger_Chol.transpose());

  // Constraint input cost cholesky decomposition
  RinvConstrainedChol.noalias() = RinvChol * QRof_RinvCholT_DmT_Qu;
}

int rank(const Eigen::MatrixXd& A) {
  return A.colPivHouseholderQr().rank();
}

Eigen::VectorXcd eigenvalues(const Eigen::MatrixXd& A) {
  return A.eigenvalues();
}

}  // namespace LinearAlgebra
}  // namespace ocs2
