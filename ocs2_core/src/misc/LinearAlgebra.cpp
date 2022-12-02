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

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void setTriangularMinimumEigenvalues(matrix_t& Lr, scalar_t minEigenValue) {
  for (Eigen::Index i = 0; i < Lr.rows(); ++i) {
    scalar_t& eigenValue = Lr(i, i);  // diagonal element is the eigenvalue
    if (eigenValue < 0.0) {
      eigenValue = std::min(-minEigenValue, eigenValue);
    } else {
      eigenValue = std::max(minEigenValue, eigenValue);
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void makePsdEigenvalue(matrix_t& squareMatrix, scalar_t minEigenvalue) {
  assert(squareMatrix.rows() == squareMatrix.cols());

  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(squareMatrix, Eigen::EigenvaluesOnly);
  vector_t lambda = eig.eigenvalues();

  bool hasNegativeEigenValue = false;
  for (size_t j = 0; j < lambda.size(); j++) {
    if (lambda(j) < minEigenvalue) {
      hasNegativeEigenValue = true;
      lambda(j) = minEigenvalue;
    }
  }

  if (hasNegativeEigenValue) {
    eig.compute(squareMatrix, Eigen::ComputeEigenvectors);
    squareMatrix = eig.eigenvectors() * lambda.asDiagonal() * eig.eigenvectors().inverse();
  } else {
    squareMatrix = 0.5 * (squareMatrix + squareMatrix.transpose()).eval();
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void makePsdGershgorin(matrix_t& squareMatrix, scalar_t minEigenvalue) {
  assert(squareMatrix.rows() == squareMatrix.cols());
  squareMatrix = 0.5 * (squareMatrix + squareMatrix.transpose()).eval();
  for (size_t i = 0; i < squareMatrix.rows(); i++) {
    // Gershgorin radius: since the matrix is symmetric we use column sum instead of row sum
    auto Ri = squareMatrix.col(i).cwiseAbs().sum() - std::abs(squareMatrix(i, i));
    squareMatrix(i, i) = std::max(squareMatrix(i, i), Ri + minEigenvalue);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void makePsdCholesky(matrix_t& A, scalar_t minEigenvalue) {
  using sparse_matrix_t = Eigen::SparseMatrix<scalar_t>;

  assert(A.rows() == A.cols());

  // set the minimum eigenvalue
  A.diagonal().array() -= minEigenvalue;

  // S P' (A + E) P S = L L'
  sparse_matrix_t squareMatrix = 0.5 * A.sparseView();
  A.transposeInPlace();
  squareMatrix += 0.5 * A.sparseView();
  Eigen::IncompleteCholesky<scalar_t> incompleteCholesky(squareMatrix);
  // P' A P = M M'
  sparse_matrix_t M = (incompleteCholesky.scalingS().asDiagonal().inverse() * incompleteCholesky.matrixL());
  // L L' = P M M' P'
  sparse_matrix_t LmTwisted;
  LmTwisted.selfadjointView<Eigen::Lower>() = M.selfadjointView<Eigen::Lower>().twistedBy(incompleteCholesky.permutationP());
  const matrix_t L(LmTwisted);
  // A = L L'
  A = L * L.transpose();

  // correction for the minimum eigenvalue
  A.diagonal().array() += minEigenvalue;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void computeInverseMatrixUUT(const matrix_t& Am, matrix_t& AmInvUmUmT) {
  // Am = Lm Lm^T --> inv(Am) = inv(Lm^T) inv(Lm) where Lm^T is upper triangular
  Eigen::LLT<matrix_t> lltOfA(Am);
  AmInvUmUmT.setIdentity(Am.rows(), Am.cols());  // for dynamic size matrices
  lltOfA.matrixU().solveInPlace(AmInvUmUmT);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void computeConstraintProjection(const matrix_t& Dm, const matrix_t& RmInvUmUmT, matrix_t& DmDagger, matrix_t& DmDaggerTRmDmDaggerUUT,
                                 matrix_t& RmInvConstrainedUUT) {
  const auto numConstraints = Dm.rows();
  const auto numInputs = Dm.cols();

  // Constraint Projectors are based on the QR decomposition
  Eigen::HouseholderQR<matrix_t> QRof_RmInvUmUmTT_DmT(RmInvUmUmT.transpose() * Dm.transpose());

  matrix_t QRof_RmInvUmUmTT_DmT_Rc = QRof_RmInvUmUmTT_DmT.matrixQR().topRows(numConstraints).triangularView<Eigen::Upper>();
  setTriangularMinimumEigenvalues(QRof_RmInvUmUmTT_DmT_Rc);

  // Computes the inverse of Rc with an efficient in-place forward-backward substitution
  // Turns out that this is equal to the UUT decomposition of DmDagger^T * R * DmDagger after simplification
  DmDaggerTRmDmDaggerUUT.setIdentity(numConstraints, numConstraints);
  QRof_RmInvUmUmTT_DmT_Rc.triangularView<Eigen::Upper>().solveInPlace(DmDaggerTRmDmDaggerUUT);

  matrix_t QRof_RmInvUmUmTT_DmT_Q = QRof_RmInvUmUmTT_DmT.householderQ();
  // Auto take reference to the column view here without making a temporary
  auto QRof_RmInvUmUmTT_DmT_Qc = QRof_RmInvUmUmTT_DmT_Q.leftCols(numConstraints);
  auto QRof_RmInvUmUmTT_DmT_Qu = QRof_RmInvUmUmTT_DmT_Q.rightCols(numInputs - numConstraints);

  // Compute Weighted Pseudo Inverse, brackets used to compute the smaller, right-side product first
  DmDagger.noalias() = RmInvUmUmT * (QRof_RmInvUmUmTT_DmT_Qc * DmDaggerTRmDmDaggerUUT.transpose());

  // Constraint input cost UUT decomposition
  RmInvConstrainedUUT.noalias() = RmInvUmUmT * QRof_RmInvUmUmTT_DmT_Qu;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::pair<VectorFunctionLinearApproximation, matrix_t> qrConstraintProjection(const VectorFunctionLinearApproximation& constraint) {
  // Constraint Projectors are based on the QR decomposition
  const auto numConstraints = constraint.dfdu.rows();
  const auto numInputs = constraint.dfdu.cols();
  const Eigen::HouseholderQR<matrix_t> QRof_DT(constraint.dfdu.transpose());

  const matrix_t Q = QRof_DT.householderQ();
  const auto Q1 = Q.leftCols(numConstraints);

  const auto R = QRof_DT.matrixQR().topRows(numConstraints).triangularView<Eigen::Upper>();
  const matrix_t pseudoInverse = R.solve(Q1.transpose());  // left pseudo-inverse of D^T

  VectorFunctionLinearApproximation projectionTerms;
  projectionTerms.dfdu = Q.rightCols(numInputs - numConstraints);
  projectionTerms.dfdx.noalias() = -pseudoInverse.transpose() * constraint.dfdx;
  projectionTerms.f.noalias() = -pseudoInverse.transpose() * constraint.f;

  return std::make_pair(std::move(projectionTerms), std::move(pseudoInverse));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::pair<VectorFunctionLinearApproximation, matrix_t> luConstraintProjection(const VectorFunctionLinearApproximation& constraint,
                                                                              bool extractPseudoInverse) {
  // Constraint Projectors are based on the LU decomposition
  const Eigen::FullPivLU<matrix_t> lu(constraint.dfdu);

  VectorFunctionLinearApproximation projectionTerms;
  projectionTerms.dfdu = lu.kernel();
  projectionTerms.dfdx.noalias() = -lu.solve(constraint.dfdx);
  projectionTerms.f.noalias() = -lu.solve(constraint.f);

  matrix_t pseudoInverse;
  if (extractPseudoInverse) {
    pseudoInverse = lu.solve(matrix_t::Identity(constraint.f.size(), constraint.f.size())).transpose();  // left pseudo-inverse of D^T
  }

  return std::make_pair(std::move(projectionTerms), std::move(pseudoInverse));
}

// Explicit instantiations for dynamic sized matrices
template int rank(const matrix_t& A);
template Eigen::VectorXcd eigenvalues(const matrix_t& A);
template vector_t symmetricEigenvalues(const matrix_t& A);

}  // namespace LinearAlgebra
}  // namespace ocs2
