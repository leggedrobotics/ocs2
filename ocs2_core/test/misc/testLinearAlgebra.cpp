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

#include <gtest/gtest.h>

#include <ocs2_core/Types.h>
#include <ocs2_core/misc/LinearAlgebra.h>
#include <ocs2_core/misc/randomMatrices.h>

TEST(test_projection, testProjectionQR) {
  constexpr ocs2::scalar_t nx = 30;
  constexpr ocs2::scalar_t nu = 20;
  constexpr ocs2::scalar_t nc = 10;
  const auto constraint = [&]() {
    ocs2::VectorFunctionLinearApproximation approx;
    approx.dfdx = ocs2::matrix_t::Random(nc, nx);
    approx.dfdu = ocs2::matrix_t::Random(nc, nu);
    approx.f = ocs2::vector_t::Random(nc);
    return approx;
  }();

  auto result = ocs2::LinearAlgebra::qrConstraintProjection(constraint);
  const auto projection = std::move(result.first);
  const auto pseudoInverse = std::move(result.second);

  // range of Pu is in null-space of D
  ASSERT_TRUE((constraint.dfdu * projection.dfdu).isZero());

  // D * Px cancels the C term
  ASSERT_TRUE((constraint.dfdx + constraint.dfdu * projection.dfdx).isZero());

  // D * Pe cancels the e term
  ASSERT_TRUE((constraint.f + constraint.dfdu * projection.f).isZero());

  ASSERT_EQ(pseudoInverse.rows(), constraint.dfdu.rows());
  ASSERT_EQ(pseudoInverse.cols(), constraint.dfdu.cols());

  ASSERT_TRUE((pseudoInverse * constraint.dfdu.transpose()).isIdentity());
  ASSERT_TRUE((pseudoInverse.transpose() * constraint.dfdx).isApprox(-projection.dfdx));
  ASSERT_TRUE((pseudoInverse.transpose() * constraint.f).isApprox(-projection.f));
}

TEST(test_projection, testProjectionLU) {
  constexpr ocs2::scalar_t nx = 30;
  constexpr ocs2::scalar_t nu = 20;
  constexpr ocs2::scalar_t nc = 10;
  const auto constraint = [&]() {
    ocs2::VectorFunctionLinearApproximation approx;
    approx.dfdx = ocs2::matrix_t::Random(nc, nx);
    approx.dfdu = ocs2::matrix_t::Random(nc, nu);
    approx.f = ocs2::vector_t::Random(nc);
    return approx;
  }();

  auto result = ocs2::LinearAlgebra::luConstraintProjection(constraint);
  ASSERT_EQ(result.second.rows(), 0);
  ASSERT_EQ(result.second.cols(), 0);

  const auto projection = std::move(result.first);

  // range of Pu is in null-space of D
  ASSERT_TRUE((constraint.dfdu * projection.dfdu).isZero());

  // D * Px cancels the C term
  ASSERT_TRUE((constraint.dfdx + constraint.dfdu * projection.dfdx).isZero());

  // D * Pe cancels the e term
  ASSERT_TRUE((constraint.f + constraint.dfdu * projection.f).isZero());

  auto resultWithPseudoInverse = ocs2::LinearAlgebra::luConstraintProjection(constraint, true);
  const auto projectionWithPseudoInverse = std::move(resultWithPseudoInverse.first);
  ASSERT_TRUE(projection.f.isApprox(projectionWithPseudoInverse.f));
  ASSERT_TRUE(projection.dfdx.isApprox(projectionWithPseudoInverse.dfdx));
  ASSERT_TRUE(projection.dfdu.isApprox(projectionWithPseudoInverse.dfdu));

  const auto pseudoInverse = std::move(resultWithPseudoInverse.second);
  ASSERT_EQ(pseudoInverse.rows(), constraint.dfdu.rows());
  ASSERT_EQ(pseudoInverse.cols(), constraint.dfdu.cols());

  ASSERT_TRUE((pseudoInverse * constraint.dfdu.transpose()).isIdentity());
  ASSERT_TRUE((pseudoInverse.transpose() * constraint.dfdx).isApprox(-projection.dfdx));
  ASSERT_TRUE((pseudoInverse.transpose() * constraint.f).isApprox(-projection.f));
}

TEST(LLTofInverse, checkAgainstFullInverse) {
  constexpr size_t n = 10;        // matrix size
  constexpr ocs2::scalar_t tol = 1e-9;  // Coefficient-wise tolerance

  // Some random symmetric positive definite matrix
  ocs2::matrix_t A = ocs2::LinearAlgebra::generateSPDmatrix<ocs2::matrix_t>(n);
  ocs2::matrix_t AmInvUmUmT;

  ocs2::LinearAlgebra::computeInverseMatrixUUT(A, AmInvUmUmT);

  ocs2::matrix_t Ainv = A.inverse();
  ocs2::matrix_t Ainv_constructed = AmInvUmUmT * AmInvUmUmT.transpose();

  ASSERT_LT((Ainv - Ainv_constructed).array().abs().maxCoeff(), tol);
}

TEST(constraintProjection, checkAgainstFullComputations) {
  constexpr size_t m = 4;         // num constraints
  constexpr size_t n = 15;        // num inputs
  constexpr ocs2::scalar_t tol = 1e-9;  // Coefficient-wise tolerance

  // Some random constraint matrix
  ocs2::matrix_t D = ocs2::LinearAlgebra::generateFullRowRankmatrix(m, n);

  // Random cost matrix
  ocs2::matrix_t R = ocs2::LinearAlgebra::generateSPDmatrix<ocs2::matrix_t>(n);

  // Inverse of R
  ocs2::matrix_t RmInvUmUmT;
  ocs2::LinearAlgebra::computeInverseMatrixUUT(R, RmInvUmUmT);
  ocs2::matrix_t Rinv = RmInvUmUmT * RmInvUmUmT.transpose();

  // Compute constraint projection terms, this is what we are testing in this unit test
  ocs2::matrix_t Ddagger, DdaggerT_R_Ddagger_Chol, RinvConstrainedChol;
  ocs2::LinearAlgebra::computeConstraintProjection(D, RmInvUmUmT, Ddagger, DdaggerT_R_Ddagger_Chol, RinvConstrainedChol);

  // Reconstruct full matrices to compare
  ocs2::matrix_t RinvConstrained = RinvConstrainedChol * RinvConstrainedChol.transpose();
  ocs2::matrix_t DdaggerT_R_Ddagger = DdaggerT_R_Ddagger_Chol * DdaggerT_R_Ddagger_Chol.transpose();

  // Alternative computation following Farshidian - An Efficient Optimal Planning and Control Framework For Quadrupedal Locomotion
  // Check Ddagger
  ocs2::matrix_t RmProjected = (D * Rinv * D.transpose()).ldlt().solve(ocs2::matrix_t::Identity(m, m));
  ocs2::matrix_t Ddagger_check = Rinv * D.transpose() * RmProjected;
  ASSERT_LT((Ddagger - Ddagger_check).array().abs().maxCoeff(), tol);

  // Check Ddagger^T * R * Ddagger
  ocs2::matrix_t DdaggerT_R_Ddagger_check = Ddagger_check.transpose() * R * Ddagger_check;
  ASSERT_LT((DdaggerT_R_Ddagger - DdaggerT_R_Ddagger_check).array().abs().maxCoeff(), tol);

  // Check Constrained cost matrix defined as defined in the paper
  ocs2::matrix_t nullspaceProjection = ocs2::matrix_t::Identity(n, n) - Ddagger_check * D;
  ocs2::matrix_t RinvConstrained_check = Rinv.transpose() * nullspaceProjection.transpose() * R * nullspaceProjection * Rinv;
  ASSERT_LT((RinvConstrained - RinvConstrained_check).array().abs().maxCoeff(), tol);
}

TEST(makePsdGershgorin, makePsdGershgorin) {
  constexpr size_t n = 10;        // matrix size
  constexpr ocs2::scalar_t tol = 1e-9;  // Coefficient-wise tolerance

  // a random, symmetric, and diagonally dominant matrix
  ocs2::matrix_t ddMat = ocs2::LinearAlgebra::generateSPDmatrix<ocs2::matrix_t>(n);
  ocs2::matrix_t ddMatCorr = ddMat;
  ocs2::LinearAlgebra::makePsdGershgorin(ddMatCorr);
  ASSERT_TRUE(ddMat.isApprox(ddMatCorr, tol));

  // non-definite matrix
  auto lambdaMin = ocs2::LinearAlgebra::symmetricEigenvalues(ddMat).minCoeff();
  ocs2::matrix_t ndMat = ddMat - (lambdaMin + 1e-2) * ocs2::matrix_t::Identity(n, n);
  ocs2::matrix_t ndMatCorr = ndMat;
  const ocs2::scalar_t minDesiredEigenvalue = 1e-3;
  ocs2::LinearAlgebra::makePsdGershgorin(ndMatCorr, minDesiredEigenvalue);
  ocs2::vector_t lambda = ocs2::LinearAlgebra::symmetricEigenvalues(ndMat);
  ocs2::vector_t lambdaCorr = ocs2::LinearAlgebra::symmetricEigenvalues(ndMatCorr);
  std::cerr << "MakePSD Gershgorin:" << std::endl;
  std::cerr << "eigenvalues            " << lambda.transpose() << std::endl;
  std::cerr << "eigenvalues corrected: " << lambdaCorr.transpose() << std::endl;
  ASSERT_GE(lambdaCorr.minCoeff(), minDesiredEigenvalue);
}

TEST(makePsdCholesky, makePsdCholesky) {
  constexpr size_t n = 10;        // matrix size
  constexpr ocs2::scalar_t tol = 1e-9;  // Coefficient-wise tolerance

  // PSD matrix check
  // some random symmetric matrix
  ocs2::matrix_t psdMat = ocs2::LinearAlgebra::generateSPDmatrix<ocs2::matrix_t>(n);
  ocs2::matrix_t psdMatCorr = psdMat;
  ocs2::LinearAlgebra::makePsdCholesky(psdMatCorr, 0.0);
  ASSERT_TRUE(psdMat.isApprox(psdMatCorr, tol));

  // non-definite matrix
  auto lambdaMin = ocs2::LinearAlgebra::symmetricEigenvalues(psdMat).minCoeff();
  ocs2::matrix_t ndMat = psdMat - (lambdaMin + 1e-2) * ocs2::matrix_t::Identity(n, n);
  ocs2::matrix_t ndMatCorr = ndMat;
  constexpr ocs2::scalar_t minDesiredEigenvalue = 1e-1;
  ocs2::LinearAlgebra::makePsdCholesky(ndMatCorr, minDesiredEigenvalue);
  ocs2::vector_t lambda = ocs2::LinearAlgebra::symmetricEigenvalues(ndMat);
  ocs2::vector_t lambdaCorr = ocs2::LinearAlgebra::symmetricEigenvalues(ndMatCorr);
  std::cerr << "MakePSD Cholesky: " << std::endl;
  std::cerr << "eigenvalues            " << lambda.transpose() << std::endl;
  std::cerr << "eigenvalues corrected: " << lambdaCorr.transpose() << std::endl;
  ASSERT_GE(lambdaCorr.minCoeff(), minDesiredEigenvalue);

  // zero matrix
  ocs2::matrix_t zeroMat = ocs2::matrix_t::Zero(n, n);
  ocs2::LinearAlgebra::makePsdCholesky(zeroMat, minDesiredEigenvalue);
  ocs2::vector_t lambdaZeroMat = ocs2::LinearAlgebra::symmetricEigenvalues(zeroMat);
  ASSERT_GE(lambdaZeroMat.minCoeff(), minDesiredEigenvalue);

  // sparse matrix
  Eigen::VectorXd vec = Eigen::VectorXd::Random(n);
  vec = vec.unaryExpr([](ocs2::scalar_t x) { return std::max(x, 0.0); });
  std::cerr << "Sparse vec:\n" << vec << std::endl;
  ocs2::matrix_t sparseMat = vec * vec.transpose() - minDesiredEigenvalue * ocs2::matrix_t::Identity(n, n);
  ocs2::vector_t lambdaSparseMat = ocs2::LinearAlgebra::symmetricEigenvalues(sparseMat);
  ocs2::LinearAlgebra::makePsdCholesky(sparseMat, minDesiredEigenvalue);
  ocs2::vector_t lambdaSparseMatCorr = ocs2::LinearAlgebra::symmetricEigenvalues(sparseMat);

  std::cerr << "MakePSD Cholesky Sparse Matrix: " << std::endl;
  std::cerr << "Sparse Matrix:\n" << sparseMat << std::endl;
  std::cerr << "Sparse Matrix eigenvalues            " << lambdaSparseMat.transpose() << std::endl;
  std::cerr << "Sparse Matrix eigenvalues corrected: " << lambdaSparseMatCorr.transpose() << std::endl;

  ASSERT_GE(lambdaSparseMatCorr.minCoeff(), minDesiredEigenvalue);
}
