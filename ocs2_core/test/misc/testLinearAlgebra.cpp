#include <gtest/gtest.h>

#include <ocs2_core/Types.h>
#include <ocs2_core/misc/LinearAlgebra.h>
#include <ocs2_core/misc/randomMatrices.h>

using namespace ocs2;
using namespace LinearAlgebra;

TEST(LLTofInverse, checkAgainstFullInverse) {
  const size_t n = 10;        // matrix size
  const scalar_t tol = 1e-9;  // Coefficient-wise tolerance

  // Some random symmetric positive definite matrix
  matrix_t A = generateSPDmatrix<matrix_t>(n);
  matrix_t AmInvUmUmT;

  computeInverseMatrixUUT(A, AmInvUmUmT);

  matrix_t Ainv = A.inverse();
  matrix_t Ainv_constructed = AmInvUmUmT * AmInvUmUmT.transpose();

  ASSERT_LT((Ainv - Ainv_constructed).array().abs().maxCoeff(), tol);
}

TEST(constraintProjection, checkAgainstFullComputations) {
  const size_t m = 4;         // num constraints
  const size_t n = 15;        // num inputs
  const scalar_t tol = 1e-9;  // Coefficient-wise tolerance

  // Some random constraint matrix
  matrix_t D = generateFullRowRankmatrix(m, n);

  // Random cost matrix
  matrix_t R = generateSPDmatrix<matrix_t>(n);

  // Inverse of R
  matrix_t RmInvUmUmT;
  computeInverseMatrixUUT(R, RmInvUmUmT);
  matrix_t Rinv = RmInvUmUmT * RmInvUmUmT.transpose();

  // Compute constraint projection terms, this is what we are testing in this unit test
  matrix_t Ddagger, DdaggerT_R_Ddagger_Chol, RinvConstrainedChol;
  computeConstraintProjection(D, RmInvUmUmT, Ddagger, DdaggerT_R_Ddagger_Chol, RinvConstrainedChol);

  // Reconstruct full matrices to compare
  matrix_t RinvConstrained = RinvConstrainedChol * RinvConstrainedChol.transpose();
  matrix_t DdaggerT_R_Ddagger = DdaggerT_R_Ddagger_Chol * DdaggerT_R_Ddagger_Chol.transpose();

  // Alternative computation following Farshidian - An Efficient Optimal Planning and Control Framework For Quadrupedal Locomotion
  // Check Ddagger
  matrix_t RmProjected = (D * Rinv * D.transpose()).ldlt().solve(matrix_t::Identity(m, m));
  matrix_t Ddagger_check = Rinv * D.transpose() * RmProjected;
  ASSERT_LT((Ddagger - Ddagger_check).array().abs().maxCoeff(), tol);

  // Check Ddagger^T * R * Ddagger
  matrix_t DdaggerT_R_Ddagger_check = Ddagger_check.transpose() * R * Ddagger_check;
  ASSERT_LT((DdaggerT_R_Ddagger - DdaggerT_R_Ddagger_check).array().abs().maxCoeff(), tol);

  // Check Constrained cost matrix defined as defined in the paper
  matrix_t nullspaceProjection = matrix_t::Identity(n, n) - Ddagger_check * D;
  matrix_t RinvConstrained_check = Rinv.transpose() * nullspaceProjection.transpose() * R * nullspaceProjection * Rinv;
  ASSERT_LT((RinvConstrained - RinvConstrained_check).array().abs().maxCoeff(), tol);
}

TEST(makePsdGershgorin, makePsdGershgorin) {
  const size_t n = 10;        // matrix size
  const scalar_t tol = 1e-9;  // Coefficient-wise tolerance

  // a random, symmetric, and diagonally dominant matrix
  matrix_t ddMat = generateSPDmatrix<matrix_t>(n);
  matrix_t ddMatCorr = ddMat;
  makePsdGershgorin(ddMatCorr);
  ASSERT_TRUE(ddMat.isApprox(ddMatCorr, tol));

  // non-definite matrix
  auto lambdaMin = ocs2::LinearAlgebra::symmetricEigenvalues(ddMat).minCoeff();
  matrix_t ndMat = ddMat - (lambdaMin + 1e-2) * matrix_t::Identity(n, n);
  matrix_t ndMatCorr = ndMat;
  const scalar_t minDesiredEigenvalue = 1e-3;
  makePsdGershgorin(ndMatCorr, minDesiredEigenvalue);
  vector_t lambda = ocs2::LinearAlgebra::symmetricEigenvalues(ndMat);
  vector_t lambdaCorr = ocs2::LinearAlgebra::symmetricEigenvalues(ndMatCorr);
  std::cerr << "MakePSD Gershgorin:" << std::endl;
  std::cerr << "eigenvalues            " << lambda.transpose() << std::endl;
  std::cerr << "eigenvalues corrected: " << lambdaCorr.transpose() << std::endl;
  ASSERT_GE(lambdaCorr.minCoeff(), minDesiredEigenvalue);
}

TEST(makePsdCholesky, makePsdCholesky) {
  const size_t n = 10;        // matrix size
  const scalar_t tol = 1e-9;  // Coefficient-wise tolerance

  // PSD matrix check
  // some random symmetric matrix
  matrix_t psdMat = generateSPDmatrix<matrix_t>(n);
  matrix_t psdMatCorr = psdMat;
  makePsdCholesky(psdMatCorr, 0.0);
  ASSERT_TRUE(psdMat.isApprox(psdMatCorr, tol));

  // non-definite matrix
  auto lambdaMin = ocs2::LinearAlgebra::symmetricEigenvalues(psdMat).minCoeff();
  matrix_t ndMat = psdMat - (lambdaMin + 1e-2) * matrix_t::Identity(n, n);
  matrix_t ndMatCorr = ndMat;
  const scalar_t minDesiredEigenvalue = 1e-1;
  makePsdCholesky(ndMatCorr, minDesiredEigenvalue);
  vector_t lambda = ocs2::LinearAlgebra::symmetricEigenvalues(ndMat);
  vector_t lambdaCorr = ocs2::LinearAlgebra::symmetricEigenvalues(ndMatCorr);
  std::cerr << "MakePSD Cholesky: " << std::endl;
  std::cerr << "eigenvalues            " << lambda.transpose() << std::endl;
  std::cerr << "eigenvalues corrected: " << lambdaCorr.transpose() << std::endl;
  ASSERT_GE(lambdaCorr.minCoeff(), minDesiredEigenvalue);

  // zero matrix
  matrix_t zeroMat = matrix_t::Zero(n, n);
  makePsdCholesky(zeroMat, minDesiredEigenvalue);
  vector_t lambdaZeroMat = ocs2::LinearAlgebra::symmetricEigenvalues(zeroMat);
  ASSERT_GE(lambdaZeroMat.minCoeff(), minDesiredEigenvalue);

  // sparse matrix
  Eigen::VectorXd vec = Eigen::VectorXd::Random(n);
  vec = vec.unaryExpr([](scalar_t x) { return std::max(x, 0.0); });
  std::cerr << "Sparse vec:\n" << vec << std::endl;
  matrix_t sparseMat = vec * vec.transpose() - minDesiredEigenvalue * matrix_t::Identity(n, n);
  vector_t lambdaSparseMat = ocs2::LinearAlgebra::symmetricEigenvalues(sparseMat);
  makePsdCholesky(sparseMat, minDesiredEigenvalue);
  vector_t lambdaSparseMatCorr = ocs2::LinearAlgebra::symmetricEigenvalues(sparseMat);

  std::cerr << "MakePSD Cholesky Sparse Matrix: " << std::endl;
  std::cerr << "Sparse Matrix:\n" << sparseMat << std::endl;
  std::cerr << "Sparse Matrix eigenvalues            " << lambdaSparseMat.transpose() << std::endl;
  std::cerr << "Sparse Matrix eigenvalues corrected: " << lambdaSparseMatCorr.transpose() << std::endl;

  ASSERT_GE(lambdaSparseMatCorr.minCoeff(), minDesiredEigenvalue);
}
