

#include <gtest/gtest.h>
#include <ocs2_core/misc/LinearAlgebra.h>
#include "ocs2_core/misc/randomMatrices.h"

using namespace ocs2;
using namespace LinearAlgebra;

TEST(LLTofInverse, checkAgainstFullInverse)
{
  const size_t n = 10; // matrix size
  const double tol = 1e-9; // Coefficient-wise tolerance

  // Some random symmetric positive definite matrix
  using Matrix_t = Eigen::Matrix<double, n, n>;
  Matrix_t A = generateSPDmatrix<Matrix_t>();
  Matrix_t LinvT;

  computeLinvTLinv(A, LinvT);

  Matrix_t Ainv = A.inverse();
  Matrix_t Ainv_constructed = LinvT * LinvT.transpose();

  ASSERT_LT( (Ainv - Ainv_constructed).array().abs().maxCoeff() , tol );
}

TEST(constraintProjection, checkAgainstFullComputations)
{
  const size_t m = 4; // num constraints
  const size_t n = 15; // num inputs
  const double tol = 1e-9; // Coefficient-wise tolerance

  // Some random constraint matrix
  Eigen::MatrixXd D = generateFullRowRankmatrix(m, n);

  // Random cost matrix
  using Matrix_t = Eigen::Matrix<double, n, n>;
  Matrix_t R = generateSPDmatrix<Matrix_t>();

  // Inverse of R
  Matrix_t RinvChol;
  computeLinvTLinv(R, RinvChol);
  Matrix_t Rinv = RinvChol * RinvChol.transpose();

  // Compute constraint projection terms, this is what we are testing in this unit test
  Eigen::MatrixXd Ddagger, DdaggerT_R_Ddagger_Chol, RinvConstrainedChol;
  computeConstraintProjection(D, RinvChol, Ddagger, DdaggerT_R_Ddagger_Chol, RinvConstrainedChol);

  // Reconstruct full matrices to compare
  Eigen::MatrixXd RinvConstrained = RinvConstrainedChol * RinvConstrainedChol.transpose();
  Eigen::MatrixXd DdaggerT_R_Ddagger = DdaggerT_R_Ddagger_Chol * DdaggerT_R_Ddagger_Chol.transpose();

  // Alternative computation following Farshidian - An Efficient Optimal Planning and Control Framework For Quadrupedal Locomotion
  // Check Ddagger
  Eigen::MatrixXd RmProjected = (D * Rinv * D.transpose()).ldlt().solve(Eigen::MatrixXd::Identity(m,m));
  Eigen::MatrixXd Ddagger_check = Rinv * D.transpose() * RmProjected;
  ASSERT_LT( (Ddagger - Ddagger_check).array().abs().maxCoeff() , tol );

  // Check Ddagger^T * R * Ddagger
  Eigen::MatrixXd DdaggerT_R_Ddagger_check = Ddagger_check.transpose() * R * Ddagger_check;
  ASSERT_LT( (DdaggerT_R_Ddagger - DdaggerT_R_Ddagger_check).array().abs().maxCoeff() , tol );

  // Check Constrained cost matrix defined as defined in the paper
  Eigen::MatrixXd nullspaceProjection = Eigen::MatrixXd::Identity(n, n) - Ddagger_check * D;
  Eigen::MatrixXd RinvConstrained_check = Rinv.transpose() * nullspaceProjection.transpose() * R * nullspaceProjection * Rinv;
  ASSERT_LT( (RinvConstrained - RinvConstrained_check).array().abs().maxCoeff() , tol );
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}