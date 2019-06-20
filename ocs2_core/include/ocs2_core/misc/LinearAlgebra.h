//
// Created by rgrandia on 20.06.19.
//

#ifndef OCS2_CTRL_LINEARALGEBRA_H
#define OCS2_CTRL_LINEARALGEBRA_H

#include <Eigen/Dense>

namespace ocs2 {
namespace LinearAlgebra {

/**
 * Makes the matrix PSD.
 * @tparam Derived type.
 * @param [out] squareMatrix: The matrix to become PSD.
 * @return boolean
 */
template<typename Derived>
static bool makePSD(Eigen::MatrixBase<Derived> &squareMatrix) {

  if (squareMatrix.rows() != squareMatrix.cols())
    throw std::runtime_error("Not a square matrix: makePSD() method is for square matrix.");

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

} // namespace LinearAlgebra
} // namespace ocs2

#endif //OCS2_CTRL_LINEARALGEBRA_H
