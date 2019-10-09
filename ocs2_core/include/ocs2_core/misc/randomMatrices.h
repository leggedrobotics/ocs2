

#ifndef OCS2_CTRL_RANDOMMATRICES_H
#define OCS2_CTRL_RANDOMMATRICES_H

#include <Eigen/Dense>

namespace ocs2 {
namespace LinearAlgebra {
/**
 * Compute random symmetric positive definite (SPD) matrix
 * @return random SPD matrix.
 */
template <typename MatrixType>
MatrixType generateSPDmatrix() {
  MatrixType A;
  A.setRandom();
  A = 0.5 * (A + A.transpose()).eval();    // Avoid aliasing
  A.diagonal().array() += A.rows() * 1.0;  // makes the matrix diagonally dominant
  return A;
}

/**
 * Compute random symmetric positive definite (SPD) matrix of dimension size.
 * @param [in] size: Matrix dimension.
 * @return random SPD matrix.
 */
template <typename MatrixType>
MatrixType generateSPDmatrix(int size) {
  MatrixType A(size, size);
  A.setRandom();
  A = 0.5 * (A + A.transpose()).eval();    // Avoid aliasing
  A.diagonal().array() += A.rows() * 1.0;  // makes the matrix diagonally dominant
  return A;
}

/**
 * Compute random full row rank matrix
 * @return random SPD matrix.
 */
Eigen::MatrixXd generateFullRowRankmatrix(size_t m, size_t n) {
  if (m > n) {
    throw std::runtime_error("[generateFullRowRankmatrix] Can't generate matrix with more rows than columns");
  };

  // Some random constraint matrix
  Eigen::MatrixXd A(m, n);
  A.setRandom();
  A.block(0, 0, m, m).setIdentity();  // Makes sure rows are independent
  return A;
}
}  // namespace LinearAlgebra
}  // namespace ocs2

#endif  // OCS2_CTRL_RANDOMMATRICES_H
