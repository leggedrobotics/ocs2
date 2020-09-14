#pragma once

#include <ocs2_core/Types.h>

namespace ocs2 {
namespace LinearAlgebra {
/**
 * Compute random, symmetric, positive definite (SPD), and diagonally dominant matrix.
 * @return random SPD matrix.
 */
template <typename MatrixType>
MatrixType generateSPDmatrix() {
  MatrixType A;
  A.setRandom();
  A = 0.5 * (A + A.transpose()).eval();    // avoid aliasing
  A.diagonal().array() += A.rows() * 1.0;  // makes the matrix diagonally dominant
  return A;
}

/**
 * Compute random, symmetric, positive definite (SPD), and diagonally dominant matrix of dimension size.
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
matrix_t generateFullRowRankmatrix(size_t m, size_t n) {
  if (m > n) {
    throw std::runtime_error("[generateFullRowRankmatrix] Can't generate matrix with more rows than columns");
  };

  // Some random constraint matrix
  matrix_t A = matrix_t::Random(m, n);
  A.block(0, 0, m, m).setIdentity();  // Makes sure rows are independent
  return A;
}
}  // namespace LinearAlgebra
}  // namespace ocs2
