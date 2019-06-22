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
template<typename Derived>
bool makePSD(Eigen::MatrixBase<Derived> &squareMatrix) {

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
