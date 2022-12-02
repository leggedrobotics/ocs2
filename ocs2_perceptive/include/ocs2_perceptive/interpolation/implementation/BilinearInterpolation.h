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

#include <utility>

namespace ocs2 {
namespace bilinear_interpolation {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename Scalar>
Scalar getValue(Scalar resolution, const Eigen::Matrix<Scalar, 2, 1>& referenceCorner, const std::array<Scalar, 4>& cornerValues,
                const Eigen::Matrix<Scalar, 2, 1>& position) {
  // auxiliary variables
  const Scalar r_inv = 1.0 / resolution;
  std::array<Scalar, 4> v;
  v[0] = (position.x() - referenceCorner.x()) * r_inv;
  v[1] = (position.y() - referenceCorner.y()) * r_inv;
  v[2] = 1 - v[0];
  v[3] = 1 - v[1];
  // (1 - x)(1 - y) f_00 + x (1 - y) f_10 + (1 - x) y f_01 + x y f_11
  return cornerValues[0] * v[2] * v[3] + cornerValues[1] * v[0] * v[3] + cornerValues[2] * v[2] * v[1] + cornerValues[3] * v[0] * v[1];
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename Scalar>
std::pair<Scalar, Eigen::Matrix<Scalar, 2, 1>> getLinearApproximation(Scalar resolution, const Eigen::Matrix<Scalar, 2, 1>& referenceCorner,
                                                                      const std::array<Scalar, 4>& cornerValues,
                                                                      const Eigen::Matrix<Scalar, 2, 1>& position) {
  // auxiliary variables
  const Scalar r_inv = 1.0 / resolution;
  std::array<Scalar, 6> v;
  v[0] = (position.y() - referenceCorner.y()) * r_inv;
  v[1] = (position.x() - referenceCorner.x()) * r_inv;
  v[2] = 1.0 - v[0];
  v[3] = 1.0 - v[1];
  v[4] = cornerValues[2] * v[3];  // (1 - x) f_01
  v[5] = cornerValues[3] * v[1];  // x f_11
  v[3] = cornerValues[0] * v[3];  // (1 - x) f_00
  v[1] = cornerValues[1] * v[1];  // x f_10

  // x ( 1 - y) f_10 + (1 - x) (1 - y) f_00 + (1 - x) y f_01 + x y f_11
  const Scalar value = v[1] * v[2] + v[3] * v[2] + v[4] * v[0] + v[5] * v[0];

  Eigen::Matrix<Scalar, 2, 1> gradient;
  // (1 - y) (f_10 - f_00) + y (f_11 - f_01)
  gradient.x() = (v[2] * (cornerValues[1] - cornerValues[0]) + v[0] * (cornerValues[3] - cornerValues[2])) * r_inv;
  // (1 - x) (f_01 - f_00) + x (f_11 - f_10)
  gradient.y() = (v[4] + v[5] - (v[3] + v[1])) * r_inv;

  return {value, gradient};
}

}  // namespace bilinear_interpolation
}  // namespace ocs2
