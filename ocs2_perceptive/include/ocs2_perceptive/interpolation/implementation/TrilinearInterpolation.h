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
namespace trilinear_interpolation {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename Scalar>
Scalar getValue(Scalar resolution, const Eigen::Matrix<Scalar, 3, 1>& referenceCorner, const std::array<Scalar, 8>& cornerValues,
                const Eigen::Matrix<Scalar, 3, 1>& position) {
  // auxiliary variables
  const Scalar r_inv = 1.0 / resolution;
  std::array<Scalar, 10> v;
  v[0] = (position.x() - referenceCorner.x()) * r_inv;
  v[1] = (position.y() - referenceCorner.y()) * r_inv;
  v[2] = (position.z() - referenceCorner.z()) * r_inv;
  v[3] = 1 - v[0];
  v[4] = 1 - v[1];
  v[5] = 1 - v[2];
  v[6] = v[3] * cornerValues[0] + v[0] * cornerValues[1];  // f_00 = (1 - x) f_000 + x f_100
  v[7] = v[3] * cornerValues[2] + v[0] * cornerValues[3];  // f_10 = (1 - x) f_010 + x f_110
  v[8] = v[3] * cornerValues[4] + v[0] * cornerValues[5];  // f_01 = (1 - x) f_001 + x f_101
  v[9] = v[3] * cornerValues[6] + v[0] * cornerValues[7];  // f_11 = (1 - x) f_011 + x f_111
  // (1 - z) ((1 - y) f_00 + y f_10) + z ((1 - y) f_01 + y f_11)
  return v[5] * (v[4] * v[6] + v[1] * v[7]) + v[2] * (v[4] * v[8] + v[1] * v[9]);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename Scalar>
std::pair<Scalar, Eigen::Matrix<Scalar, 3, 1>> getLinearApproximation(Scalar resolution, const Eigen::Matrix<Scalar, 3, 1>& referenceCorner,
                                                                      const std::array<Scalar, 8>& cornerValues,
                                                                      const Eigen::Matrix<Scalar, 3, 1>& position) {
  /// auxiliary variables
  const Scalar r_inv = 1.0 / resolution;
  std::array<Scalar, 12> v;
  v[0] = (position.x() - referenceCorner.x()) * r_inv;
  v[1] = (position.y() - referenceCorner.y()) * r_inv;
  v[2] = (position.z() - referenceCorner.z()) * r_inv;
  v[3] = 1 - v[0];
  v[4] = 1 - v[1];
  v[5] = 1 - v[2];
  v[6] = v[3] * cornerValues[0] + v[0] * cornerValues[1];  // f_00 = (1 - x) f_000 + x f_100
  v[7] = v[3] * cornerValues[2] + v[0] * cornerValues[3];  // f_10 = (1 - x) f_010 + x f_110
  v[8] = v[3] * cornerValues[4] + v[0] * cornerValues[5];  // f_01 = (1 - x) f_001 + x f_101
  v[9] = v[3] * cornerValues[6] + v[0] * cornerValues[7];  // f_11 = (1 - x) f_011 + x f_111
  v[10] = v[4] * v[6] + v[1] * v[7];                       // f_0 = (1 - y) f_00 + y f_10
  v[11] = v[4] * v[8] + v[1] * v[9];                       // f_1 = (1 - y) f_01 + y f_11

  // f = (1 - z) f_0 + z f_1
  const Scalar value = v[5] * v[10] + v[2] * v[11];

  Eigen::Matrix<Scalar, 3, 1> gradient;
  // df_z = f_11 - f_10
  gradient.z() = (v[11] - v[10]) * r_inv;
  // df_y = (1 - z) (f_10 - f00) - z (f_11 - f_01)
  gradient.y() = (v[5] * (v[7] - v[6]) + v[2] * (v[9] - v[8])) * r_inv;
  // df_x = (1 - z) (1 - y) (f_100 - f_000) + (1 - z) y (f_110 - f_010) + z (1 - y) (f_101 - f_001) + z y (f_111 - f_011)
  gradient.x() = (v[5] * v[4] * (cornerValues[1] - cornerValues[0]) + v[5] * v[1] * (cornerValues[3] - cornerValues[2]) +
                  v[2] * v[4] * (cornerValues[5] - cornerValues[4]) + v[2] * v[1] * (cornerValues[7] - cornerValues[6])) *
                 r_inv;

  return {value, gradient};
}

}  // namespace trilinear_interpolation
}  // namespace ocs2
