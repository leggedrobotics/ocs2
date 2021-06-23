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

#pragma once

#include "ocs2_core/Types.h"

#include <cppad/cg.hpp>

namespace ocs2 {

/**
 * Get the inverse of the sub-block of the centroidal momentum matrix which corresponds to the floating base variables.
 *  Ab_inv = [1/m I_{3,3}, -1/m*Ab_12*Ab_22^(-1),
 *               O_{3,3}, Ab_22^(-1)]
 *
 * @param [in] A(q): centroidal momentum matrix
 * @param [out] Ab_inv(q): inverse of the 6x6 left-block of A(q)
 */
template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 6, 6> getFloatingBaseCentroidalMomentumMatrixInverse(const Eigen::Matrix<SCALAR_T, 6, 6>& Ab) {
  const SCALAR_T mass = Ab(0, 0);
  Eigen::Matrix<SCALAR_T, 3, 3> Ab_22_inv = Ab.template block<3, 3>(3, 3).inverse();
  Eigen::Matrix<SCALAR_T, 6, 6> Ab_inv = Eigen::Matrix<SCALAR_T, 6, 6>::Zero();
  Ab_inv << 1.0 / mass * Eigen::Matrix<SCALAR_T, 3, 3>::Identity(), -1.0 / mass * Ab.template block<3, 3>(0, 3) * Ab_22_inv,
      Eigen::Matrix<SCALAR_T, 3, 3>::Zero(), Ab_22_inv;
  return Ab_inv;
}

template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 3> getRotationMatrixFromZyxEulerAngles(const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAngles) {
  const SCALAR_T z = eulerAngles(0);
  const SCALAR_T y = eulerAngles(1);
  const SCALAR_T x = eulerAngles(2);

  const SCALAR_T c1 = cos(z);
  const SCALAR_T c2 = cos(y);
  const SCALAR_T c3 = cos(x);
  const SCALAR_T s1 = sin(z);
  const SCALAR_T s2 = sin(y);
  const SCALAR_T s3 = sin(x);

  // clang-format off
  Eigen::Matrix<SCALAR_T, 3, 3> rotationMatrix;
  rotationMatrix << c1 * c2, c1 * s2 * s3 - s1 * c3, c1 * s2 * c3 + s1 * s3,
                    s1 * c2, s1 * s2 * s3 + c1 * c3, s1 * s2 * c3 - c1 * s3,
                        -s2,                c2 * s3,                c2 * c3;
  // clang-format on
  return rotationMatrix;
}

template <typename SCALAR_T>
void getAngularVelocityInWorldFrameFromEulerAnglesZyx(const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAngles,
                                                      const Eigen::Matrix<SCALAR_T, 3, 1>& derivativesEulerAngles,
                                                      Eigen::Matrix<SCALAR_T, 3, 1>& angularVelocity) {
  const SCALAR_T z = eulerAngles(0);
  const SCALAR_T y = eulerAngles(1);
  const SCALAR_T x = eulerAngles(2);
  Eigen::Matrix<SCALAR_T, 3, 3> transformationMatrix;
  // clang-format off
  transformationMatrix << 0, -sin(z), cos(y)*cos(z),
                          0, cos(z), cos(y)*sin(z),
                          1,      0, -sin(y);
  // clang-format on

  angularVelocity = transformationMatrix * derivativesEulerAngles;
}

template <typename SCALAR_T>
void getAngularAccelerationInWorldFrameFromEulerAnglesZyx(const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAngles,
                                                          const Eigen::Matrix<SCALAR_T, 3, 1>& derivativesEulerAngles,
                                                          const Eigen::Matrix<SCALAR_T, 3, 1>& secondDerivativesEulerAngles,
                                                          Eigen::Matrix<SCALAR_T, 3, 1>& angularAcceleration) {
  const SCALAR_T z = eulerAngles(0);
  const SCALAR_T y = eulerAngles(1);
  const SCALAR_T x = eulerAngles(2);
  const SCALAR_T dz = derivativesEulerAngles(0);
  const SCALAR_T dy = derivativesEulerAngles(1);
  const SCALAR_T dx = derivativesEulerAngles(2);

  Eigen::Matrix<SCALAR_T, 3, 3> transformationMatrix;
  // clang-format off
  transformationMatrix << 0, -sin(z), cos(y)*cos(z),
                          0,  cos(z), cos(y)*sin(z),
                          1,       0,       -sin(y);
  // clang-format on

  Eigen::Matrix<SCALAR_T, 3, 3> derivativeTransformationMatrix;
  // clang-format off
  derivativeTransformationMatrix << 0, -cos(z)*dz, (-sin(y)*dy)*cos(z) + cos(y)*(-sin(z)*dz),
                                    0, -sin(z)*dz,  (-sin(y)*dy)*sin(z) + cos(y)*(cos(z)*dz),
                                    0,          0,                                -cos(y)*dy;
  // clang-format on

  angularAcceleration = derivativeTransformationMatrix * derivativesEulerAngles + transformationMatrix * secondDerivativesEulerAngles;
}

template <typename SCALAR_T>
void getEulerAnglesZyxDerivativesFromLocalAngularVelocities(const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAngles,
                                                            const Eigen::Matrix<SCALAR_T, 3, 1>& angularVelocity,
                                                            Eigen::Matrix<SCALAR_T, 3, 1>& derivativesEulerAngles) {
  const SCALAR_T z = eulerAngles(0);
  const SCALAR_T y = eulerAngles(1);
  const SCALAR_T x = eulerAngles(2);

  // Note: This matrix is ill-defined only for y = pi/2, but for anymal's base this is the case when it is perpendicular to the ground,
  // which never happens
  Eigen::Matrix<SCALAR_T, 3, 3> transformationMatrix;

  // clang-format off
  transformationMatrix << 0,    sin(x)/cos(y),   cos(x)/cos(y),
                          0,        cos(x),       -sin(x),
                          1, sin(x)*sin(y)/cos(y), cos(x)*sin(y)/cos(y);
  // clang-format on

  derivativesEulerAngles = transformationMatrix * angularVelocity;
}

template <typename SCALAR_T>
void getEulerAnglesZyxDerivativesFromGlobalAngularVelocities(const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAngles,
                                                             const Eigen::Matrix<SCALAR_T, 3, 1>& angularVelocity,
                                                             Eigen::Matrix<SCALAR_T, 3, 1>& derivativesEulerAngles) {
  const SCALAR_T z = eulerAngles(0);
  const SCALAR_T y = eulerAngles(1);
  const SCALAR_T x = eulerAngles(2);

  // Note: This matrix is ill-defined only for y = pi/2, but for anymal's base this is the case when it is perpendicular to the ground,
  // which never happens
  Eigen::Matrix<SCALAR_T, 3, 3> transformationMatrix;

  // clang-format off
  transformationMatrix << cos(z)*sin(y)/cos(y),    sin(y)*sin(z)/cos(y),   1,
                          -sin(z),        cos(z),       0,
                          cos(z)/cos(y), sin(z)/cos(y), 0;
  // clang-format on

  derivativesEulerAngles = transformationMatrix * angularVelocity;
}

template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 3> getMappingFromEulerAnglesZyxDerivativeToGlobalAngularVelocities(
    const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAngles) {
  const SCALAR_T z = eulerAngles(0);
  const SCALAR_T y = eulerAngles(1);
  const SCALAR_T x = eulerAngles(2);
  Eigen::Matrix<SCALAR_T, 3, 3> transformationMatrix;
  // clang-format off
  transformationMatrix << SCALAR_T(0),     -sin(z),        cos(y)*cos(z),
                          SCALAR_T(0),      cos(z),        cos(y)*sin(z),
                          SCALAR_T(1),     SCALAR_T(0),      -sin(y);
  // clang-format on

  return transformationMatrix;
}

}  // namespace ocs2
