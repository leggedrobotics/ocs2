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
 * Get the skew symmetric representation of a vector
 *
 * v_hat = [0.0, -v.z(), v.y(),
 *          v.z(), 0.0, -v.x(),
 *          -v.y(), v.x(), 0.0];
 *
 * @param [in] vector 3x1
 * @param [out] matrix 3x3
 */
template <typename SCALAR_T>
void getSkewMatrix(const Eigen::Matrix<SCALAR_T, 3, 1>& v, Eigen::Matrix<SCALAR_T, 3, 3>& skewSymmetricMatrix) {
  skewSymmetricMatrix << SCALAR_T(0.0), -v.z(), v.y(), v.z(), SCALAR_T(0.0), -v.x(), -v.y(), v.x(), SCALAR_T(0.0);
}

template <typename SCALAR_T>
void getQuaternionFromEulerAnglesZyx(const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAngles, Eigen::Quaternion<SCALAR_T>& quaternion) {
  const SCALAR_T yaw = eulerAngles(0);
  const SCALAR_T pitch = eulerAngles(1);
  const SCALAR_T roll = eulerAngles(2);

  const SCALAR_T cy = cos(yaw * 0.5);
  const SCALAR_T sy = sin(yaw * 0.5);
  const SCALAR_T cp = cos(pitch * 0.5);
  const SCALAR_T sp = sin(pitch * 0.5);
  const SCALAR_T cr = cos(roll * 0.5);
  const SCALAR_T sr = sin(roll * 0.5);

  quaternion.w() = cy * cp * cr + sy * sp * sr;
  quaternion.x() = cy * cp * sr - sy * sp * cr;
  quaternion.y() = sy * cp * sr + cy * sp * cr;
  quaternion.z() = sy * cp * cr - cy * sp * sr;
}

template <typename SCALAR_T>
Eigen::Quaternion<SCALAR_T> getQuaternionFromEulerAnglesZyx(const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAnglesZyx) {
  const auto yaw = scalar_t(eulerAnglesZyx(0));
  const auto pitch = scalar_t(eulerAnglesZyx(1));
  const auto roll = scalar_t(eulerAnglesZyx(2));
  Eigen::Quaternion<scalar_t> quat = Eigen::AngleAxisd(yaw, Eigen::Matrix<scalar_t, 3, 1>::UnitZ()) *
                                     Eigen::AngleAxisd(pitch, Eigen::Matrix<scalar_t, 3, 1>::UnitY()) *
                                     Eigen::AngleAxisd(roll, Eigen::Matrix<scalar_t, 3, 1>::UnitX());
  return quat.template cast<SCALAR_T>();
}

template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 1> quaternionDistance(const Eigen::Quaternion<SCALAR_T>& eeQuaternion,
                                                 const Eigen::Quaternion<SCALAR_T>& commandQuaternion) {
  return (eeQuaternion.w() * commandQuaternion.vec() - commandQuaternion.w() * eeQuaternion.vec() -
          commandQuaternion.vec().cross(eeQuaternion.vec()));
}

template <typename SCALAR_T>
Eigen::Quaternion<SCALAR_T> matrixToQuaternion(const Eigen::Matrix<SCALAR_T, 3, 3>& R) {
  SCALAR_T t1, t2, t;
  SCALAR_T x1, x2, x;
  SCALAR_T y1, y2, y;
  SCALAR_T z1, z2, z;
  SCALAR_T w1, w2, w;

  t1 = CppAD::CondExpGt(R(0, 0), R(1, 1), 1 + R(0, 0) - R(1, 1) - R(2, 2), 1 - R(0, 0) + R(1, 1) - R(2, 2));
  t2 = CppAD::CondExpLt(R(0, 0), -R(1, 1), 1 - R(0, 0) - R(1, 1) + R(2, 2), 1 + R(0, 0) + R(1, 1) + R(2, 2));
  t = CppAD::CondExpLt(R(2, 2), SCALAR_T(0.0), t1, t2);

  x1 = CppAD::CondExpGt(R(0, 0), R(1, 1), t, R(1, 0) + R(0, 1));
  x2 = CppAD::CondExpLt(R(0, 0), -R(1, 1), R(0, 2) + R(2, 0), R(2, 1) - R(1, 2));
  x = CppAD::CondExpLt(R(2, 2), SCALAR_T(0.0), x1, x2);

  y1 = CppAD::CondExpGt(R(0, 0), R(1, 1), R(1, 0) + R(0, 1), t);
  y2 = CppAD::CondExpLt(R(0, 0), -R(1, 1), R(2, 1) + R(1, 2), R(0, 2) - R(2, 0));
  y = CppAD::CondExpLt(R(2, 2), SCALAR_T(0.0), y1, y2);

  z1 = CppAD::CondExpGt(R(0, 0), R(1, 1), R(0, 2) + R(2, 0), R(2, 1) + R(1, 2));
  z2 = CppAD::CondExpLt(R(0, 0), -R(1, 1), t, R(1, 0) - R(0, 1));
  z = CppAD::CondExpLt(R(2, 2), SCALAR_T(0.0), z1, z2);

  w1 = CppAD::CondExpGt(R(0, 0), R(1, 1), R(2, 1) - R(1, 2), R(0, 2) - R(2, 0));
  w2 = CppAD::CondExpLt(R(0, 0), -R(1, 1), R(1, 0) - R(0, 1), t);
  w = CppAD::CondExpLt(R(2, 2), SCALAR_T(0.0), w1, w2);

  Eigen::Matrix<SCALAR_T, 4, 1> q({x, y, z, w});
  q *= 0.5 / sqrt(t);

  Eigen::Quaternion<SCALAR_T> quaternion;
  quaternion.x() = q(0);
  quaternion.y() = q(1);
  quaternion.z() = q(2);
  quaternion.w() = q(3);
  return quaternion;
}

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
