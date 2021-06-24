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

#pragma once

#include <Eigen/Core>
#include <array>
#include <cmath>

namespace ocs2 {

/**
 * Compute the matrix that maps derivatives of ZYX-Euler angles to global angular velocities
 *
 * @param [in] eulerAngles: ZYX-Euler angles
 * @return 3x3 matrix mapping
 */
template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 3> getMappingFromEulerAnglesZyxDerivativeToGlobalAngularVelocity(
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

/**
 * Compute angular velocities expressed in the world frame from derivatives of ZYX-Euler angles
 *
 * @param [in] eulerAngles: ZYX-Euler angles
 * @param [in] derivativesEulerAngles: time-derivative of ZYX-Euler angles
 * @return angular velocity expressed in world frame
 */
template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 1> getGlobalAngularVelocityFromEulerAnglesZyxDerivatives(
    const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAngles, const Eigen::Matrix<SCALAR_T, 3, 1>& derivativesEulerAngles) {
  Eigen::Matrix<SCALAR_T, 3, 3> transformationMatrix = getMappingFromEulerAnglesZyxDerivativeToGlobalAngularVelocity(eulerAngles);
  return transformationMatrix * derivativesEulerAngles;
}

/**
 * Compute angular accelerations expressed in the world frame from derivatives of ZYX-Euler angles
 *
 * @param [in] eulerAngles: ZYX-Euler angles
 * @param [in] derivativesEulerAngles: time-derivative of ZYX-Euler angles
 * @param [in] secondDerivativesEulerAngles: second order time-derivative of ZYX-Euler angles
 * @return angular acceleration expressed in world frame
 */
template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 1> getGlobalAngularAccelerationFromEulerAnglesZyxDerivatives(
    const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAngles, const Eigen::Matrix<SCALAR_T, 3, 1>& derivativesEulerAngles,
    const Eigen::Matrix<SCALAR_T, 3, 1>& secondDerivativesEulerAngles) {
  const SCALAR_T z = eulerAngles(0);
  const SCALAR_T y = eulerAngles(1);
  const SCALAR_T x = eulerAngles(2);
  const SCALAR_T dz = derivativesEulerAngles(0);
  const SCALAR_T dy = derivativesEulerAngles(1);
  const SCALAR_T dx = derivativesEulerAngles(2);
  Eigen::Matrix<SCALAR_T, 3, 3> transformationMatrix = getMappingFromEulerAnglesZyxDerivativeToGlobalAngularVelocity(eulerAngles);
  Eigen::Matrix<SCALAR_T, 3, 3> derivativeTransformationMatrix;
  // clang-format off
  derivativeTransformationMatrix << SCALAR_T(0), -cos(z)*dz, (-sin(y)*dy)*cos(z) + cos(y)*(-sin(z)*dz),
                                    SCALAR_T(0), -sin(z)*dz,  (-sin(y)*dy)*sin(z) + cos(y)*(cos(z)*dz),
                                    SCALAR_T(0),  SCALAR_T(0),   -cos(y)*dy;
  // clang-format on
  return derivativeTransformationMatrix * derivativesEulerAngles + transformationMatrix * secondDerivativesEulerAngles;
}

/**
 * Compute derivatives of ZYX-Euler angles from local angular velocities
 *
 * @param [in] eulerAngles: ZYX-Euler angles
 * @param [in] angularVelocity: angular velocity expressed in local frame
 * @return derivatives of ZYX-Euler angles
 */
template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 1> getEulerAnglesZyxDerivativesFromLocalAngularVelocity(const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAngles,
                                                                                   const Eigen::Matrix<SCALAR_T, 3, 1>& angularVelocity) {
  const SCALAR_T z = eulerAngles(0);
  const SCALAR_T y = eulerAngles(1);
  const SCALAR_T x = eulerAngles(2);
  // Note: This matrix is ill-defined only for y = pi/2, but for anymal's base this is the case when it is perpendicular to the ground,
  // which never happens
  Eigen::Matrix<SCALAR_T, 3, 3> transformationMatrix;
  // clang-format off
  transformationMatrix << SCALAR_T(0),    sin(x)/cos(y),   cos(x)/cos(y),
                          SCALAR_T(0),        cos(x),       -sin(x),
                          SCALAR_T(1), sin(x)*sin(y)/cos(y), cos(x)*sin(y)/cos(y);
  // clang-format on
  return transformationMatrix * angularVelocity;
}

/**
 * Compute derivatives of ZYX-Euler angles from global angular velocities
 *
 * @param [in] eulerAngles: ZYX-Euler angles
 * @param [in] angularVelocity: angular velocity expressed in world frame
 * @return derivatives of ZYX-Euler angles
 */
template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 1> getEulerAnglesZyxDerivativesFromGlobalAngularVelocity(const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAngles,
                                                                                    const Eigen::Matrix<SCALAR_T, 3, 1>& angularVelocity) {
  const SCALAR_T z = eulerAngles(0);
  const SCALAR_T y = eulerAngles(1);
  const SCALAR_T x = eulerAngles(2);
  // Note: This matrix is ill-defined only for y = pi/2, but for anymal's base this is the case when it is perpendicular to the ground,
  // which never happens
  Eigen::Matrix<SCALAR_T, 3, 3> transformationMatrix;
  // clang-format off
  transformationMatrix << cos(z)*sin(y)/cos(y),    sin(y)*sin(z)/cos(y),   SCALAR_T(1),
                              -sin(z),                     cos(z),         SCALAR_T(0),
                           cos(z)/cos(y),               sin(z)/cos(y),     SCALAR_T(0);
  // clang-format on
  return transformationMatrix * angularVelocity;
}

}  // namespace ocs2