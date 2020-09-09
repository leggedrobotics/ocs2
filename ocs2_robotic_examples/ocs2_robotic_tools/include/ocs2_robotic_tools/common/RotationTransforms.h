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

#include <ocs2_core/automatic_differentiation/CppAdInterface.h>

namespace ocs2 {

/**
 * Compute the quaternion distance measure
 *
 * @param [in] q: measured end effector quaternion.
 * @param [in] qRef: desired end effector quaternion.
 * @return A 3x1 vector representing the quaternion distance.
 * In particular, if Qd and Qe are the desired and the measured end-effector
 * quaternions, the measured and desired frames are aligned if this vector is 0.
 */
template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 1> quaternionDistance(const Eigen::Quaternion<SCALAR_T>& q, const Eigen::Quaternion<SCALAR_T>& qRef) {
  return q.w() * qRef.vec() - qRef.w() * q.vec() + q.vec().cross(qRef.vec());
}

/**
 * Compute the quaternion distance measure jacobian
 *
 * @param [in] q: measured end effector quaternion.
 * @param [in] qRef: desired end effector quaternion.
 * @return A 3x4 matrix representing the quaternion distance jacobian.
 * The columns are the partial derivatives of [q.x, q.y, q,z, qw] (default Eigen order)
 */
template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 4> quaternionDistanceJacobian(const Eigen::Quaternion<SCALAR_T>& q, const Eigen::Quaternion<SCALAR_T>& qRef) {
  Eigen::Matrix<SCALAR_T, 3, 4> jacobian;
  // clang-format off
  jacobian << -qRef.w(), qRef.z(), -qRef.y(), qRef.x(),
              -qRef.z(), -qRef.w(), qRef.x(), qRef.y(),
              qRef.y(), -qRef.x(), -qRef.w(), qRef.z();
  // clang-format on
  return jacobian;
}

/**
 * Compute the quaternion corresponding to euler angles zyx
 *
 * @param [in] eulerAnglesZyx
 * @return The corresponding quaternion
 */
template <typename SCALAR_T>
Eigen::Quaternion<SCALAR_T> getQuaternionFromEulerAnglesZyx(const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAnglesZyx) {
  // clang-format off
  return Eigen::AngleAxis<SCALAR_T>(eulerAnglesZyx(0), Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxis<SCALAR_T>(eulerAnglesZyx(1), Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxis<SCALAR_T>(eulerAnglesZyx(2), Eigen::Vector3d::UnitX());
  // clang-format on
}

/**
 * @brief Transform a set of Euler Angles (each in [-pi, pi)) into Euler Angles in the range [-pi,pi),[-pi/2,pi/2),[-pi,pi)
 * @param[in,out] Reference to eulerAngles XYZ or ZYX which will be modified in place
 * @note Code taken from https://github.com/ANYbotics/kindr/blob/master/include/kindr/rotations/EulerAnglesXyz.hpp
 * @note Works for Euler Angles XYZ and ZYX alike
 */
template <typename SCALAR_T>
void makeEulerAnglesUnique(Eigen::Matrix<SCALAR_T, 3, 1>& eulerAngles) {
  SCALAR_T tol(1e-9);  // FIXME(jcarius) magic number
  SCALAR_T pi(M_PI);

  if (eulerAngles.y() < -pi / 2 - tol) {
    if (eulerAngles.x() < 0) {
      eulerAngles.x() = eulerAngles.x() + pi;
    } else {
      eulerAngles.x() = eulerAngles.x() - pi;
    }

    eulerAngles.y() = -(eulerAngles.y() + pi);

    if (eulerAngles.z() < 0) {
      eulerAngles.z() = eulerAngles.z() + pi;
    } else {
      eulerAngles.z() = eulerAngles.z() - pi;
    }
  } else if (-pi / 2 - tol <= eulerAngles.y() && eulerAngles.y() <= -pi / 2 + tol) {
    eulerAngles.x() -= eulerAngles.z();
    eulerAngles.z() = 0;
  } else if (-pi / 2 + tol < eulerAngles.y() && eulerAngles.y() < pi / 2 - tol) {
    // ok
  } else if (pi / 2 - tol <= eulerAngles.y() && eulerAngles.y() <= pi / 2 + tol) {
    // todo: pi/2 should not be in range, other formula?
    eulerAngles.x() += eulerAngles.z();
    eulerAngles.z() = 0;
  } else  // pi/2 + tol < eulerAngles.y()
  {
    if (eulerAngles.x() < 0) {
      eulerAngles.x() = eulerAngles.x() + pi;
    } else {
      eulerAngles.x() = eulerAngles.x() - pi;
    }

    eulerAngles.y() = -(eulerAngles.y() - pi);

    if (eulerAngles.z() < 0) {
      eulerAngles.z() = eulerAngles.z() + pi;
    } else {
      eulerAngles.z() = eulerAngles.z() - pi;
    }
  }
}

template <typename SCALAR_T>
inline Eigen::Quaternion<SCALAR_T> matrixToQuaternion(const Eigen::Matrix<SCALAR_T, 3, 3>& R) {
  return Eigen::Quaternion<SCALAR_T>(R);
}

template <>
inline Eigen::Quaternion<CppAdInterface::ad_scalar_t> matrixToQuaternion(const Eigen::Matrix<CppAdInterface::ad_scalar_t, 3, 3>& R) {
  using ad_scalar_t = CppAdInterface::ad_scalar_t;
  ad_scalar_t t1, t2, t;
  ad_scalar_t x1, x2, x;
  ad_scalar_t y1, y2, y;
  ad_scalar_t z1, z2, z;
  ad_scalar_t w1, w2, w;

  t1 = CppAD::CondExpGt(R(0, 0), R(1, 1), 1 + R(0, 0) - R(1, 1) - R(2, 2), 1 - R(0, 0) + R(1, 1) - R(2, 2));
  t2 = CppAD::CondExpLt(R(0, 0), -R(1, 1), 1 - R(0, 0) - R(1, 1) + R(2, 2), 1 + R(0, 0) + R(1, 1) + R(2, 2));
  t = CppAD::CondExpLt(R(2, 2), ad_scalar_t(0.0), t1, t2);

  x1 = CppAD::CondExpGt(R(0, 0), R(1, 1), t, R(1, 0) + R(0, 1));
  x2 = CppAD::CondExpLt(R(0, 0), -R(1, 1), R(0, 2) + R(2, 0), R(2, 1) - R(1, 2));
  x = CppAD::CondExpLt(R(2, 2), ad_scalar_t(0.0), x1, x2);

  y1 = CppAD::CondExpGt(R(0, 0), R(1, 1), R(1, 0) + R(0, 1), t);
  y2 = CppAD::CondExpLt(R(0, 0), -R(1, 1), R(2, 1) + R(1, 2), R(0, 2) - R(2, 0));
  y = CppAD::CondExpLt(R(2, 2), ad_scalar_t(0.0), y1, y2);

  z1 = CppAD::CondExpGt(R(0, 0), R(1, 1), R(0, 2) + R(2, 0), R(2, 1) + R(1, 2));
  z2 = CppAD::CondExpLt(R(0, 0), -R(1, 1), t, R(1, 0) - R(0, 1));
  z = CppAD::CondExpLt(R(2, 2), ad_scalar_t(0.0), z1, z2);

  w1 = CppAD::CondExpGt(R(0, 0), R(1, 1), R(2, 1) - R(1, 2), R(0, 2) - R(2, 0));
  w2 = CppAD::CondExpLt(R(0, 0), -R(1, 1), R(1, 0) - R(0, 1), t);
  w = CppAD::CondExpLt(R(2, 2), ad_scalar_t(0.0), w1, w2);

  Eigen::Matrix<ad_scalar_t, 4, 1> q({x, y, z, w});
  q *= 0.5 / sqrt(t);

  Eigen::Quaternion<ad_scalar_t> quaternion;
  quaternion.x() = q(0);
  quaternion.y() = q(1);
  quaternion.z() = q(2);
  quaternion.w() = q(3);

  return quaternion;
}
}  // namespace ocs2
