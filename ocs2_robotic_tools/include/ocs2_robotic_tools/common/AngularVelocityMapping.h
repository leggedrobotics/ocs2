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

#include <Eigen/Core>
#include <array>
#include <cmath>

namespace ocs2 {

/**
 * Compute the mapping from angular velocity w to rotation quaternion time derivative q_dot.
 *
 * @param [in]: q: orientation quaternion
 * @return mapping matrix E_R inverse, mapping angular velocity to quaternion time derivative.
 *         The colums are in the order [x, y, z, w] (default Eigen order as in q.coeffs()).
 */
template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 4, 3> angularVelocityToQuaternionTimeDerivative(const Eigen::Quaternion<SCALAR_T>& q) {
  const SCALAR_T qx = SCALAR_T(0.5) * q.x();
  const SCALAR_T qy = SCALAR_T(0.5) * q.y();
  const SCALAR_T qz = SCALAR_T(0.5) * q.z();
  const SCALAR_T qw = SCALAR_T(0.5) * q.w();
  Eigen::Matrix<SCALAR_T, 4, 3> M;
  // clang-format off
  // Robot Dynamics 2018, equation (2.97)
  M <<  qw,  qz, -qy,
       -qz,  qw,  qx,
        qy, -qx,  qw,
       -qx, -qy, -qz;
  // clang-format on
  return M;
}

/**
 * @brief Computes the matrix which transforms derivatives of angular velocities in the body frame to euler angles derivatives
 * WARNING: matrix is singular when rotation around y axis is +/- 90 degrees
 *
 * @param[in] eulerAngles: euler angles in XYZ convention
 * @return M: matrix that does the transformation
 */
template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 3> getMappingFromLocalAngularVelocityToEulerAnglesXyzDerivative(
    const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAngles) {
  const SCALAR_T sy = sin(eulerAngles(1));
  const SCALAR_T cy = cos(eulerAngles(1));
  const SCALAR_T sz = sin(eulerAngles(2));
  const SCALAR_T cz = cos(eulerAngles(2));
  const SCALAR_T cz_cy = cz / cy;
  const SCALAR_T sz_cy = sz / cy;

  Eigen::Matrix<SCALAR_T, 3, 3> M;
  // clang-format off
  M <<      cz_cy,       -sz_cy,   SCALAR_T(0.0),
               sz,           cz,   SCALAR_T(0.0),
      -sy * cz_cy,   sy * sz_cy,   SCALAR_T(1.0);
  // clang-format on

  return M;
}

/**
 * to map local angular velocity \omega_W expressed in body coordinates, to changes in Euler Angles expressed in an inertial frame q_I
 * we have to map them via \dot{q}_I = H \omega_W, where H is the matrix defined in kindr getMappingFromLocalAngularVelocityToDiff.
 * You can see the kindr cheat sheet to figure out how to build this matrix. The following code computes the Jacobian of \dot{q}_I
 * with respect to \q_I and \omega_W. Thus the lower part of the Jacobian is H and the upper part is dH/dq_I \omega_W. We include
 * both parts for more efficient computation. The following code is computed using auto-diff.
 * @param eulerAnglesXyz
 * @param angularVelocity
 * @return
 */
template <typename SCALAR_T>
inline Eigen::Matrix<SCALAR_T, 6, 3> JacobianOfAngularVelocityMapping(const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAnglesXyz,
                                                                      const Eigen::Matrix<SCALAR_T, 3, 1>& angularVelocity) {
  using std::cos;
  using std::sin;

  Eigen::Matrix<SCALAR_T, 6, 1> x;
  x << eulerAnglesXyz, angularVelocity;

  std::array<SCALAR_T, 10> v;

  Eigen::Matrix<SCALAR_T, 6, 3> jac;
  Eigen::Map<Eigen::Matrix<SCALAR_T, 3 * 6, 1>> y(jac.data());

#pragma clang diagnostic push
#pragma clang diagnostic ignored "cppcoreguidelines-pro-bounds-pointer-arithmetic"
  y[9] = sin(x[2]);
  y[10] = cos(x[2]);
  v[0] = cos(x[1]);
  v[1] = 1 / v[0];
  y[3] = v[1] * y[10];
  v[2] = sin(x[1]);
  y[1] = 0 - (0 - (0 - x[4] * y[9] + x[3] * y[10]) * 1 / v[0] * v[1]) * v[2];
  v[3] = sin(x[2]);
  v[4] = 0 - v[1];
  y[4] = v[4] * y[9];
  v[5] = y[10];
  y[2] = 0 - x[3] * v[1] * v[3] + x[4] * v[4] * v[5];
  y[8] = 0 - x[4] * v[3] + x[3] * v[5];
  v[6] = v[1] * y[9];
  v[7] = v[4] * y[10];
  v[8] = v[2];
  y[15] = v[7] * v[8];
  y[16] = v[6] * v[8];
  v[9] = x[4] * v[8];
  v[8] = x[3] * v[8];
  y[13] = (x[4] * v[6] + x[3] * v[7]) * v[0] - (0 - (v[9] * y[9] - v[8] * y[10]) * 1 / v[0] * v[1]) * v[2];
  y[14] = 0 - v[8] * v[4] * v[3] + v[9] * v[1] * v[5];
  // dependent variables without operations
  y[0] = 0;
  y[5] = 0;
  y[6] = 0;
  y[7] = 0;
  y[11] = 0;
  y[12] = 0;
  y[17] = 1;
#pragma clang diagnostic pop

  return jac;
}

}  // namespace ocs2
