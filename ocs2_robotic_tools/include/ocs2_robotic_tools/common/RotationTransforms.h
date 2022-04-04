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

// CppAD
#include <ocs2_core/automatic_differentiation/Types.h>

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
  return Eigen::AngleAxis<SCALAR_T>(eulerAnglesZyx(0), Eigen::Matrix<SCALAR_T, 3, 1>::UnitZ()) *
         Eigen::AngleAxis<SCALAR_T>(eulerAnglesZyx(1), Eigen::Matrix<SCALAR_T, 3, 1>::UnitY()) *
         Eigen::AngleAxis<SCALAR_T>(eulerAnglesZyx(2), Eigen::Matrix<SCALAR_T, 3, 1>::UnitX());
  // clang-format on
}

/**
 * Compute the rotation matrix corresponding to euler angles zyx
 *
 * @param [in] eulerAnglesZyx
 * @return The corresponding rotation matrix
 */
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
  rotationMatrix << c1 * c2,      c1 * s2 * s3 - s1 * c3,       c1 * s2 * c3 + s1 * s3,
                    s1 * c2,      s1 * s2 * s3 + c1 * c3,       s1 * s2 * c3 - c1 * s3,
                      -s2,                c2 * s3,                      c2 * c3;
  // clang-format on
  return rotationMatrix;
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

/**
 * Compute a quaternion from a matrix
 *
 * @param [in] R: Rotation Matrix.
 * @return A quaternion representing an equivalent rotation to R.
 */
template <typename SCALAR_T>
Eigen::Quaternion<SCALAR_T> matrixToQuaternion(const Eigen::Matrix<SCALAR_T, 3, 3>& R) {
  return Eigen::Quaternion<SCALAR_T>(R);
}

/**
 * Compute a quaternion from a matrix, specialized for CppAd (the default Eigen implementation uses branches)
 *
 * @param [in] R: Rotation Matrix templated on ad_scalar_t.
 * @return A quaternion representing an equivalent rotation to R.
 */
Eigen::Quaternion<ad_scalar_t> matrixToQuaternion(const Eigen::Matrix<ad_scalar_t, 3, 3>& R);

/**
 * Returns the logarithmic map of the rotation
 *      w = theta * n = log(R);
 *
 * Will find an angle, theta, in the interval [0, pi]
 *
 * To make the computation numerically stable and compatible with autodiff, we have to switch between 3 cases.
 * - For most angles we use the logarithmic map directly
 * - For angles close to 0.0, we use the taylor expansion
 * - For angles close to PI, we use a quaternion based solution.
 *
 * @tparam SCALAR_T : numeric type
 * @param rotationMatrix : 3x3 rotation matrix
 * @return 3x1 rotation vector, theta * n, with theta equal to the rotation angle, and n equal to the rotation axis.
 */
template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 1> rotationMatrixToRotationVector(const Eigen::Matrix<SCALAR_T, 3, 3>& rotationMatrix) {
  // Helper function to select a 3d vector compatible with CppAd
  auto selectSolutionGt = [](SCALAR_T left, SCALAR_T right, const Eigen::Matrix<SCALAR_T, 3, 1>& if_true,
                             const Eigen::Matrix<SCALAR_T, 3, 1>& if_false) -> Eigen::Matrix<SCALAR_T, 3, 1> {
    return {CppAD::CondExpGt(left, right, if_true[0], if_false[0]), CppAD::CondExpGt(left, right, if_true[1], if_false[1]),
            CppAD::CondExpGt(left, right, if_true[2], if_false[2])};
  };
  const auto& R = rotationMatrix;

  const SCALAR_T trace = R(0, 0) + R(1, 1) + R(2, 2);
  const Eigen::Matrix<SCALAR_T, 3, 1> skewVector(R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1));

  // Tolerance to select alternative solution near singularity
  const SCALAR_T eps(1e-8);

  // Rotation close to zero -> use taylor expansion, use when trace > 3.0 - eps
  const Eigen::Matrix<SCALAR_T, 3, 1> taylorExpansionSol = (SCALAR_T(0.5) - (trace - SCALAR_T(3.0)) / SCALAR_T(12.0)) * skewVector;

  // Normal rotation, use normal logarithmic map
  const SCALAR_T tmp = SCALAR_T(0.5) * (trace - SCALAR_T(1.0));
  const SCALAR_T theta = acos(tmp);
  const Eigen::Matrix<SCALAR_T, 3, 1> normalSol = (SCALAR_T(0.5) * theta / sqrt(SCALAR_T(1.0) - tmp * tmp)) * skewVector;

  // Quaternion solution, when close to pi, use when trace < -1.0 + eps
  auto q = ocs2::matrixToQuaternion(R);

  // Correct sign to make qw positive
  q.vec() = selectSolutionGt(q.w(), SCALAR_T(0.0), q.vec(), -q.vec());
  q.w() = CppAD::CondExpGt(q.w(), SCALAR_T(0.0), q.w(), -q.w());

  // Norm of vector part of the quaternion. Compute from trace to avoid squaring element and losing precision
  const SCALAR_T qVecNorm = SCALAR_T(0.5) * sqrt(SCALAR_T(3.0) - trace);

  Eigen::Matrix<SCALAR_T, 3, 1> quaternionSol = SCALAR_T(4.0) * atan(qVecNorm / (q.w() + SCALAR_T(1.0))) * q.vec() / qVecNorm;

  // Select solution
  const SCALAR_T smallAngleThreshold = SCALAR_T(3.0) - eps;   // select taylorExpansionSol if trace > 3 - eps
  const SCALAR_T largeAngleThreshold = -SCALAR_T(1.0) + eps;  // select quaternionSol if trace < -1.0 + eps
  return selectSolutionGt(trace, largeAngleThreshold, selectSolutionGt(trace, smallAngleThreshold, taylorExpansionSol, normalSol),
                          quaternionSol);
}

/**
 * Computes a rotation error as a 3D vector in world frame. This 3D vector is the angle * axis representation of the rotation error.
 * This operation is also known as "box-minus": error = lhs [-] rhs
 *
 * Example usage:
 *    rotationErrorInWorld = rotationErrorInWorld(rotationBaseMeasuredToWorld, rotationBaseReferenceToWorld)
 *
 * @tparam SCALAR_T : numerical type
 * @param rotationMatrixLhs : rotation from lhs frame to world
 * @param rotationMatrixRhs : rotation from rhs frame to world
 * @return error = lhs [-] rhs
 */
template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 1> rotationErrorInWorld(const Eigen::Matrix<SCALAR_T, 3, 3>& rotationMatrixLhs,
                                                   const Eigen::Matrix<SCALAR_T, 3, 3>& rotationMatrixRhs) {
  /* Note that this error (W_R_lhs * rhs_R_W) does not follow the usual concatination of rotations.
   * It follows from simplifying:
   *    errorInWorld = W_R_lhs * angleAxis(rhs_R_W * W_R_lhs)
   *                 = angleAxis(W_R_lhs * rhs_R_W * W_R_lhs * lhs_R_W)
   *                 = angleAxis(W_R_lhs * rhs_R_W)
   */
  const Eigen::Matrix<SCALAR_T, 3, 3> rotationErrorInWorld = rotationMatrixLhs * rotationMatrixRhs.transpose();
  return rotationMatrixToRotationVector(rotationErrorInWorld);
}

/**
 * The returned rotation error is expressed in the local frame (lhs or rhs).
 * Because the rotation error rotates between the lhs and rhs frame, its representation in both frames is numerically identical.
 *
 * Example usage:
 *    rotationErrorInBase = rotationErrorInLocal(rotationBaseMeasuredToWorld, rotationBaseReferenceToWorld)
 *
 * with the note that rotationErrorInBaseMeasured = rotationErrorInBaseReference
 *
 * See rotationErrorInWorld for further explanation.
 * @tparam SCALAR_T : numerical type
 * @param rotationMatrixLhs : rotation from lhs frame to world
 * @param rotationMatrixRhs : rotation from rhs frame to world
 * @return error = lhs [-] rhs
 */
template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 1> rotationErrorInLocal(const Eigen::Matrix<SCALAR_T, 3, 3>& rotationMatrixLhs,
                                                   const Eigen::Matrix<SCALAR_T, 3, 3>& rotationMatrixRhs) {
  const Eigen::Matrix<SCALAR_T, 3, 3> rotationErrorInLocal = rotationMatrixRhs.transpose() * rotationMatrixLhs;
  return rotationMatrixToRotationVector(rotationErrorInLocal);
}

}  // namespace ocs2