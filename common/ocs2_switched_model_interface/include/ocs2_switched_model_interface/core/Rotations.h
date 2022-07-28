//
// Created by rgrandia on 22.10.19.
//

#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>

#include <ocs2_robotic_tools/common/RotationTransforms.h>

namespace switched_model {

/**
 * Base to origin rotation matrix
 * @tparam Derived
 * @param [in] eulerAngles
 * @return
 */
template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 3> rotationMatrixBaseToOrigin(const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAnglesXYZ) {
  // Generated code for:
  //  // inputs are the intrinsic rotation angles in RADIANTS
  //  SCALAR_T sinAlpha = sin(eulerAngles(0));
  //  SCALAR_T cosAlpha = cos(eulerAngles(0));
  //  SCALAR_T sinBeta = sin(eulerAngles(1));
  //  SCALAR_T cosBeta = cos(eulerAngles(1));
  //  SCALAR_T sinGamma = sin(eulerAngles(2));
  //  SCALAR_T cosGamma = cos(eulerAngles(2));
  //
  //  Eigen::Matrix<SCALAR_T, 3, 3> Rx, Ry, Rz;
  //  Rx << SCALAR_T(1), SCALAR_T(0), SCALAR_T(0), SCALAR_T(0), cosAlpha, -sinAlpha, SCALAR_T(0), sinAlpha, cosAlpha;
  //  Ry << cosBeta, SCALAR_T(0), sinBeta, SCALAR_T(0), SCALAR_T(1), SCALAR_T(0), -sinBeta, SCALAR_T(0), cosBeta;
  //  Rz << cosGamma, -sinGamma, SCALAR_T(0), sinGamma, cosGamma, SCALAR_T(0), SCALAR_T(0), SCALAR_T(0), SCALAR_T(1);
  //
  //  return Rx * Ry * Rz;
  Eigen::Matrix<SCALAR_T, 3, 3> o_R_b;

  // auxiliary variables
  std::array<SCALAR_T, 8> v{};

  v[0] = cos(eulerAnglesXYZ[1]);
  v[1] = cos(eulerAnglesXYZ[2]);
  o_R_b(0) = v[0] * v[1];
  v[2] = sin(eulerAnglesXYZ[0]);
  v[3] = -v[2];
  o_R_b(6) = sin(eulerAnglesXYZ[1]);
  v[4] = -o_R_b(6);
  v[5] = v[3] * v[4];
  v[6] = cos(eulerAnglesXYZ[0]);
  v[7] = sin(eulerAnglesXYZ[2]);
  o_R_b(1) = v[5] * v[1] + v[6] * v[7];
  v[4] = v[6] * v[4];
  o_R_b(2) = v[4] * v[1] + v[2] * v[7];
  v[7] = -v[7];
  o_R_b(3) = v[0] * v[7];
  o_R_b(4) = v[5] * v[7] + v[6] * v[1];
  o_R_b(5) = v[4] * v[7] + v[2] * v[1];
  o_R_b(7) = v[3] * v[0];
  o_R_b(8) = v[6] * v[0];
  return o_R_b;
}

template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 3> rotationMatrixOriginToBase(const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAngles) {
  return rotationMatrixBaseToOrigin<SCALAR_T>(eulerAngles).transpose();
}

template <typename SCALAR_T>
void rotateInPlace2d(Eigen::Matrix<SCALAR_T, 2, 1>& v, SCALAR_T angle) {
  const SCALAR_T c = cos(angle);
  const SCALAR_T s = sin(angle);
  const SCALAR_T vx = v.x();
  const SCALAR_T vy = v.y();
  v.x() = c * vx - s * vy;
  v.y() = s * vx + c * vy;
}

template <typename SCALAR_T>
void invRotateInPlace2d(Eigen::Matrix<SCALAR_T, 2, 1>& v, SCALAR_T angle) {
  const SCALAR_T c = cos(angle);
  const SCALAR_T s = sin(angle);
  const SCALAR_T vx = v.x();
  const SCALAR_T vy = v.y();
  v.x() = c * vx + s * vy;
  v.y() = -s * vx + c * vy;
}

template <typename SCALAR_T>
void rotateInPlaceZ(Eigen::Matrix<SCALAR_T, 3, 1>& v, SCALAR_T angle) {
  const SCALAR_T c = cos(angle);
  const SCALAR_T s = sin(angle);
  const SCALAR_T vx = v.x();
  const SCALAR_T vy = v.y();
  v.x() = c * vx - s * vy;
  v.y() = s * vx + c * vy;
}

template <typename SCALAR_T>
void invRotateInPlaceZ(Eigen::Matrix<SCALAR_T, 3, 1>& v, SCALAR_T angle) {
  const SCALAR_T c = cos(angle);
  const SCALAR_T s = sin(angle);
  const SCALAR_T vx = v.x();
  const SCALAR_T vy = v.y();
  v.x() = c * vx + s * vy;
  v.y() = -s * vx + c * vy;
}

template <typename SCALAR_T>
void rotateInPlaceY(Eigen::Matrix<SCALAR_T, 3, 1>& v, SCALAR_T angle) {
  const SCALAR_T c = cos(angle);
  const SCALAR_T s = sin(angle);
  const SCALAR_T vx = v.x();
  const SCALAR_T vz = v.z();
  v.x() = c * vx + s * vz;
  v.z() = -s * vx + c * vz;
}

template <typename SCALAR_T>
void invRotateInPlaceY(Eigen::Matrix<SCALAR_T, 3, 1>& v, SCALAR_T angle) {
  const SCALAR_T c = cos(angle);
  const SCALAR_T s = sin(angle);
  const SCALAR_T vx = v.x();
  const SCALAR_T vz = v.z();
  v.x() = c * vx - s * vz;
  v.z() = s * vx + c * vz;
}

template <typename SCALAR_T>
void rotateInPlaceX(Eigen::Matrix<SCALAR_T, 3, 1>& v, SCALAR_T angle) {
  const SCALAR_T c = cos(angle);
  const SCALAR_T s = sin(angle);
  const SCALAR_T vy = v.y();
  const SCALAR_T vz = v.z();
  v.y() = c * vy - s * vz;
  v.z() = s * vy + c * vz;
}

template <typename SCALAR_T>
void invRotateInPlaceX(Eigen::Matrix<SCALAR_T, 3, 1>& v, SCALAR_T angle) {
  const SCALAR_T c = cos(angle);
  const SCALAR_T s = sin(angle);
  const SCALAR_T vy = v.y();
  const SCALAR_T vz = v.z();
  v.y() = c * vy + s * vz;
  v.z() = -s * vy + c * vz;
}

/**
 * Directly rotates a vector from base to origin.
 *
 * Uses 12 (3*4) multiplications, and 6 (3*2) additions.
 *
 * Constructing just the rotation matrix already takes 14 multiplication, and 4 additions. Without having rotated the vector.
 * Multiplication with the rotation matrix takes 9 multiplications and 6 additions.
 * Constructing the rotation matrix could be cheaper when doing >5 rotations with the same matrix
 */
template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 1> rotateVectorBaseToOrigin(const Eigen::Matrix<SCALAR_T, 3, 1>& v,
                                                       const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAnglesXYZ) {
  Eigen::Matrix<SCALAR_T, 3, 1> rotatedVector = v;
  rotateInPlaceZ(rotatedVector, eulerAnglesXYZ[2]);
  rotateInPlaceY(rotatedVector, eulerAnglesXYZ[1]);
  rotateInPlaceX(rotatedVector, eulerAnglesXYZ[0]);
  return rotatedVector;
}

/**
 * inverse of rotateVectorBaseToOrigin, use here the invRotate helpers instead of rotate(-angle),
 * otherwise the sin/cos(angle) and sin/cos(-angle) cannot be merged by CppAd
 */
template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 1> rotateVectorOriginToBase(const Eigen::Matrix<SCALAR_T, 3, 1>& v,
                                                       const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAnglesXYZ) {
  Eigen::Matrix<SCALAR_T, 3, 1> rotatedVector = v;
  invRotateInPlaceX(rotatedVector, eulerAnglesXYZ[0]);
  invRotateInPlaceY(rotatedVector, eulerAnglesXYZ[1]);
  invRotateInPlaceZ(rotatedVector, eulerAnglesXYZ[2]);
  return rotatedVector;
}

template <typename SCALAR_T>
Eigen::Quaternion<SCALAR_T> quaternionBaseToOrigin(const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAngles) {
  return Eigen::Quaternion<SCALAR_T>(Eigen::AngleAxis<SCALAR_T>(eulerAngles(0), Eigen::Matrix<SCALAR_T, 3, 1>::UnitX()) *
                                     Eigen::AngleAxis<SCALAR_T>(eulerAngles(1), Eigen::Matrix<SCALAR_T, 3, 1>::UnitY()) *
                                     Eigen::AngleAxis<SCALAR_T>(eulerAngles(2), Eigen::Matrix<SCALAR_T, 3, 1>::UnitZ()));
}

template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 1> eulerAnglesFromQuaternionBaseToOrigin(const Eigen::Quaternion<SCALAR_T>& q_origin_base) {
  return q_origin_base.toRotationMatrix().eulerAngles(0, 1, 2);
}

/**
 * Calculates the skew matrix for vector cross product
 * @tparam Derived
 * @param [in] in
 * @return
 */
template <typename SCALAR_T, typename Derived>
Eigen::Matrix<SCALAR_T, 3, 3> crossProductMatrix(const Eigen::DenseBase<Derived>& in) {
  if (in.innerSize() != 3 || in.outerSize() != 1) {
    throw std::runtime_error("Input argument should be a 3-by-1 vector.");
  }

  Eigen::Matrix<SCALAR_T, 3, 3> out;
  out << SCALAR_T(0.0), -in(2), +in(1), +in(2), SCALAR_T(0.0), -in(0), -in(1), +in(0), SCALAR_T(0.0);
  return out;
}

/**
 * Computes the matrix which transforms derivatives of angular velocities in the body frame to euler angles derivatives
 * WARNING: matrix is singular when rotation around y axis is +/- 90 degrees
 * @param[in] eulerAngles: euler angles in xyz convention
 * @return M: matrix that does the transformation
 */
template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 3> angularVelocitiesToEulerAngleDerivativesMatrix(const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAngles) {
  Eigen::Matrix<SCALAR_T, 3, 3> M;
  SCALAR_T sinPsi = sin(eulerAngles(2));
  SCALAR_T cosPsi = cos(eulerAngles(2));
  SCALAR_T sinTheta = sin(eulerAngles(1));
  SCALAR_T cosTheta = cos(eulerAngles(1));

  M << cosPsi / cosTheta, -sinPsi / cosTheta, SCALAR_T(0), sinPsi, cosPsi, SCALAR_T(0), -cosPsi * sinTheta / cosTheta,
      sinTheta * sinPsi / cosTheta, SCALAR_T(1);

  return M;
}

template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 1> angularVelocitiesToEulerAngleDerivatives(const Eigen::Matrix<SCALAR_T, 3, 1>& angularVelocitiesInBase,
                                                                       const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAngles) {
  const SCALAR_T sinPsi = sin(eulerAngles(2));
  const SCALAR_T cosPsi = cos(eulerAngles(2));
  const SCALAR_T sinTheta = sin(eulerAngles(1));
  const SCALAR_T cosTheta = cos(eulerAngles(1));

  const SCALAR_T dRoll = (cosPsi * angularVelocitiesInBase(0) - sinPsi * angularVelocitiesInBase(1)) / cosTheta;
  const SCALAR_T dPitch = sinPsi * angularVelocitiesInBase(0) + cosPsi * angularVelocitiesInBase(1);
  const SCALAR_T dYaw = angularVelocitiesInBase(2) - sinTheta * dRoll;

  return {dRoll, dPitch, dYaw};
}

/**
 * Computes the derivative w.r.t eulerXYZ of rotating a vector from base to origin : d/d(euler) (o_R_b(euler) * v_base)
 *
 * @param[in] eulerAnglesXYZ: euler angles in xyz convention.
 * @param[in] vectorInBase: the vector in base that is rotated.
 * @return d/d(euler) (o_R_b(euler) * v_base)
 */
template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 3> rotationBaseToOriginJacobian(const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAnglesXYZ,
                                                           const Eigen::Matrix<SCALAR_T, 3, 1>& vectorInBase) {
  // dependent variables
  Eigen::Matrix<SCALAR_T, 3, 3> jac;

  // auxiliary variables
  std::array<SCALAR_T, 12> v{};

  jac(0) = SCALAR_T(0.0);
  v[0] = -sin(eulerAnglesXYZ[1]);
  v[1] = cos(eulerAnglesXYZ[2]);
  v[2] = sin(eulerAnglesXYZ[2]);
  v[3] = v[1] * vectorInBase[0] - v[2] * vectorInBase[1];
  v[4] = cos(eulerAnglesXYZ[1]);
  jac(3) = v[0] * v[3] + v[4] * vectorInBase[2];
  v[5] = v[4];
  v[6] = -v[2];
  v[7] = v[1];
  v[8] = v[6] * vectorInBase[0] - v[7] * vectorInBase[1];
  jac(6) = v[5] * v[8];
  v[9] = -sin(eulerAnglesXYZ[0]);
  v[2] = v[2] * vectorInBase[0] + v[1] * vectorInBase[1];
  v[1] = cos(eulerAnglesXYZ[0]);
  v[10] = v[0];
  v[5] = v[10] * v[3] + v[5] * vectorInBase[2];
  jac(1) = v[9] * v[2] - v[1] * v[5];
  v[11] = -v[9];
  v[4] = (-v[4]) * v[3] + v[0] * vectorInBase[2];
  jac(4) = -v[11] * v[4];
  v[3] = v[1];
  v[7] = v[7] * vectorInBase[0] + v[6] * vectorInBase[1];
  v[10] = v[10] * v[8];
  jac(7) = v[3] * v[7] - v[11] * v[10];
  jac(2) = v[1] * v[2] + v[9] * v[5];
  jac(5) = v[3] * v[4];
  jac(8) = v[11] * v[7] + v[3] * v[10];
  return jac;
}

template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 1> rotationErrorInWorldEulerXYZ(const Eigen::Matrix<SCALAR_T, 3, 1>& eulerXYZcurrent,
                                                           const Eigen::Matrix<SCALAR_T, 3, 1>& eulerXYZreference) {
  return ocs2::rotationErrorInWorld(rotationMatrixBaseToOrigin(eulerXYZcurrent), rotationMatrixBaseToOrigin(eulerXYZreference));
}

template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 1> rotationErrorInLocalEulerXYZ(const Eigen::Matrix<SCALAR_T, 3, 1>& eulerXYZcurrent,
                                                           const Eigen::Matrix<SCALAR_T, 3, 1>& eulerXYZreference) {
  return ocs2::rotationErrorInLocal(rotationMatrixBaseToOrigin(eulerXYZcurrent), rotationMatrixBaseToOrigin(eulerXYZreference));
}

}  // namespace switched_model
