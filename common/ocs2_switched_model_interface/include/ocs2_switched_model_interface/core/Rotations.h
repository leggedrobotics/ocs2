//
// Created by rgrandia on 22.10.19.
//

#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>

namespace switched_model {

/**
 * Base to origin rotation matrix
 * @tparam Derived
 * @param [in] eulerAngles
 * @return
 */
template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 3> RotationMatrixBasetoOrigin(const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAngles) {
  if (eulerAngles.innerSize() != 3 || eulerAngles.outerSize() != 1) throw std::runtime_error("Input argument should be a 3-by-1 vector.");

  // inputs are the intrinsic rotation angles in RADIANTS
  SCALAR_T sinAlpha = sin(eulerAngles(0));
  SCALAR_T cosAlpha = cos(eulerAngles(0));
  SCALAR_T sinBeta = sin(eulerAngles(1));
  SCALAR_T cosBeta = cos(eulerAngles(1));
  SCALAR_T sinGamma = sin(eulerAngles(2));
  SCALAR_T cosGamma = cos(eulerAngles(2));

  Eigen::Matrix<SCALAR_T, 3, 3> Rx, Ry, Rz;
  Rx << SCALAR_T(1), SCALAR_T(0), SCALAR_T(0), SCALAR_T(0), cosAlpha, -sinAlpha, SCALAR_T(0), sinAlpha, cosAlpha;
  Ry << cosBeta, SCALAR_T(0), sinBeta, SCALAR_T(0), SCALAR_T(1), SCALAR_T(0), -sinBeta, SCALAR_T(0), cosBeta;
  Rz << cosGamma, -sinGamma, SCALAR_T(0), sinGamma, cosGamma, SCALAR_T(0), SCALAR_T(0), SCALAR_T(0), SCALAR_T(1);

  return Rx * Ry * Rz;
}

template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 3> RotationMatrixOrigintoBase(const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAngles) {
  return RotationMatrixBasetoOrigin<SCALAR_T>(eulerAngles).transpose();
}

template <typename SCALAR_T>
Eigen::Quaternion<SCALAR_T> QuaternionBaseToOrigin(const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAngles) {
  const auto roll = eulerAngles(0);
  const auto pitch = eulerAngles(1);
  const auto yaw = eulerAngles(2);
  return Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
}

template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 1> EulerAnglesFromQuaternionBaseToOrigin(const Eigen::Quaternion<SCALAR_T>& q_origin_base) {
  return q_origin_base.toRotationMatrix().eulerAngles(0, 1, 2);
}

/**
 * Calculates the skew matrix for vector cross product
 * @tparam Derived
 * @param [in] in
 * @return
 */
template <typename SCALAR_T, typename Derived>
Eigen::Matrix<SCALAR_T, 3, 3> CrossProductMatrix(const Eigen::DenseBase<Derived>& in) {
  if (in.innerSize() != 3 || in.outerSize() != 1) throw std::runtime_error("Input argument should be a 3-by-1 vector.");

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
Eigen::Matrix<SCALAR_T, 3, 3> AngularVelocitiesToEulerAngleDerivativesMatrix(const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAngles) {
  Eigen::Matrix<SCALAR_T, 3, 3> M;
  SCALAR_T sinPsi = sin(eulerAngles(2));
  SCALAR_T cosPsi = cos(eulerAngles(2));
  SCALAR_T sinTheta = sin(eulerAngles(1));
  SCALAR_T cosTheta = cos(eulerAngles(1));

  M << cosPsi / cosTheta, -sinPsi / cosTheta, SCALAR_T(0), sinPsi, cosPsi, SCALAR_T(0), -cosPsi * sinTheta / cosTheta,
      sinTheta * sinPsi / cosTheta, SCALAR_T(1);

  return M;
}

}  // namespace switched_model
