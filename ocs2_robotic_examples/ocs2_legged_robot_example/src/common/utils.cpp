//
// Created by jpsleiman on 27.11.19.
//

#include <Eigen/Dense>
#include <array>
#include <cppad/cg.hpp>
#include <iostream>
#include <memory>

namespace ocs2 {
namespace alma_c {
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
  skewSymmetricMatrix << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
}

template <typename SCALAR_T>
void getQuaternionFromEulerAnglesZyx(const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAngles, Eigen::Quaternion<SCALAR_T>& quaternion) {
  SCALAR_T yaw = eulerAngles(0);
  SCALAR_T pitch = eulerAngles(1);
  SCALAR_T roll = eulerAngles(2);

  SCALAR_T cy = cos(yaw * 0.5);
  SCALAR_T sy = sin(yaw * 0.5);
  SCALAR_T cp = cos(pitch * 0.5);
  SCALAR_T sp = sin(pitch * 0.5);
  SCALAR_T cr = cos(roll * 0.5);
  SCALAR_T sr = sin(roll * 0.5);

  quaternion.w() = cy * cp * cr + sy * sp * sr;
  quaternion.x() = cy * cp * sr - sy * sp * cr;
  quaternion.y() = sy * cp * sr + cy * sp * cr;
  quaternion.z() = sy * cp * cr - cy * sp * sr;
}

template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 1> quaternionDistance(const Eigen::Quaternion<SCALAR_T>& eeQuaternion,
                                                 const Eigen::Quaternion<SCALAR_T>& commandQuaternion) {
  return (eeQuaternion.w() * commandQuaternion.vec() - commandQuaternion.w() * eeQuaternion.vec() -
          commandQuaternion.vec().cross(eeQuaternion.vec()));
}

template <typename SCALAR_T>
void matrixToQuaternion(Eigen::Quaternion<SCALAR_T>& quaternion, const Eigen::Matrix<SCALAR_T, 3, 3>& R) {
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

  quaternion.x() = q(0);
  quaternion.y() = q(1);
  quaternion.z() = q(2);
  quaternion.w() = q(3);
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
void getFloatingBaseCentroidalMomentumMatrixInverse(const Eigen::Matrix<SCALAR_T, 6, 6>& Ab, Eigen::Matrix<SCALAR_T, 6, 6>& Ab_inv) {
  Eigen::Matrix<SCALAR_T, 3, 3> Ab_22_inv = Ab.template block<3, 3>(3, 3).inverse();
  SCALAR_T robotMass = Ab(0, 0);
  Ab_inv << 1.0 / robotMass * Eigen::Matrix<SCALAR_T, 3, 3>::Identity(), -1.0 / robotMass * Ab.template block<3, 3>(0, 3) * Ab_22_inv,
      Eigen::Matrix<SCALAR_T, 3, 3>::Zero(), Ab_22_inv;
}

template <typename SCALAR_T>
void getRotationMatrixFromZyxEulerAngles(const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAngles, Eigen::Matrix<SCALAR_T, 3, 3>& rotationMatrix) {
  SCALAR_T z = eulerAngles(0);
  SCALAR_T y = eulerAngles(1);
  SCALAR_T x = eulerAngles(2);

  SCALAR_T c1 = cos(z);
  SCALAR_T c2 = cos(y);
  SCALAR_T c3 = cos(x);
  SCALAR_T s1 = sin(z);
  SCALAR_T s2 = sin(y);
  SCALAR_T s3 = sin(x);

  rotationMatrix << c1 * c2, c1 * s2 * s3 - s1 * c3, c1 * s2 * c3 + s1 * s3, s1 * c2, s1 * s2 * s3 + c1 * c3, s1 * s2 * c3 - c1 * s3, -s2,
      c2 * s3, c2 * c3;
}

template <typename SCALAR_T>
void getAngularVelocityInWorldFrameFromEulerAnglesZyx(const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAngles,
                                                      const Eigen::Matrix<SCALAR_T, 3, 1>& derivativesEulerAngles,
                                                      Eigen::Matrix<SCALAR_T, 3, 1>& angularVelocity) {
  SCALAR_T z = eulerAngles(0);
  SCALAR_T y = eulerAngles(1);
  SCALAR_T x = eulerAngles(2);
  Eigen::Matrix<SCALAR_T, 3, 3> transformationMatrix;
  transformationMatrix << 0, -sin(z), cos(y) * cos(z), 0, cos(z), cos(y) * sin(z), 1, 0, -sin(y);

  angularVelocity = transformationMatrix * derivativesEulerAngles;
}

template <typename SCALAR_T>
void getAngularAccelerationInWorldFrameFromEulerAnglesZyx(const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAngles,
                                                          const Eigen::Matrix<SCALAR_T, 3, 1>& derivativesEulerAngles,
                                                          const Eigen::Matrix<SCALAR_T, 3, 1>& secondDerivativesEulerAngles,
                                                          Eigen::Matrix<SCALAR_T, 3, 1>& angularAcceleration) {
  SCALAR_T z = eulerAngles(0);
  SCALAR_T y = eulerAngles(1);
  SCALAR_T x = eulerAngles(2);
  SCALAR_T dz = derivativesEulerAngles(0);
  SCALAR_T dy = derivativesEulerAngles(1);
  SCALAR_T dx = derivativesEulerAngles(2);

  Eigen::Matrix<SCALAR_T, 3, 3> transformationMatrix;
  transformationMatrix << 0, -sin(z), cos(y) * cos(z), 0, cos(z), cos(y) * sin(z), 1, 0, -sin(y);

  Eigen::Matrix<SCALAR_T, 3, 3> derivativeTransformationMatrix;
  derivativeTransformationMatrix << 0, -cos(z) * dz, (-sin(y) * dy) * cos(z) + cos(y) * (-sin(z) * dz), 0, -sin(z) * dz,
      (-sin(y) * dy) * sin(z) + cos(y) * (cos(z) * dz), 0, 0, -cos(y) * dy;

  angularAcceleration = derivativeTransformationMatrix * derivativesEulerAngles + transformationMatrix * secondDerivativesEulerAngles;
}

template <typename SCALAR_T>
void getEulerAnglesZyxDerivativesFromLocalAngularVelocities(const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAngles,
                                                            const Eigen::Matrix<SCALAR_T, 3, 1>& angularVelocity,
                                                            Eigen::Matrix<SCALAR_T, 3, 1>& derivativesEulerAngles) {
  SCALAR_T z = eulerAngles(0);
  SCALAR_T y = eulerAngles(1);
  SCALAR_T x = eulerAngles(2);
  Eigen::Matrix<SCALAR_T, 3, 3> transformationMatrix;
  // Note: This matrix is ill-defined only for y = pi/2, but for anymal's base this is the case when it is perpendicular to the ground,
  // which never happens
  transformationMatrix << 0, sin(x) / cos(y), cos(x) / cos(y), 0, cos(x), -sin(x), 1, sin(x) * sin(y) / cos(y), cos(x) * sin(y) / cos(y);

  derivativesEulerAngles = transformationMatrix * angularVelocity;
}
}  // namespace alma_c
}  // namespace ocs2