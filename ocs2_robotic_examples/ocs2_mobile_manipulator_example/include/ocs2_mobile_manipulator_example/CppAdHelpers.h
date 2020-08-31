#pragma once

#include <cppad/cg/math.hpp>

#include "definitions.h"

namespace mobile_manipulator {

/**
 * CppAD compatible transform of rotation matrix to quaternion
 * Author: Johannes Pankert
 */
inline Eigen::Quaternion<ad_scalar_t> matrixToQuaternion(const Eigen::Matrix<ad_scalar_t, 3, 3>& R) {
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

inline Eigen::Quaternion<scalar_t> matrixToQuaternion(const Eigen::Matrix<scalar_t, 3, 3>& R) {
  return Eigen::Quaternion<scalar_t>(R);
}

template <typename SCALAR>
inline Eigen::Matrix<SCALAR, 3, 3> skewSymmetricMatrix(Eigen::Matrix<SCALAR, 3, 1> v) {
  Eigen::Matrix<SCALAR, 3, 3> skewSymmetricMatrix;
  // clang-format off
  skewSymmetricMatrix <<    0, -v(2),  v(1),
                         v(2),     0,  -v(0),
                        -v(1),  v(0),     0;
  // clan-format on
  return skewSymmetricMatrix;
}

}  // namespace mobile_manipulator
