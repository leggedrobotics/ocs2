#pragma once

#include <ocs2_switched_model_interface/core/SwitchedModel.h>

namespace anymal {
/*
 * Uses only upper triangular part of A
 */
template <typename SCALAR_T>
switched_model::matrix3_s_t<SCALAR_T> inv33sym(const switched_model::matrix3_s_t<SCALAR_T>& A, SCALAR_T DinvMax = SCALAR_T(1e12)) {
  // Analytical inverse of symmetrical 3x3 matrix
  switched_model::matrix3_s_t<SCALAR_T> Ainv;
  Ainv(0, 0) = A(2, 2) * A(1, 1) - A(1, 2) * A(1, 2);
  Ainv(0, 1) = A(0, 2) * A(1, 2) - A(2, 2) * A(0, 1);
  Ainv(0, 2) = A(0, 1) * A(1, 2) - A(0, 2) * A(1, 1);

  Ainv(1, 1) = A(2, 2) * A(0, 0) - A(0, 2) * A(0, 2);
  Ainv(1, 2) = A(0, 1) * A(0, 2) - A(0, 0) * A(1, 2);

  Ainv(2, 2) = A(0, 0) * A(1, 1) - A(0, 1) * A(0, 1);

  // Determinant
  SCALAR_T D = A(0, 0) * Ainv(0, 0) + A(0, 1) * Ainv(0, 1) + A(0, 2) * Ainv(0, 2);
  SCALAR_T Dinv = CppAD::CondExpGt(CppAD::abs(D), 1.0 / DinvMax, SCALAR_T(1.0) / D, DinvMax);

  // Scaling and copying to symmetric part.
  Ainv(0, 0) = Ainv(0, 0) * Dinv;
  Ainv(0, 1) = Ainv(0, 1) * Dinv;
  Ainv(0, 2) = Ainv(0, 2) * Dinv;

  Ainv(1, 0) = Ainv(0, 1);
  Ainv(1, 1) = Ainv(1, 1) * Dinv;
  Ainv(1, 2) = Ainv(1, 2) * Dinv;

  Ainv(2, 0) = Ainv(0, 2);
  Ainv(2, 1) = Ainv(1, 2);
  Ainv(2, 2) = Ainv(2, 2) * Dinv;
  return Ainv;
}

/*
 * Uses only upper triangular part of A
 */
template <typename SCALAR_T>
switched_model::vector3_s_t<SCALAR_T> solve33sym(const switched_model::matrix3_s_t<SCALAR_T>& A,
                                                 const switched_model::vector3_s_t<SCALAR_T>& b, SCALAR_T DinvMax = SCALAR_T(1e12)) {
  // Analytical inverse of symmetrical 3x3 matrix, stored in a flat array
  switched_model::vector6_s_t<SCALAR_T> Ainv;
  Ainv(0) = A(2, 2) * A(1, 1) - A(1, 2) * A(1, 2);
  Ainv(1) = A(0, 2) * A(1, 2) - A(2, 2) * A(0, 1);
  Ainv(2) = A(0, 1) * A(1, 2) - A(0, 2) * A(1, 1);
  Ainv(3) = A(2, 2) * A(0, 0) - A(0, 2) * A(0, 2);
  Ainv(4) = A(0, 1) * A(0, 2) - A(0, 0) * A(1, 2);
  Ainv(5) = A(0, 0) * A(1, 1) - A(0, 1) * A(0, 1);

  // Determinant
  SCALAR_T D = A(0, 0) * Ainv(0) + A(0, 1) * Ainv(1) + A(0, 2) * Ainv(2);
  SCALAR_T Dinv = CppAD::CondExpGt(CppAD::abs(D), 1.0 / DinvMax, SCALAR_T(1.0) / D, DinvMax);

  switched_model::vector3_s_t<SCALAR_T> c;
  c(0) = Dinv * (Ainv(0) * b(0) + Ainv(1) * b(1) + Ainv(2) * b(2));
  c(1) = Dinv * (Ainv(1) * b(0) + Ainv(3) * b(1) + Ainv(4) * b(2));
  c(2) = Dinv * (Ainv(2) * b(0) + Ainv(4) * b(1) + Ainv(5) * b(2));
  return c;
}

/**
 * Solve c = inv(M) * b, where M is an inertia tensor with first linear and then angular components.
 *
 * i.e. M = [m * I_3x3, crossTerm';
 *           crossTerm, Theta ]
 *
 * The implementation is compatible with auto-differentiation.
 */
template <typename SCALAR_T>
switched_model::vector6_s_t<SCALAR_T> inertiaTensorSolveLinearAngular(const switched_model::matrix6_s_t<SCALAR_T>& M,
                                                                      const switched_model::vector6_s_t<SCALAR_T>& b,
                                                                      SCALAR_T DinvMax = SCALAR_T(1e12)) {
  /*
   * Uses the schur-complement method plus the structure of the inertia tensor to efficiently construct the result of inv(M) * b
   *
   * M has the following structure:
   * topLeft(3x3) = m * I_3x3
   * topRight(3x3) = m * [comVector]_x'
   * bottomRight(3x3) = Theta = Theta_com + m * [comVector]_x * [comVector]_x'
   */

  // total mass
  const SCALAR_T invm = SCALAR_T(1.0) / M(2, 2);

  // extract underlying vector for the crossterms s.t. M.upperRight = crossTerm' = [m * comVector]_x'
  const switched_model::vector3_s_t<SCALAR_T> scaledComVector{M(1, 5), M(2, 3), M(0, 4)};  // is actually m * comVector
  const switched_model::vector3_s_t<SCALAR_T> comVector = invm * scaledComVector;          // store a version that is pre-multiplied by 1/m

  // prepare rhs for solve33sym, tmp = b.tail(3) -  comVector x b.head(3)
  switched_model::vector3_s_t<SCALAR_T> tmp = b.template tail<3>() - comVector.template cross(b.template head<3>());

  // prepare lhs for solve33sym, fill only upper triangular part
  switched_model::matrix3_s_t<SCALAR_T> centroidalInertia;  // = theta_com = theta - 1/m * crossTerm * crossTerm'
  const SCALAR_T v0v0 = comVector(0) * scaledComVector(0);
  const SCALAR_T v1v1 = comVector(1) * scaledComVector(1);
  const SCALAR_T v2v2 = comVector(2) * scaledComVector(2);
  centroidalInertia(0, 0) = M(3, 3) - v1v1 - v2v2;
  centroidalInertia(0, 1) = M(3, 4) + comVector(0) * scaledComVector(1);
  centroidalInertia(0, 2) = M(3, 5) + comVector(0) * scaledComVector(2);
  centroidalInertia(1, 1) = M(4, 4) - v0v0 - v2v2;
  centroidalInertia(1, 2) = M(4, 5) + comVector(1) * scaledComVector(2);
  centroidalInertia(2, 2) = M(5, 5) - v0v0 - v1v1;

  // Solve the linear system, only need to invert the 3x3 schur complement (= centroidalInertia)
  switched_model::vector6_s_t<SCALAR_T> c;
  // angular acceleration
  c.template tail<3>() = solve33sym(centroidalInertia, tmp, DinvMax);
  // Linear acceleration
  c.template head<3>() = invm * b.template head<3>() + comVector.template cross(c.template tail<3>());

  return c;
}

/**
 * Solve c = inv(M) * b, where M is an inertia tensor with first angular, then linear components.
 *
 * i.e. M = [Theta, crossTerm;
 *           crossTerm', m * I_3x3 ]
 *
 * The implementation is compatible with auto-differentiation.
 */
template <typename SCALAR_T>
switched_model::vector6_s_t<SCALAR_T> inertiaTensorSolveAngularLinear(const switched_model::matrix6_s_t<SCALAR_T>& M,
                                                                      const switched_model::vector6_s_t<SCALAR_T>& b,
                                                                      SCALAR_T DinvMax = SCALAR_T(1e12)) {
  /*
   * Uses the schur complement method plus the structure of the interia tensor to efficiently construct the result of inv(M) * b
   *
   * M has the following structure:
   * topLeft(3x3) = Theta = Theta_com + m * [comVector]_x * [comVector]_x'
   * topRight(3x3) = m * [comVector]_x
   * bottomRight(3x3) = m * I_3x3
   */

  // total mass, bottom right block is m * I_3x3;
  const SCALAR_T invm = SCALAR_T(1.0) / M(5, 5);

  // extract underlying vector for the crossterms s.t. M.upperRight = crossTerm = [m * comVector]_x
  const switched_model::vector3_s_t<SCALAR_T> scaledComVector{M(2, 4), M(0, 5), M(1, 3)};  // is actually m * comVector
  const switched_model::vector3_s_t<SCALAR_T> comVector = invm * scaledComVector;          // store a version that is pre-multiplied by 1/m

  // prepare rhs for solve33sym, tmp = b.head(3) * crossTerm' * b.tail(3)
  switched_model::vector3_s_t<SCALAR_T> tmp = b.template head<3>() - comVector.template cross(b.template tail<3>());

  // prepare lhs for solve33sym, fill only upper triangular part
  switched_model::matrix3_s_t<SCALAR_T> centroidalInertia;  // = theta_com = theta - 1/m * crossTerm * crossTerm'
  const SCALAR_T v0v0 = comVector(0) * scaledComVector(0);
  const SCALAR_T v1v1 = comVector(1) * scaledComVector(1);
  const SCALAR_T v2v2 = comVector(2) * scaledComVector(2);
  centroidalInertia(0, 0) = M(0, 0) - v1v1 - v2v2;
  centroidalInertia(0, 1) = M(0, 1) + comVector(0) * scaledComVector(1);
  centroidalInertia(0, 2) = M(0, 2) + comVector(0) * scaledComVector(2);
  centroidalInertia(1, 1) = M(1, 1) - v0v0 - v2v2;
  centroidalInertia(1, 2) = M(1, 2) + comVector(1) * scaledComVector(2);
  centroidalInertia(2, 2) = M(2, 2) - v0v0 - v1v1;

  // Solve the linear system, only need to invert the 3x3 schur complement (= centroidalInertia)
  switched_model::vector6_s_t<SCALAR_T> c;
  c.template head<3>() = solve33sym(centroidalInertia, tmp, DinvMax);
  c.template tail<3>() = invm * b.template tail<3>() + comVector.template cross(c.template head<3>());

  return c;
}

}  // namespace anymal
