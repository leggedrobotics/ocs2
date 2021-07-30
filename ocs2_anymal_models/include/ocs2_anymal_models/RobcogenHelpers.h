//
// Created by rgrandia on 23.09.20.
//

#pragma once

#include <ocs2_switched_model_interface/core/Rotations.h>
#include <ocs2_switched_model_interface/core/SwitchedModel.h>
#include <ocs2_switched_model_interface/core/WholebodyDynamics.h>

namespace anymal {
namespace robcogen_helpers {

template <typename SCALAR_T, int NUM_JOINTS_IMPL>
struct BaseDynamicsTerms {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Matrix<SCALAR_T, switched_model::BASE_COORDINATE_SIZE, switched_model::BASE_COORDINATE_SIZE> Mb;
  Eigen::Matrix<SCALAR_T, switched_model::BASE_COORDINATE_SIZE, NUM_JOINTS_IMPL> Mj;
  switched_model::base_coordinate_s_t<SCALAR_T> C;
  switched_model::base_coordinate_s_t<SCALAR_T> G;
};

template <typename INVDYN, typename JSIM, typename SCALAR_T, int NUM_JOINTS_IMPL>
BaseDynamicsTerms<SCALAR_T, NUM_JOINTS_IMPL> getBaseDynamicsTermsImpl(INVDYN& inverseDynamics, JSIM& jsim,
                                                                      const switched_model::base_coordinate_s_t<SCALAR_T>& qBase,
                                                                      const switched_model::base_coordinate_s_t<SCALAR_T>& qdBase,
                                                                      const Eigen::Matrix<SCALAR_T, NUM_JOINTS_IMPL, 1>& qJoints,
                                                                      const Eigen::Matrix<SCALAR_T, NUM_JOINTS_IMPL, 1>& qdJoints) {
  BaseDynamicsTerms<SCALAR_T, NUM_JOINTS_IMPL> baseDynamicsTerms;

  const auto M = jsim.update(qJoints);
  baseDynamicsTerms.Mb = M.template topLeftCorner<switched_model::BASE_COORDINATE_SIZE, switched_model::BASE_COORDINATE_SIZE>();
  baseDynamicsTerms.Mj = M.template topRightCorner<switched_model::BASE_COORDINATE_SIZE, NUM_JOINTS_IMPL>();

  inverseDynamics.setJointStatus(qJoints);

  {  // Get gravitational vector
    // gravity vector in the base frame
    switched_model::vector6_s_t<SCALAR_T> gravity;
    const SCALAR_T gravitationalAcceleration = SCALAR_T(9.81);
    gravity << switched_model::vector3_s_t<SCALAR_T>::Zero(),
        switched_model::rotateVectorOriginToBase(
            switched_model::vector3_s_t<SCALAR_T>(SCALAR_T(0.0), SCALAR_T(0.0), -gravitationalAcceleration),
            switched_model::getOrientation(qBase));

    switched_model::vector6_s_t<SCALAR_T> baseWrench;
    Eigen::Matrix<SCALAR_T, NUM_JOINTS_IMPL, 1> jForces;
    inverseDynamics.G_terms_fully_actuated(baseDynamicsTerms.G, jForces, gravity);
  }

  {  // Get coriolis/centrifugal vector
    switched_model::vector6_s_t<SCALAR_T> baseWrench;
    Eigen::Matrix<SCALAR_T, NUM_JOINTS_IMPL, 1> jForces;
    inverseDynamics.C_terms_fully_actuated(baseDynamicsTerms.C, jForces, qdBase, qdJoints);
  }

  return baseDynamicsTerms;
}

template <typename INVDYN, typename JSIM, typename SCALAR_T>
typename switched_model::WholebodyDynamics<SCALAR_T>::DynamicsTerms getDynamicsTermsImpl(
    INVDYN& inverseDynamics, JSIM& jsim, const switched_model::rbd_state_s_t<SCALAR_T>& rbdState) {
  const switched_model::base_coordinate_s_t<SCALAR_T> qBase = switched_model::getBasePose(rbdState);
  const switched_model::joint_coordinate_s_t<SCALAR_T> qJoints = switched_model::getJointPositions(rbdState);
  const switched_model::base_coordinate_s_t<SCALAR_T> qdBase = switched_model::getBaseLocalVelocity(rbdState);
  const switched_model::joint_coordinate_s_t<SCALAR_T> qdJoints = switched_model::getJointVelocities(rbdState);

  typename switched_model::WholebodyDynamics<SCALAR_T>::DynamicsTerms dynamicsTerms;
  dynamicsTerms.M = jsim.update(qJoints);

  inverseDynamics.setJointStatus(qJoints);

  {  // Get gravitational vector
    // gravity vector in the base frame
    switched_model::vector6_s_t<SCALAR_T> gravity;
    //! @todo(jcarius) Gravity hardcoded
    const SCALAR_T gravitationalAcceleration = SCALAR_T(9.81);
    gravity << switched_model::vector3_s_t<SCALAR_T>::Zero(),
        switched_model::rotateVectorOriginToBase(
            switched_model::vector3_s_t<SCALAR_T>(SCALAR_T(0.0), SCALAR_T(0.0), -gravitationalAcceleration),
            switched_model::getOrientation(qBase));

    switched_model::vector6_s_t<SCALAR_T> baseWrench;
    switched_model::joint_coordinate_s_t<SCALAR_T> jForces;
    inverseDynamics.G_terms_fully_actuated(baseWrench, jForces, gravity);
    dynamicsTerms.G << baseWrench, jForces;
  }

  {  // Get coriolis/centrifugal vector
    switched_model::vector6_s_t<SCALAR_T> baseWrench;
    switched_model::joint_coordinate_s_t<SCALAR_T> jForces;
    inverseDynamics.C_terms_fully_actuated(baseWrench, jForces, qdBase, qdJoints);
    dynamicsTerms.C << baseWrench, jForces;
  }

  return dynamicsTerms;
}

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
 * Solve c = inv(M) * b, where M is an inertia tensor with first angular, then linear components.
 *
 * i.e. M = [Theta, crossTerm;
 *           crossTerm', m * I_3x3 ]
 *
 * The implementation is compatible with auto-differentiation.
 */
template <typename SCALAR_T>
switched_model::vector6_s_t<SCALAR_T> inertiaTensorSolve(const switched_model::matrix6_s_t<SCALAR_T>& M,
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

template <typename INVDYN, typename JSIM, typename SCALAR_T, int NUM_JOINTS_IMPL>
switched_model::base_coordinate_s_t<SCALAR_T> computeBaseAcceleration(
    INVDYN& inverseDynamics, JSIM& jsim, const switched_model::base_coordinate_s_t<SCALAR_T>& basePose,
    const switched_model::base_coordinate_s_t<SCALAR_T>& baseLocalVelocities,
    const Eigen::Matrix<SCALAR_T, NUM_JOINTS_IMPL, 1>& jointPositions, const Eigen::Matrix<SCALAR_T, NUM_JOINTS_IMPL, 1>& jointVelocities,
    const Eigen::Matrix<SCALAR_T, NUM_JOINTS_IMPL, 1>& jointAccelerations,
    const switched_model::base_coordinate_s_t<SCALAR_T>& forcesOnBaseInBaseFrame) {
  const auto baseDynamics = getBaseDynamicsTermsImpl(inverseDynamics, jsim, basePose, baseLocalVelocities, jointPositions, jointVelocities);

  switched_model::base_coordinate_s_t<SCALAR_T> baseForces = forcesOnBaseInBaseFrame - baseDynamics.G - baseDynamics.C;
  baseForces.noalias() -= baseDynamics.Mj * jointAccelerations;

  return inertiaTensorSolve<SCALAR_T>(baseDynamics.Mb, baseForces);
}

}  // namespace robcogen_helpers
}  // namespace anymal
