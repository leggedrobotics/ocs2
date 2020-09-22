#pragma once

#include <ocs2_anymal_raisim/AnymalRaisimConversions.h>

#include <ocs2_switched_model_interface/core/Rotations.h>

namespace anymal {

Eigen::VectorXd AnymalRaisimConversions::inputToRaisimGeneralizedForce(double, const input_vector_t& input, const state_vector_t&,
                                                                       const Eigen::VectorXd& q, const Eigen::VectorXd& dq) const {
  const auto ocs2RbdState = raisimGenCoordGenVelToRbdState(q, dq);

  const switched_model::base_coordinate_t qBase = switched_model::getBasePose(ocs2RbdState);
  const switched_model::joint_coordinate_t qJoints = switched_model::getJointPositions(ocs2RbdState);
  const switched_model::base_coordinate_t qdBase = switched_model::getBaseLocalVelocity(ocs2RbdState);
  const switched_model::joint_coordinate_t qdJoints = switched_model::getJointVelocities(ocs2RbdState);

  // force transformed in the lowerLeg coordinate
  switched_model::base_coordinate_t extForceBase = switched_model::base_coordinate_t::Zero();
  for (int j = 0; j < 4; j++) {
    const auto b_footPosition = kinematicModelPtr_->positionBaseToFootInBaseFrame(j, qJoints);
    extForceBase.head<3>() += b_footPosition.cross(input.segment<3>(3 * j));
    extForceBase.tail<3>() += input.segment<3>(3 * j);
  }
  switched_model::joint_coordinate_t extForceJoint = switched_model::joint_coordinate_t::Zero();
  for (int j = 0; j < 4; j++) {
    const auto b_footJacobian = kinematicModelPtr_->baseToFootJacobianInBaseFrame(j, qJoints);
    extForceJoint += b_footJacobian.bottomRows<3>().transpose() * input.segment<3>(3 * j);
  }

  InertiaProperties_t inertias;
  ForceTransforms_t forceTransforms;
  JSIM_t Mm(inertias, forceTransforms);
  Mm.update(qJoints);

  MotionTransforms_t MotionTransforms;
  InverseDynamics_t inverseDynamics(inertias, MotionTransforms);
  inverseDynamics.setJointStatus(qJoints);

  switched_model::generalized_coordinate_t Gv;
  {
    // gravity vetor in the base frame
    iit::rbd::Vector6D gravity;
    const Eigen::Matrix3d b_R_o = switched_model::rotationMatrixOriginToBase<double>(qBase.head<3>());
    //! @todo(jcarius) Gravity hardcoded
    const double gravitationalAcceleration = 9.81;
    gravity << Eigen::Vector3d::Zero(), b_R_o * Eigen::Vector3d(0.0, 0.0, -gravitationalAcceleration);

    iit::rbd::Vector6D baseWrench;
    switched_model::joint_coordinate_t jForces;
    inverseDynamics.G_terms_fully_actuated(baseWrench, jForces, gravity);
    Gv << baseWrench, jForces;
  }

  switched_model::generalized_coordinate_t Cv;
  {
    iit::rbd::Vector6D baseWrench;
    switched_model::joint_coordinate_t jForces;
    inverseDynamics.C_terms_fully_actuated(baseWrench, jForces, qdBase, qdJoints);
    Cv << baseWrench, jForces;
  }

  // compute Base local acceleration about Base frame
  switched_model::joint_coordinate_t qddJoints = switched_model::joint_coordinate_t::Zero();  // TODO(jcarius) recover this?!
  switched_model::base_coordinate_t comLocalAcceleration =
      -Mm.template topRightCorner<6, 12>() * qddJoints - Cv.template head<6>() - Gv.template head<6>() + extForceBase;
  comLocalAcceleration = Mm.template topLeftCorner<6, 6>().ldlt().solve(comLocalAcceleration);

  // compute joint torque
  switched_model::joint_coordinate_t ocs2rbdInput = Mm.template bottomLeftCorner<12, 6>() * comLocalAcceleration +
                                                    Mm.template bottomRightCorner<12, 12>() * qddJoints + Cv.template tail<12>() +
                                                    Gv.template tail<12>() - extForceJoint;

  if (ocs2rbdInput.array().abs().maxCoeff() > 100) {
    std::stringstream ss;
    ss << "AnymalRaisimConversions::inputToRaisimGeneralizedForce -- Raisim input unstable:"
       << "\nocs2rbdInput = " << ocs2rbdInput.transpose() << "\nq = " << q.transpose();
    throw std::runtime_error(ss.str());
  }

  // convert to raisim input
  Eigen::Matrix<double, 18, 1> raisimInput;
  raisimInput.setZero();
  raisimInput.tail<12>() = ocs2rbdInput;

  return raisimInput;
}

}  // namespace anymal
