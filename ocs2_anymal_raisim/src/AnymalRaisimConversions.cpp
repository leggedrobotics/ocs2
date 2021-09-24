#include <ocs2_anymal_raisim/AnymalRaisimConversions.h>

#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <ocs2_switched_model_interface/core/Rotations.h>
#include <ocs2_switched_model_interface/core/SwitchedModelStateEstimator.h>

namespace anymal {

std::pair<Eigen::VectorXd, Eigen::VectorXd> AnymalRaisimConversions::stateToRaisimGenCoordGenVel(const state_vector_t& state,
                                                                                                 const input_vector_t& input) const {
  const auto ocs2RbdState = switched_model::estimateRbdModelState(state, input.segment<switched_model::JOINT_COORDINATE_SIZE>(12));
  const auto basePose = switched_model::getBasePose(ocs2RbdState);
  const auto baseTwist = switched_model::getBaseLocalVelocity(ocs2RbdState);

  // quaternion between world and base orientation
  const Eigen::Quaterniond q_world_base = switched_model::quaternionBaseToOrigin<double>(ocs2RbdState.head<3>());

  Eigen::VectorXd q(3 + 4 + 12);
  q << switched_model::getPositionInOrigin(basePose), q_world_base.w(), q_world_base.x(), q_world_base.y(), q_world_base.z(),
      switched_model::getJointPositions(state);
  if (terrain_ != nullptr) {
    q(2) += terrain_->getHeight(q(0), q(1));
  }

  Eigen::VectorXd dq(3 + 3 + 12);
  dq << q_world_base * switched_model::getLinearVelocity(baseTwist), q_world_base * switched_model::getAngularVelocity(baseTwist),
      switched_model::getJointVelocities(input);

  return {q, dq};
}

switched_model::rbd_state_t AnymalRaisimConversions::raisimGenCoordGenVelToRbdState(const Eigen::VectorXd& q,
                                                                                    const Eigen::VectorXd& dq) const {
  if (q.tail<12>().array().abs().maxCoeff() > 2.0 * M_PI   // joint position
      or dq.segment<6>(0).array().abs().maxCoeff() > 10.0  // linear/angular base velocity
      or dq.tail<12>().array().abs().maxCoeff() > 30.0     // joint velocity
  ) {
    std::stringstream ss;
    ss << "AnymalRaisimConversions::raisimGenCoordGenVelToRbdState -- Raisim state unstable:"
       << "\nq = " << q.transpose() << "\ndq = " << dq.transpose();
    throw std::runtime_error(ss.str());
  }

  Eigen::Quaterniond q_world_base(q(3), q(4), q(5), q(6));  // quaternion coefficients w, x, y z
  Eigen::Vector3d eulerAngles = switched_model::eulerAnglesFromQuaternionBaseToOrigin<double>(q_world_base);
  ocs2::makeEulerAnglesUnique(eulerAngles);

  switched_model::rbd_state_t ocs2RbdState;
  ocs2RbdState << eulerAngles, q.head<3>(), q.tail<12>(), q_world_base.inverse() * dq.segment<3>(3),
      q_world_base.inverse() * dq.segment<3>(0), dq.tail<12>();

  if (terrain_ != nullptr) {
    ocs2RbdState(5) -= terrain_->getHeight(q(0), q(1));
  }

  return ocs2RbdState;
}

AnymalRaisimConversions::state_vector_t AnymalRaisimConversions::raisimGenCoordGenVelToState(const Eigen::VectorXd& q,
                                                                                             const Eigen::VectorXd& dq) const {
  return switched_model::estimateComkinoModelState(raisimGenCoordGenVelToRbdState(q, dq));
}

std::pair<Eigen::VectorXd, Eigen::VectorXd> AnymalRaisimConversions::inputToRaisimPdTargets(double, const input_vector_t& input,
                                                                                            const state_vector_t&, const Eigen::VectorXd& q,
                                                                                            const Eigen::VectorXd& dq) {
  Eigen::VectorXd positionSetpoint = q;

  Eigen::VectorXd velocitySetpoint(dq.size());
  velocitySetpoint.setZero();
  velocitySetpoint.tail(12) = input.tail<12>();

  return {positionSetpoint, velocitySetpoint};
}

void AnymalRaisimConversions::extractModelData(double time, const raisim::ArticulatedSystem& sys) {}

Eigen::VectorXd AnymalRaisimConversions::inputToRaisimGeneralizedForce(double, const input_vector_t& input, const state_vector_t&,
                                                                       const Eigen::VectorXd& q, const Eigen::VectorXd& dq) const {
  const auto ocs2RbdState = raisimGenCoordGenVelToRbdState(q, dq);

  const switched_model::joint_coordinate_t qJoints = switched_model::getJointPositions(ocs2RbdState);

  const auto dynamicsTerms = wholebodyModelPtr_->getDynamicsTerms(ocs2RbdState);
  const auto& Mm = dynamicsTerms.M;
  const auto& Gv = dynamicsTerms.G;
  const auto& Cv = dynamicsTerms.C;

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
