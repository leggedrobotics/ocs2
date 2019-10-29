#include <ocs2_anymal_raisim/AnymalRaisimConversions.h>
#include <ocs2_anymal_switched_model/core/AnymalKinematics.h>
#include <ocs2_anymal_switched_model/generated/inverse_dynamics.h>
#include <ocs2_anymal_switched_model/generated/jsim.h>
#include <ocs2_switched_model_interface/core/Rotations.h>

namespace anymal {

std::pair<Eigen::VectorXd, Eigen::VectorXd> AnymalRaisimConversions::stateToRaisimGenCoordGenVel(const state_vector_t& state,
                                                                                                 const input_vector_t& input) const {
  const auto ocs2RbdState =
      switchedModelStateEstimator_.estimateRbdModelState(state, input.segment<switched_model::JOINT_COORDINATE_SIZE>(12));

  // quaternion between world and base orientation
  const Eigen::Quaterniond q_world_base = switched_model::quaternionBaseToOrigin<double>(ocs2RbdState.head<3>());

  Eigen::VectorXd q(3 + 4 + 12);
  q << ocs2RbdState.segment<3>(3), q_world_base.w(), q_world_base.x(), q_world_base.y(), q_world_base.z(), ocs2RbdState.segment<12>(6);

  Eigen::VectorXd dq(3 + 3 + 12);
  dq << q_world_base * ocs2RbdState.segment<3>(21), q_world_base * ocs2RbdState.segment<3>(18), ocs2RbdState.tail<12>();

  return {q, dq};
}

switched_model::rbd_state_t AnymalRaisimConversions::raisimGenCoordGenVelToRbdState(const Eigen::VectorXd& q, const Eigen::VectorXd& dq) {
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
  makeEulerAnglesUnique(eulerAngles);

  switched_model::rbd_state_t ocs2RbdState;
  ocs2RbdState << eulerAngles, q.head<3>(), q.tail<12>(), q_world_base.inverse() * dq.segment<3>(3),
      q_world_base.inverse() * dq.segment<3>(0), dq.tail<12>();

  return ocs2RbdState;
}

AnymalRaisimConversions::state_vector_t AnymalRaisimConversions::raisimGenCoordGenVelToState(const Eigen::VectorXd& q,
                                                                                             const Eigen::VectorXd& dq) const {
  return switchedModelStateEstimator_.estimateComkinoModelState(raisimGenCoordGenVelToRbdState(q, dq));
}

Eigen::VectorXd AnymalRaisimConversions::inputToRaisimGeneralizedForce(double time, const input_vector_t& input,
                                                                       const state_vector_t& state, const Eigen::VectorXd& q,
                                                                       const Eigen::VectorXd& dq) const {
  const auto ocs2RbdState = raisimGenCoordGenVelToRbdState(q, dq);

  const switched_model::base_coordinate_t qBase = switched_model::getBasePose(ocs2RbdState);
  const switched_model::joint_coordinate_t qJoints = switched_model::getJointPositions(ocs2RbdState);
  const switched_model::base_coordinate_t qdBase = switched_model::getBaseLocalVelocity(ocs2RbdState);
  const switched_model::joint_coordinate_t qdJoints = switched_model::getJointVelocities(ocs2RbdState);

  AnymalKinematics kinematics;

  // force transformed in the lowerLeg coordinate
  switched_model::base_coordinate_t extForceBase = switched_model::base_coordinate_t::Zero();
  for (int j = 0; j < 4; j++) {
    const auto b_footPosition = kinematics.positionBaseToFootInBaseFrame(j, qJoints);
    extForceBase.head<3>() += b_footPosition.cross(input.segment<3>(3 * j));
    extForceBase.tail<3>() += input.segment<3>(3 * j);
  }
  switched_model::joint_coordinate_t extForceJoint = switched_model::joint_coordinate_t::Zero();
  for (int j = 0; j < 4; j++) {
    const auto b_footJacobian = kinematics.baseToFootJacobianInBaseFrame(j, qJoints);
    extForceJoint += b_footJacobian.bottomRows<3>().transpose() * input.segment<3>(3 * j);
  }

  iit::ANYmal::dyn::InertiaProperties inertias;
  iit::ANYmal::ForceTransforms forceTransforms;
  iit::ANYmal::dyn::JSIM Mm(inertias, forceTransforms);
  Mm.update(qJoints);

  iit::ANYmal::MotionTransforms MotionTransforms;
  iit::ANYmal::dyn::InverseDynamics inverseDynamics(inertias, MotionTransforms);
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

  // p gains on joint velocity level
  switched_model::joint_coordinate_t kp;
  kp.setConstant(5.0);
  ocs2rbdInput += kp.asDiagonal() * (input.tail<12>() - qdJoints);

  // convert to raisim input
  Eigen::Matrix<double, 18, 1> raisimInput;
  raisimInput.setZero();
  raisimInput.tail<12>() = ocs2rbdInput;

  return raisimInput;
}

void AnymalRaisimConversions::extractModelData(double time, const raisim::ArticulatedSystem& sys) {}

void AnymalRaisimConversions::makeEulerAnglesUnique(Eigen::Vector3d& eulerAngles) {
  double tol = 1e-9;

  if (eulerAngles.y() < -M_PI / 2 - tol) {
    if (eulerAngles.x() < 0) {
      eulerAngles.x() = eulerAngles.x() + M_PI;
    } else {
      eulerAngles.x() = eulerAngles.x() - M_PI;
    }

    eulerAngles.y() = -(eulerAngles.y() + M_PI);

    if (eulerAngles.z() < 0) {
      eulerAngles.z() = eulerAngles.z() + M_PI;
    } else {
      eulerAngles.z() = eulerAngles.z() - M_PI;
    }
  } else if (-M_PI / 2 - tol <= eulerAngles.y() && eulerAngles.y() <= -M_PI / 2 + tol) {
    eulerAngles.x() -= eulerAngles.z();
    eulerAngles.z() = 0;
  } else if (-M_PI / 2 + tol < eulerAngles.y() && eulerAngles.y() < M_PI / 2 - tol) {
    // ok
  } else if (M_PI / 2 - tol <= eulerAngles.y() && eulerAngles.y() <= M_PI / 2 + tol) {
    // todo: M_PI/2 should not be in range, other formula?
    eulerAngles.x() += eulerAngles.z();
    eulerAngles.z() = 0;
  } else  // M_PI/2 + tol < eulerAngles.y()
  {
    if (eulerAngles.x() < 0) {
      eulerAngles.x() = eulerAngles.x() + M_PI;
    } else {
      eulerAngles.x() = eulerAngles.x() - M_PI;
    }

    eulerAngles.y() = -(eulerAngles.y() - M_PI);

    if (eulerAngles.z() < 0) {
      eulerAngles.z() = eulerAngles.z() + M_PI;
    } else {
      eulerAngles.z() = eulerAngles.z() - M_PI;
    }
  }
}

}  // namespace anymal
