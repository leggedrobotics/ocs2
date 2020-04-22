#include <ocs2_anymal_raisim/AnymalRaisimConversions.h>

#include <ocs2_robotic_tools/common/RotationTransforms.h>
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
  if (terrain_ != nullptr) {
    q(2) += terrain_->getHeight(q(0), q(1));
  }

  Eigen::VectorXd dq(3 + 3 + 12);
  dq << q_world_base * ocs2RbdState.segment<3>(21), q_world_base * ocs2RbdState.segment<3>(18), ocs2RbdState.tail<12>();

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
  return switchedModelStateEstimator_.estimateComkinoModelState(raisimGenCoordGenVelToRbdState(q, dq));
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

}  // namespace anymal
