#include <Eigen/Geometry>
#include <ocs2_anymal_interface/AnymalRaisimConversions.hpp>

namespace anymal {

std::pair<Eigen::VectorXd, Eigen::VectorXd> AnymalRaisimConversions::stateToRaisimGenCoordGenVel(
    const Eigen::Matrix<double, 24, 1>& state, const Eigen::Matrix<double, 24, 1>& input) {
  std::cout << "ocs2 state: " << state.transpose() << std::endl;

  // ocs2RbdState is: rpy [0-2], xyz [3-5] , qJoints [6-17], baseLocalVelocities [18-23], dqJoints [24-35]
  Eigen::Matrix<double, 2 * (6 + 12), 1> ocs2RbdState;
  anymalInterface_->computeRbdModelState(state, input, ocs2RbdState);

  std::cout << "ocs2 rbd state: " << ocs2RbdState.transpose() << std::endl;

  // quaternion between world and base orientation
  Eigen::Quaterniond q_world_base = Eigen::AngleAxisd(ocs2RbdState(0), Eigen::Vector3d::UnitX()) *
                                    Eigen::AngleAxisd(ocs2RbdState(1), Eigen::Vector3d::UnitY()) *
                                    Eigen::AngleAxisd(ocs2RbdState(2), Eigen::Vector3d::UnitZ());

  // Raisim q: position (x y z), quaternion (w x y z), join positions
  Eigen::VectorXd q(3 + 4 + 12);
  q << ocs2RbdState.segment<3>(3), q_world_base.w(), q_world_base.x(), q_world_base.y(), q_world_base.z(), ocs2RbdState.segment<12>(6);

  // Raisim dq: linear vel in world, angular vel in world, joint vel
  Eigen::VectorXd dq(3 + 3 + 12);
  dq << q_world_base * ocs2RbdState.segment<3>(21), q_world_base * ocs2RbdState.segment<3>(18), ocs2RbdState.tail<12>();

  return {q, dq};
}

Eigen::Matrix<double, 24, 1> AnymalRaisimConversions::raisimGenCoordGenVelToState(const Eigen::VectorXd& q, const Eigen::VectorXd& dq) {
  Eigen::Quaterniond q_world_base(q(3), q(4), q(5), q(6));  // quaternion coefficients w, x, y z
  Eigen::Vector3d eulerAngles = q_world_base.toRotationMatrix().eulerAngles(0, 1, 2);

  Eigen::Matrix<double, 2 * (6 + 12), 1> ocs2RbdState;
  ocs2RbdState << eulerAngles, q.head<3>(), q.tail<12>(), q_world_base.inverse() * dq.segment<3>(3),
      q_world_base.inverse() * dq.segment<3>(0), dq.tail<12>();

  Eigen::Matrix<double, 24, 1> state;
  anymalInterface_->computeSwitchedModelState(ocs2RbdState, state);
  return state;
}

Eigen::VectorXd AnymalRaisimConversions::inputToRaisimGeneralizedForce(double time, const Eigen::Matrix<double, 24, 1>& input,
                                                                       const Eigen::Matrix<double, 24, 1>& state) {
  std::cout << "inputToRaisimGeneralizedForce ...";
  Eigen::Matrix<double, 2 * (6 + 12), 1> ocs2RbdState;
  Eigen::Matrix<double, 18, 1> raisimInput;
  raisimInput.setZero();

  Eigen::Matrix<double, 12, 1> ocs2RbdInput;
  size_t subsystem;
  mrt_->rolloutPolicy(time, state, ocs2RbdState, ocs2RbdInput, subsystem);
  raisimInput.tail<12>() = ocs2RbdInput;

  std::cout << " done." << std::endl;
  return raisimInput;
}

void AnymalRaisimConversions::extractModelData(double time, const raisim::ArticulatedSystem& sys) {}

}  // namespace anymal
