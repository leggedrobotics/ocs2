#include <Eigen/Geometry>
#include <ocs2_anymal_interface/AnymalRaisimConversions.hpp>

void make_euler_angles_unique(Eigen::Vector3d& eulerAngles) {
  // wrap angles into [-pi,pi),[-pi/2,pi/2),[-pi,pi)
  // from: https://github.com/ANYbotics/kindr/blob/0b159ec60b710706656b70148211ed04573fbfda/include/kindr/rotations/EulerAnglesXyz.hpp

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

namespace anymal {

std::pair<Eigen::VectorXd, Eigen::VectorXd> AnymalRaisimConversions::stateToRaisimGenCoordGenVel(
    const Eigen::Matrix<double, 24, 1>& state, const Eigen::Matrix<double, 24, 1>& input) {
  // ocs2RbdState is: rpy [0-2], eulerAngles [3-5] , qJoints [6-17], baseLocalVelocities [18-23], dqJoints [24-35]
  Eigen::Matrix<double, 2 * (6 + 12), 1> ocs2RbdState;
  anymalInterface_->computeRbdModelState(state, input, ocs2RbdState);

  // quaternion between world and base orientation
  Eigen::Quaterniond q_world_base = Eigen::AngleAxisd(ocs2RbdState(2), Eigen::Vector3d::UnitZ()) *
                                    Eigen::AngleAxisd(ocs2RbdState(1), Eigen::Vector3d::UnitY()) *
                                    Eigen::AngleAxisd(ocs2RbdState(0), Eigen::Vector3d::UnitX());

  // Raisim q: position (x y z), quaternion (w x y z), joint positions
  Eigen::VectorXd q(3 + 4 + 12);
  q << ocs2RbdState.segment<3>(3), q_world_base.w(), q_world_base.x(), q_world_base.y(), q_world_base.z(), ocs2RbdState.segment<12>(6);

  // Raisim dq: linear vel in world, angular vel in world, joint vel
  Eigen::VectorXd dq(3 + 3 + 12);
  dq << q_world_base * ocs2RbdState.segment<3>(21), q_world_base * ocs2RbdState.segment<3>(18), ocs2RbdState.tail<12>();

  //  std::cout << "stateToRaisimGenCoordGenVel:";
  //  std::cout << "\n\tocs2 state: " << state.transpose();
  //  std::cout << "\n\tocs2 rbd state: " << ocs2RbdState.transpose();
  //  std::cout << "\n\t raisim_q " << q.transpose();
  //  std::cout << "\n\t raisim_dq " << dq.transpose() << std::endl;

  return {q, dq};
}

Eigen::Matrix<double, 24, 1> AnymalRaisimConversions::raisimGenCoordGenVelToState(const Eigen::VectorXd& q, const Eigen::VectorXd& dq) {
  Eigen::Quaterniond q_world_base(q(3), q(4), q(5), q(6));  // quaternion coefficients w, x, y z
  Eigen::Vector3d eulerAngles = q_world_base.toRotationMatrix().eulerAngles(2, 1, 0);
  make_euler_angles_unique(eulerAngles);

  Eigen::Matrix<double, 2 * (6 + 12), 1> ocs2RbdState;
  ocs2RbdState << eulerAngles(2), eulerAngles(1), eulerAngles(0), q.head<3>(), q.tail<12>(), q_world_base.inverse() * dq.segment<3>(3),
      q_world_base.inverse() * dq.segment<3>(0), dq.tail<12>();

  Eigen::Matrix<double, 24, 1> state;
  anymalInterface_->computeSwitchedModelState(ocs2RbdState, state);

  //  std::cout << "raisimGenCoordGenVelToState:";
  //  std::cout << "\n\t raisim_q " << q.transpose();
  //  std::cout << "\n\t raisim_dq " << dq.transpose() << std::endl;
  //  std::cout << "\n\tocs2 rbd state: " << ocs2RbdState.transpose();
  //  std::cout << "\n\tocs2 state: " << state.transpose() << std::endl;

  return state;
}

Eigen::VectorXd AnymalRaisimConversions::inputToRaisimGeneralizedForce(double time, const Eigen::Matrix<double, 24, 1>& input,
                                                                       const Eigen::Matrix<double, 24, 1>& state) {
  // TODO: pass raisim q, dq here, too

  Eigen::Matrix<double, 2 * (6 + 12), 1> ocs2RbdState;
  anymalInterface_->computeRbdModelState(state, input, ocs2RbdState);

  //    std::cout << "inputToRaisimGeneralizedForce ...";
  Eigen::Matrix<double, 2 * (6 + 12), 1> ocs2RbdStateRef;
  Eigen::Matrix<double, 18, 1> raisimInput;
  raisimInput.setZero();

  Eigen::Matrix<double, 12, 1> ocs2RbdInput;
  size_t subsystem;
  mrt_->rolloutPolicy(time, ocs2RbdState, ocs2RbdStateRef, ocs2RbdInput, subsystem);
  raisimInput.tail<12>() = ocs2RbdInput;

  //  std::cout << " done." << std::endl;
  return raisimInput;
}

void AnymalRaisimConversions::extractModelData(double time, const raisim::ArticulatedSystem& sys) {}

}  // namespace anymal
