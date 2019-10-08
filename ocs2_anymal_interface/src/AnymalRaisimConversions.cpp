#include <Eigen/Geometry>
#include <ocs2_anymal_interface/AnymalRaisimConversions.hpp>

namespace anymal {

std::pair<Eigen::VectorXd, Eigen::VectorXd> AnymalRaisimConversions::stateToRaisimGenCoordGenVel(const state_vector_t& state,
                                                                                                 const input_vector_t& input) {
  rbd_state_vector_t ocs2RbdState;
  anymalInterface_->computeRbdModelState(state, input, ocs2RbdState);

  // quaternion between world and base orientation
  const Eigen::Quaterniond q_world_base = Eigen::AngleAxisd(ocs2RbdState(0), Eigen::Vector3d::UnitX()) *
                                          Eigen::AngleAxisd(ocs2RbdState(1), Eigen::Vector3d::UnitY()) *
                                          Eigen::AngleAxisd(ocs2RbdState(2), Eigen::Vector3d::UnitZ());

  Eigen::VectorXd q(3 + 4 + 12);
  q << ocs2RbdState.segment<3>(3), q_world_base.w(), q_world_base.x(), q_world_base.y(), q_world_base.z(), ocs2RbdState.segment<12>(6);

  Eigen::VectorXd dq(3 + 3 + 12);
  dq << q_world_base * ocs2RbdState.segment<3>(21), q_world_base * ocs2RbdState.segment<3>(18), ocs2RbdState.tail<12>();

  return {q, dq};
}

AnymalRaisimConversions::rbd_state_vector_t AnymalRaisimConversions::raisimGenCoordGenVelToRbdState(const Eigen::VectorXd& q,
                                                                                                    const Eigen::VectorXd& dq) {
  Eigen::Quaterniond q_world_base(q(3), q(4), q(5), q(6));  // quaternion coefficients w, x, y z
  Eigen::Vector3d eulerAngles = q_world_base.toRotationMatrix().eulerAngles(0, 1, 2);
  makeEulerAnglesUnique(eulerAngles);

  rbd_state_vector_t ocs2RbdState;
  ocs2RbdState << eulerAngles, q.head<3>(), q.tail<12>(), q_world_base.inverse() * dq.segment<3>(3),
      q_world_base.inverse() * dq.segment<3>(0), dq.tail<12>();

  return ocs2RbdState;
}

AnymalRaisimConversions::state_vector_t AnymalRaisimConversions::raisimGenCoordGenVelToState(const Eigen::VectorXd& q,
                                                                                             const Eigen::VectorXd& dq) {
  state_vector_t state;
  anymalInterface_->computeSwitchedModelState(raisimGenCoordGenVelToRbdState(q, dq), state);
  return state;
}

Eigen::VectorXd AnymalRaisimConversions::inputToRaisimGeneralizedForce(double time, const input_vector_t& input,
                                                                       const state_vector_t& state, const Eigen::VectorXd& q,
                                                                       const Eigen::VectorXd& dq) {
  const rbd_state_vector_t ocs2RbdState = raisimGenCoordGenVelToRbdState(q, dq);

  Eigen::Matrix<double, 2 * (6 + 12), 1> ocs2RbdStateRef;
  Eigen::Matrix<double, 12, 1> ocs2RbdInput;
  size_t subsystem;
  mrt_->rolloutPolicy(time, ocs2RbdState, ocs2RbdStateRef, ocs2RbdInput, subsystem);

  Eigen::Matrix<double, 18, 1> raisimInput;
  raisimInput.setZero();
  raisimInput.tail<12>() = ocs2RbdInput;

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
