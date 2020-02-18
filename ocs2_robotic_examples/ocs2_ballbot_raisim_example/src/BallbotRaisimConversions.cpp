#include <ocs2_ballbot_raisim_example/BallbotRaisimConversions.h>

#include <ocs2_ballbot_example/BallbotParameters.h>
#include <ocs2_robotic_tools/common/AngularVelocityMapping.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

namespace ocs2 {
namespace ballbot {

BallbotRaisimConversions::BallbotRaisimConversions() {
  BallbotParameters<double> params;
  ballRadius_ = params.ballRadius_;
  omniWheelRadius_ = params.wheelRadius_;
  distanceBaseToBallCenter_ = params.heightBallCenterToBase_;
  rbaseBallInBase_ = Eigen::Vector3d{0, 0, -distanceBaseToBallCenter_};
}

Eigen::Vector3d BallbotRaisimConversions::ballCenterInWorld(const Eigen::VectorXd& q) const {
  const Eigen::Vector3d r_world_base_inWorld = q.head<3>();
  const Eigen::Quaterniond q_world_base(q(3), q(4), q(5), q(6));

  return r_world_base_inWorld + q_world_base * rbaseBallInBase_;
}

std::pair<Eigen::VectorXd, Eigen::VectorXd> BallbotRaisimConversions::stateToRaisimGenCoordGenVel(const state_vector_t& state,
                                                                                                  const input_vector_t&) const {
  // assume ball is on the ground
  const Eigen::Vector3d r_world_ball_inWorld{state(0), state(1), ballRadius_};
  const Eigen::Vector3d v_ball_inWorld{state(5), state(6), 0.0};
  const Eigen::Vector3d omega_base_inWorld = eulerAngleZyxDerivativesToAngularVelocityInWorld<double>(state.segment<3>(2), state.tail<3>());
  const Eigen::Quaterniond q_world_base = getQuaternionFromEulerAnglesZyx<double>(state.segment<3>(2));

  Eigen::VectorXd q(3 + 4 + 4);
  q.head<3>() = r_world_ball_inWorld - q_world_base * rbaseBallInBase_;
  q(3) = q_world_base.w();
  q(4) = q_world_base.x();
  q(5) = q_world_base.y();
  q(6) = q_world_base.z();
  q(7) = 1.0;  // unit quaternion -> rotation of the ball not relevant
  q.segment<3>(8).setZero();

  Eigen::VectorXd dq(3 + 3 + 3);
  dq.head<3>() = v_ball_inWorld + omega_base_inWorld.cross(q_world_base * (-rbaseBallInBase_));
  dq.segment<3>(3) = omega_base_inWorld;
  dq.tail<3>() = q_world_base.inverse() * Eigen::Vector3d::UnitZ().cross(v_ball_inWorld) / ballRadius_;

  return {q, dq};
}

auto BallbotRaisimConversions::raisimGenCoordGenVelToState(const Eigen::VectorXd& q, const Eigen::VectorXd& dq) const -> state_vector_t {
  assert(q.size() == 3 + 4 + 4);
  assert(dq.size() == 3 + 3 + 3);

  const auto r_world_ball_inWorld = ballCenterInWorld(q);
  const Eigen::Quaterniond q_world_base(q(3), q(4), q(5), q(6));  // w x y z

  const Eigen::Vector3d omega_base_inWorld = dq.segment<3>(3);
  Eigen::Vector3d eulerAngles = q_world_base.toRotationMatrix().eulerAngles(2, 1, 0);
  makeEulerAnglesUnique<double>(eulerAngles);

  state_vector_t state;
  state(0) = r_world_ball_inWorld(0);  // ball x
  state(1) = r_world_ball_inWorld(1);  // ball y
  state.segment<3>(2) = eulerAngles;   // base ypr in EulerAngles ZYX convention
  state.segment<2>(5) = dq.head<2>() + omega_base_inWorld.cross(q_world_base * rbaseBallInBase_).head<2>();  // ball x-y velocity
  state.segment<3>(7) = angularVelocityInWorldToEulerAngleZyxDerivatives<double>(state.segment<3>(2), omega_base_inWorld);
  return state;
}

Eigen::VectorXd BallbotRaisimConversions::inputToRaisimGeneralizedForce(double, const input_vector_t& input, const state_vector_t&,
                                                                        const Eigen::VectorXd& q, const Eigen::VectorXd&) const {
  const double geometricFactor = ballRadius_ / omniWheelRadius_;
  const Eigen::Matrix3d torqueTransformationMatrixWheelsToBase =
      geometricFactor * sqrt(2.0) / 2.0 *
      (Eigen::Matrix3d() << 1.0, -0.5, -0.5, 0.0, sqrt(3) / 2.0, -sqrt(3) / 2.0, -1.0, -1.0, -1.0).finished();
  const Eigen::Quaterniond q_world_base(q(3), q(4), q(5), q(6));  // w x y z

  Eigen::VectorXd raisimGeneralizedForce(Eigen::VectorXd::Zero(6 + 3));
  raisimGeneralizedForce.tail<3>() = q_world_base * (-torqueTransformationMatrixWheelsToBase) * input;
  return raisimGeneralizedForce;
}

}  // namespace ballbot
}  // namespace ocs2
