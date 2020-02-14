#include <ocs2_ballbot_raisim_example/BallbotRaisimConversions.h>

#include <ocs2_ballbot_example/BallbotParameters.h>

namespace ocs2 {
namespace ballbot {

// todo: Check against
// https://bitbucket.org/leggedrobotics/rezero_base/src/master/rezero_base_mpc_control/balance_controller/include/balance_controller/ConversionsOcs2.hpp
void makeEulerAnglesUnique(Eigen::Vector3d& eulerAngles) {
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

BallbotRaisimConversions::BallbotRaisimConversions() {
  BallbotParameters<double> params;
  ballRadius_ = params.ballRadius_;
  omniWheelRadius_ = params.wheelRadius_;
  distanceBaseToBallCenter_ = params.heightBallCenterToBase_;
}

Eigen::Vector3d BallbotRaisimConversions::ballCenterInWorld(const Eigen::VectorXd& q) const {
  const Eigen::Vector3d r_base_ball_inBase = (Eigen::Vector3d() << 0, 0, -distanceBaseToBallCenter_).finished();

  const Eigen::Vector3d r_world_base_inWorld = q.head<3>();
  const Eigen::Quaterniond q_world_base(q(3), q(4), q(5), q(6));

  return r_world_base_inWorld + q_world_base * r_base_ball_inBase;
}

Eigen::Vector3d BallbotRaisimConversions::angularVelocityInWorldToEulerAngleDerivatives(const Eigen::Vector3d& eulerAngles,
                                                                                        const Eigen::Vector3d& omega_world_base_inWorld) {
  const double cyaw = cos(eulerAngles(0));
  const double cpitch = cos(eulerAngles(1));

  const double syaw = sin(eulerAngles(0));
  const double spitch = sin(eulerAngles(1));

  assert(abs(cyaw) > 1e-8);  // test for singularity in debug mode

  Eigen::Matrix3d transform;
  transform << cyaw * spitch / cpitch, spitch * syaw / cpitch, 1,  // clang-format off
                                -syaw,                   cyaw, 0,
                        cyaw / cpitch,            syaw/cpitch, 0;  // clang-format on

  return transform * omega_world_base_inWorld;
}

Eigen::Vector3d BallbotRaisimConversions::eulerAngleDerivativesToAngularVelocityInWorld(const Eigen::Vector3d& eulerAngles,
                                                                                        const Eigen::Vector3d& eulerAnglesTimeDerivative) {
  const double cyaw = cos(eulerAngles(0));
  const double cpitch = cos(eulerAngles(1));

  const double syaw = sin(eulerAngles(0));
  const double spitch = sin(eulerAngles(1));

  Eigen::Matrix3d transform;
  transform << 0, -syaw, cpitch * cyaw,  // clang-format off
               0,  cyaw, cpitch * syaw,
               1,     0, -spitch;  // clang-format on

  return transform * eulerAnglesTimeDerivative;
}

std::pair<Eigen::VectorXd, Eigen::VectorXd> BallbotRaisimConversions::stateToRaisimGenCoordGenVel(const state_vector_t& state,
                                                                                                  const input_vector_t&) const {
  // assume ball is on the ground
  const Eigen::Vector3d r_world_ball_inWorld = (Eigen::Vector3d() << state(0), state(1), ballRadius_).finished();
  const Eigen::Vector3d r_base_ball_inBase = (Eigen::Vector3d() << 0, 0, -distanceBaseToBallCenter_).finished();
  const Eigen::Vector3d v_ball_inWorld = (Eigen::Vector3d() << state(5), state(6), 0.0).finished();
  const Eigen::Vector3d omega_base_inWorld = eulerAngleDerivativesToAngularVelocityInWorld(state.segment<3>(2), state.tail<3>());
  const Eigen::Quaterniond q_world_base = Eigen::AngleAxisd(state(2), Eigen::Vector3d::UnitZ()) *
                                          Eigen::AngleAxisd(state(3), Eigen::Vector3d::UnitY()) *
                                          Eigen::AngleAxisd(state(4), Eigen::Vector3d::UnitX());

  Eigen::VectorXd q(3 + 4 + 4);
  q.head<3>() = r_world_ball_inWorld - q_world_base * r_base_ball_inBase;
  q(3) = q_world_base.w();
  q(4) = q_world_base.x();
  q(5) = q_world_base.y();
  q(6) = q_world_base.z();
  q(7) = 1.0;  // unit quaternion -> actual rotation of the ball not relevant
  q.segment<3>(8).setZero();

  Eigen::VectorXd dq(3 + 3 + 3);
  dq.head<3>() = v_ball_inWorld + omega_base_inWorld.cross(q_world_base * (-r_base_ball_inBase));
  dq.segment<3>(3) = omega_base_inWorld;
  dq.tail<3>() = q_world_base.inverse() * Eigen::Vector3d::UnitZ().cross(v_ball_inWorld) / ballRadius_;

  return {q, dq};
}

auto BallbotRaisimConversions::raisimGenCoordGenVelToState(const Eigen::VectorXd& q, const Eigen::VectorXd& dq) const -> state_vector_t {
  assert(q.size() == 3 + 4 + 4);
  assert(dq.size() == 3 + 3 + 3);

  const auto r_world_ball_inWorld = ballCenterInWorld(q);
  const Eigen::Quaterniond q_world_base(q(3), q(4), q(5), q(6));  // w x y z

  Eigen::Vector3d r_base_ball_inBase;
  r_base_ball_inBase << 0, 0, -distanceBaseToBallCenter_;

  const Eigen::Vector3d omega_base_inWorld = dq.segment<3>(3);
  Eigen::Vector3d eulerAngles = q_world_base.toRotationMatrix().eulerAngles(2, 1, 0);
  makeEulerAnglesUnique(eulerAngles);

  state_vector_t state;
  state(0) = r_world_ball_inWorld(0);  // ball x
  state(1) = r_world_ball_inWorld(1);  // ball y
  state.segment<3>(2) = eulerAngles;   // base ypr in EulerAngles ZYX convention
  state.segment<2>(5) = dq.head<2>() + omega_base_inWorld.cross(q_world_base * r_base_ball_inBase).head<2>();  // ball x-y velocity
  state.segment<3>(7) = angularVelocityInWorldToEulerAngleDerivatives(state.segment<3>(2), omega_base_inWorld);
  return state;
}

Eigen::VectorXd BallbotRaisimConversions::inputToRaisimGeneralizedForce(double, const input_vector_t& input, const state_vector_t&,
                                                                        const Eigen::VectorXd&, const Eigen::VectorXd&) const {
  const double geometricFactor = ballRadius_ / omniWheelRadius_;
  const Eigen::Matrix3d torqueTransformationMatrixWheelsToBase =
      geometricFactor * sqrt(2.0) / 2.0 *
      (Eigen::Matrix3d() << 1.0, -0.5, -0.5, 0.0, sqrt(3) / 2.0, -sqrt(3) / 2.0, -1.0, -1.0, -1.0).finished();

  Eigen::VectorXd raisimGeneralizedForce(Eigen::VectorXd::Zero(6 + 3));
  raisimGeneralizedForce.tail<3>() = -torqueTransformationMatrixWheelsToBase * input;
  return raisimGeneralizedForce;
}

}  // namespace ballbot
}  // namespace ocs2
