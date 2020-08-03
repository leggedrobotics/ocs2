/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

#include <ocs2_ballbot_example/BallbotParameters.h>
#include <ocs2_ballbot_example/definitions.h>
#include <ocs2_robotic_tools/common/AngularVelocityMapping.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include <ocs2_ballbot_raisim_example/BallbotRaisimConversions.h>

namespace ocs2 {
namespace ballbot {

BallbotRaisimConversions::BallbotRaisimConversions() {
  BallbotParameters params;
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

std::pair<Eigen::VectorXd, Eigen::VectorXd> BallbotRaisimConversions::stateToRaisimGenCoordGenVel(const vector_t& state,
                                                                                                  const vector_t&) const {
  // assume ball is on the ground
  const double terrainHeight = (terrain_ == nullptr) ? 0.0 : terrain_->getHeight(state(0), state(1));
  const Eigen::Vector3d r_world_ball_inWorld{state(0), state(1), ballRadius_ + terrainHeight};
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

auto BallbotRaisimConversions::raisimGenCoordGenVelToState(const Eigen::VectorXd& q, const Eigen::VectorXd& dq) const -> vector_t {
  assert(q.size() == 3 + 4 + 4);
  assert(dq.size() == 3 + 3 + 3);
  if (dq.head<6>().cwiseAbs().maxCoeff() > 100.0) {
    throw std::runtime_error("BallbotRaisimConversions::raisimGenCoordGenVelToState: Body velocity diverged");
  }

  const auto r_world_ball_inWorld = ballCenterInWorld(q);
  const Eigen::Quaterniond q_world_base(q(3), q(4), q(5), q(6));  // w x y z

  const Eigen::Vector3d omega_base_inWorld = dq.segment<3>(3);
  Eigen::Vector3d eulerAngles = q_world_base.toRotationMatrix().eulerAngles(2, 1, 0);
  makeEulerAnglesUnique<double>(eulerAngles);
  if (eulerAngles.tail<2>().cwiseAbs().maxCoeff() > M_PI / 2) {
    throw std::runtime_error("BallbotRaisimConversions::raisimGenCoordGenVelToState: pitch or roll diverged");
  }

  vector_t state(ocs2::ballbot::STATE_DIM_);
  state(0) = r_world_ball_inWorld(0);  // ball x
  state(1) = r_world_ball_inWorld(1);  // ball y
  state.segment<3>(2) = eulerAngles;   // base ypr in EulerAngles ZYX convention
  state.segment<2>(5) = dq.head<2>() + omega_base_inWorld.cross(q_world_base * rbaseBallInBase_).head<2>();  // ball x-y velocity
  state.segment<3>(7) = angularVelocityInWorldToEulerAngleZyxDerivatives<double>(state.segment<3>(2), omega_base_inWorld);
  return state;
}

Eigen::VectorXd BallbotRaisimConversions::inputToRaisimGeneralizedForce(double time, const vector_t& input, const vector_t&,
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
