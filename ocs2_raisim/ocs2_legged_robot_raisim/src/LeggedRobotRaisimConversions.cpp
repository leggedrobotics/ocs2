/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

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

#include "ocs2_legged_robot_raisim/LeggedRobotRaisimConversions.h"

#include <ocs2_robotic_tools/common/RotationTransforms.h>

namespace ocs2 {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::pair<Eigen::VectorXd, Eigen::VectorXd> LeggedRobotRaisimConversions::stateToRaisimGenCoordGenVel(const vector_t& state,
                                                                                                      const vector_t& input) {
  // get RBD state
  const vector_t rbdState = centroidalModelRbdConversions_.computeRbdStateFromCentroidalModel(state, input);

  // convert to generalized coordinate and generalized velocity
  return rbdStateToRaisimGenCoordGenVel(rbdState);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t LeggedRobotRaisimConversions::raisimGenCoordGenVelToState(const Eigen::VectorXd& q, const Eigen::VectorXd& dq) {
  // get RBD state
  const vector_t rbdState = raisimGenCoordGenVelToRbdState(q, dq);

  // convert to state
  return centroidalModelRbdConversions_.computeCentroidalStateFromRbdModel(rbdState);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
Eigen::VectorXd LeggedRobotRaisimConversions::inputToRaisimGeneralizedForce(double time, const vector_t& input, const vector_t& state,
                                                                            const Eigen::VectorXd& q, const Eigen::VectorXd& dq) {
  // get RBD torque
  const vector_t desiredJointAccelerations = vector_t::Zero(12);  // TODO(areske): retrieve this from controller?
  const vector_t measuredRbdState = raisimGenCoordGenVelToRbdState(q, dq);
  const vector_t rbdTorque =
      centroidalModelRbdConversions_.computeRbdTorqueFromCentroidalModelPD(state, input, desiredJointAccelerations, measuredRbdState);

  // convert to generalized force
  return rbdTorqueToRaisimGeneralizedForce(rbdTorque);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::pair<Eigen::VectorXd, Eigen::VectorXd> LeggedRobotRaisimConversions::inputToRaisimPdTargets(double time, const vector_t& input,
                                                                                                 const vector_t& state,
                                                                                                 const Eigen::VectorXd& q,
                                                                                                 const Eigen::VectorXd& dq) {
  Eigen::VectorXd positionSetpoint = q;
  positionSetpoint.tail<12>() = ocs2JointOrderToRaisimJointOrder(state.tail<12>());
  Eigen::VectorXd velocitySetpoint = dq;
  velocitySetpoint.tail<12>() = ocs2JointOrderToRaisimJointOrder(input.tail<12>());
  return {positionSetpoint, velocitySetpoint};
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t LeggedRobotRaisimConversions::raisimJointOrderToOcs2JointOrder(const Eigen::VectorXd& raisimJoint) {
  vector_t ocs2Joint(12);
  ocs2Joint << raisimJoint.head<3>(), raisimJoint.segment<3>(6), raisimJoint.segment<3>(3), raisimJoint.tail<3>();
  return ocs2Joint;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
Eigen::VectorXd LeggedRobotRaisimConversions::ocs2JointOrderToRaisimJointOrder(const vector_t& ocs2Joint) {
  Eigen::VectorXd raisimJoint(12);
  raisimJoint << ocs2Joint.head<3>(), ocs2Joint.segment<3>(6), ocs2Joint.segment<3>(3), ocs2Joint.tail<3>();
  return raisimJoint;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::pair<Eigen::VectorXd, Eigen::VectorXd> LeggedRobotRaisimConversions::rbdStateToRaisimGenCoordGenVel(const vector_t& rbdState) {
  // set continuous orientation
  continuousOrientation_ = rbdState.head<3>();

  // convert to generalized coordinate
  Eigen::VectorXd q(19);
  const Eigen::Quaterniond quaternion = getQuaternionFromEulerAnglesZyx(continuousOrientation_);
  q << rbdState.segment<3>(3), quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z(),
      ocs2JointOrderToRaisimJointOrder(rbdState.segment<12>(6));

  // convert to generalized velocity
  Eigen::VectorXd dq(18);
  dq << rbdState.segment<3>(21), rbdState.segment<3>(18), ocs2JointOrderToRaisimJointOrder(rbdState.tail<12>());

  // height relative to terrain
  if (terrainPtr_ != nullptr) {
    q(2) += terrainPtr_->getHeight(q(0), q(1));
  }

  // check
  if (check_ && (q.tail<12>().array().abs().maxCoeff() > 3.0 * M_PI || dq.tail<12>().array().abs().maxCoeff() > 7.5)) {
    std::stringstream ss;
    ss << "LeggedRobotRaisimConversions::rbdStateToRaisimGenCoordGenVel -- RaiSim state violates actuator limits:"
       << "\nq = " << q.transpose() << "\ndq = " << dq.transpose();
    throw std::runtime_error(ss.str());
  }

  return {q, dq};
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t LeggedRobotRaisimConversions::raisimGenCoordGenVelToRbdState(const Eigen::VectorXd& q, const Eigen::VectorXd& dq) {
  // check
  if (check_ && (q.tail<12>().array().abs().maxCoeff() > 3.0 * M_PI || dq.tail<12>().array().abs().maxCoeff() > 7.5)) {
    std::stringstream ss;
    ss << "LeggedRobotRaisimConversions::raisimGenCoordGenVelToRbdState -- RaiSim state violates actuator limits:"
       << "\nq = " << q.transpose() << "\ndq = " << dq.transpose();
    throw std::runtime_error(ss.str());
  }

  // get continuous orientation
  const Eigen::Quaterniond quaternion(q(3), q(4), q(5), q(6));
  Eigen::Vector3d orientation = quaternion.toRotationMatrix().eulerAngles(2, 1, 0);
  ocs2::makeEulerAnglesUnique(orientation);
  const auto yaw = findOrientationClostestToReference(orientation[0], continuousOrientation_[0]);
  continuousOrientation_ << yaw, orientation[1], orientation[2];

  // convert to RBD state
  vector_t rbdState(36);
  rbdState << continuousOrientation_, q.head<3>(), raisimJointOrderToOcs2JointOrder(q.tail<12>()), dq.segment<3>(3), dq.head<3>(),
      raisimJointOrderToOcs2JointOrder(dq.tail<12>());

  // height relative to terrain
  if (terrainPtr_ != nullptr) {
    rbdState(5) -= terrainPtr_->getHeight(q(0), q(1));
  }

  return rbdState;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
Eigen::VectorXd LeggedRobotRaisimConversions::rbdTorqueToRaisimGeneralizedForce(const vector_t& rbdTorque) {
  // convert to generalized force
  Eigen::VectorXd generalizedForce = Eigen::VectorXd::Zero(18);
  generalizedForce.tail<12>() = ocs2JointOrderToRaisimJointOrder(rbdTorque.tail<12>());

  // check
  if (check_ && generalizedForce.array().abs().maxCoeff() > 80.0) {
    std::stringstream ss;
    ss << "LeggedRobotRaisimConversions::rbdTorqueToRaisimGeneralizedForce -- RaiSim input violates actuator limits:"
       << "\ngeneralizedForce = " << generalizedForce.transpose();
    throw std::runtime_error(ss.str());
  }

  return generalizedForce;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t LeggedRobotRaisimConversions::findOrientationClostestToReference(scalar_t yaw, scalar_t reference) {
  while (std::abs(reference - yaw) > M_PI) {
    yaw += std::copysign(scalar_t(2.0 * M_PI), reference - yaw);
  }
  return yaw;
}

}  // namespace legged_robot
}  // namespace ocs2
