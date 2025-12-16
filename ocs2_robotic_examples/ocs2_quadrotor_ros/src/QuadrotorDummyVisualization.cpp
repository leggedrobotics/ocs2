/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

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

#include "ocs2_quadrotor_ros/QuadrotorDummyVisualization.h"

#include <geometry_msgs/msg/transform_stamped.hpp>

namespace ocs2 {
namespace quadrotor {

void QuadrotorDummyVisualization::update(const SystemObservation& observation,
                                         const PrimalSolution& policy,
                                         const CommandData& command) {
  const auto& targetTrajectories = command.mpcTargetTrajectories_;

  // publish command transform
  const Eigen::Vector3d desiredPositionWorldToTarget(
      targetTrajectories.stateTrajectory.back()(0),
      targetTrajectories.stateTrajectory.back()(1),
      targetTrajectories.stateTrajectory.back()(2));
  const Eigen::Quaterniond desiredQuaternionBaseToWorld =
      Eigen::AngleAxisd{targetTrajectories.stateTrajectory.back()(3),
                        Eigen::Vector3d::UnitZ()} *
      Eigen::AngleAxisd{targetTrajectories.stateTrajectory.back()(4),
                        Eigen::Vector3d::UnitY()} *
      Eigen::AngleAxisd{targetTrajectories.stateTrajectory.back()(5),
                        Eigen::Vector3d::UnitX()};
  geometry_msgs::msg::TransformStamped command_frame_transform;
  command_frame_transform.header.stamp = node_->get_clock()->now();
  command_frame_transform.header.frame_id = "odom";
  command_frame_transform.child_frame_id = "command";
  command_frame_transform.transform.translation.x =
      desiredPositionWorldToTarget.x();
  command_frame_transform.transform.translation.y =
      desiredPositionWorldToTarget.y();
  command_frame_transform.transform.translation.z =
      desiredPositionWorldToTarget.z();
  command_frame_transform.transform.rotation.w =
      desiredQuaternionBaseToWorld.w();
  command_frame_transform.transform.rotation.x =
      desiredQuaternionBaseToWorld.x();
  command_frame_transform.transform.rotation.y =
      desiredQuaternionBaseToWorld.y();
  command_frame_transform.transform.rotation.z =
      desiredQuaternionBaseToWorld.z();
  tfBroadcaster_.sendTransform(command_frame_transform);

  const Eigen::Quaterniond q =
      Eigen::AngleAxisd{observation.state(3), Eigen::Vector3d::UnitZ()} *
      Eigen::AngleAxisd{observation.state(4), Eigen::Vector3d::UnitY()} *
      Eigen::AngleAxisd{observation.state(5), Eigen::Vector3d::UnitX()};
  geometry_msgs::msg::TransformStamped base_frame_transform;
  base_frame_transform.header.stamp = node_->get_clock()->now();
  base_frame_transform.header.frame_id = "world";
  base_frame_transform.child_frame_id = "base";
  base_frame_transform.transform.translation.x = observation.state(0);
  base_frame_transform.transform.translation.y = observation.state(1);
  base_frame_transform.transform.translation.z = observation.state(2);
  base_frame_transform.transform.rotation.w = q.w();
  base_frame_transform.transform.rotation.x = q.x();
  base_frame_transform.transform.rotation.y = q.y();
  base_frame_transform.transform.rotation.z = q.z();
  tfBroadcaster_.sendTransform(base_frame_transform);
}

}  // namespace quadrotor
}  // namespace ocs2
