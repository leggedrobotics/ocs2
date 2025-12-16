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

#include "ocs2_ballbot_ros/BallbotDummyVisualization.h"

#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <urdf/model.h>

#include <kdl_parser/kdl_parser.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace ocs2 {
namespace ballbot {

BallbotDummyVisualization::BallbotDummyVisualization(
    const rclcpp::Node::SharedPtr& node)
    : node_(node),
      tfBroadcaster_(node),
      jointPublisher_(node_->create_publisher<sensor_msgs::msg::JointState>(
          "joint_states", 1)) {}

void BallbotDummyVisualization::update(const SystemObservation& observation,
                                       const PrimalSolution& policy,
                                       const CommandData& command) {
  const auto& targetTrajectories = command.mpcTargetTrajectories_;

  // publish world transform
  builtin_interfaces::msg::Time timeMsg = node_->get_clock()->now();
  geometry_msgs::msg::TransformStamped world_transform;
  world_transform.header.stamp = timeMsg;
  world_transform.header.frame_id = "odom";
  world_transform.child_frame_id = "world";
  world_transform.transform.translation.x = 0.0;
  world_transform.transform.translation.y = 0.0;
  world_transform.transform.translation.z = 0.0;
  world_transform.transform.rotation.x = 0.0;
  world_transform.transform.rotation.y = 0.0;
  world_transform.transform.rotation.z = 0.0;
  world_transform.transform.rotation.w = 1.0;
  tfBroadcaster_.sendTransform(world_transform);

  // publish command transform
  const Eigen::Vector3d desiredPositionWorldToTarget(
      targetTrajectories.stateTrajectory.back()(0),
      targetTrajectories.stateTrajectory.back()(1), 0.0);
  const auto desiredQuaternionBaseToWorld =
      getQuaternionFromEulerAnglesZyx<double>(
          targetTrajectories.stateTrajectory.back().segment<3>(2));
  geometry_msgs::msg::TransformStamped command_frame_transform;
  command_frame_transform.header.stamp = timeMsg;
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

  // publish joints transforms
  sensor_msgs::msg::JointState joint_state;
  joint_state.header.stamp = node_->get_clock()->now();
  joint_state.name.resize(5);
  joint_state.position.resize(5);
  joint_state.name[0] = "jball_x";
  joint_state.name[1] = "jball_y";
  joint_state.name[2] = "jbase_z";
  joint_state.name[3] = "jbase_y";
  joint_state.name[4] = "jbase_x";
  joint_state.position[0] = observation.state(0);
  joint_state.position[1] = observation.state(1);
  joint_state.position[2] = observation.state(2);
  joint_state.position[3] = observation.state(3);
  joint_state.position[4] = observation.state(4);
  jointPublisher_->publish(joint_state);
}

}  // namespace ballbot
}  // namespace ocs2
