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

#include <tf/tf.h>
#include <urdf/model.h>
#include <visualization_msgs/Marker.h>
#include <kdl_parser/kdl_parser.hpp>

#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include "ocs2_ballbot_ros/BallbotDummyVisualization.h"

namespace ocs2 {
namespace ballbot {

void BallbotDummyVisualization::update(const SystemObservation& observation, const PrimalSolution& policy, const CommandData& command) {
  const auto& targetTrajectories = command.mpcTargetTrajectories_;

  // publish world transform
  ros::Time timeMsg = ros::Time::now();
  geometry_msgs::TransformStamped world_transform;
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
  const Eigen::Vector3d desiredPositionWorldToTarget(targetTrajectories.stateTrajectory.back()(0),
                                                     targetTrajectories.stateTrajectory.back()(1), 0.0);
  const auto desiredQuaternionBaseToWorld =
      getQuaternionFromEulerAnglesZyx<double>(targetTrajectories.stateTrajectory.back().segment<3>(2));
  geometry_msgs::TransformStamped command_frame_transform;
  command_frame_transform.header.stamp = timeMsg;
  command_frame_transform.header.frame_id = "odom";
  command_frame_transform.child_frame_id = "command";
  command_frame_transform.transform.translation.x = desiredPositionWorldToTarget.x();
  command_frame_transform.transform.translation.y = desiredPositionWorldToTarget.y();
  command_frame_transform.transform.translation.z = desiredPositionWorldToTarget.z();
  command_frame_transform.transform.rotation.w = desiredQuaternionBaseToWorld.w();
  command_frame_transform.transform.rotation.x = desiredQuaternionBaseToWorld.x();
  command_frame_transform.transform.rotation.y = desiredQuaternionBaseToWorld.y();
  command_frame_transform.transform.rotation.z = desiredQuaternionBaseToWorld.z();
  tfBroadcaster_.sendTransform(command_frame_transform);

  // publish joints transforms
  std::map<std::string, double> jointPositions{{"jball_x", observation.state(0)},
                                               {"jball_y", observation.state(1)},
                                               {"jbase_z", observation.state(2)},
                                               {"jbase_y", observation.state(3)},
                                               {"jbase_x", observation.state(4)}};
  robotStatePublisherPtr_->publishTransforms(jointPositions, timeMsg);
}

void BallbotDummyVisualization::launchVisualizerNode(ros::NodeHandle& nodeHandle) {
  // load a kdl-tree from the urdf robot description and initialize the robot state publisher
  std::string urdfName = "robot_description";
  urdf::Model model;
  if (!model.initParam(urdfName)) {
    ROS_ERROR("URDF model load was NOT successful");
  }
  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(model, tree)) {
    ROS_ERROR("Failed to extract kdl tree from xml robot description");
  }

  robotStatePublisherPtr_.reset(new robot_state_publisher::RobotStatePublisher(tree));
  robotStatePublisherPtr_->publishFixedTransforms(true);
}

}  // namespace ballbot
}  // namespace ocs2
