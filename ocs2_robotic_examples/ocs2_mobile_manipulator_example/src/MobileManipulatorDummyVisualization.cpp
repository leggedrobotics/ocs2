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
#include <kdl_parser/kdl_parser.hpp>

#include "ocs2_mobile_manipulator_example/MobileManipulatorDummyVisualization.h"
#include "ocs2_mobile_manipulator_example/definitions.h"

namespace mobile_manipulator {

/* Helper functions */
// TODO(mspieler): move to separate file
static vector_t getArmJointPositions(vector_t state) {
  return state.tail(6);
}

static Eigen::Vector3d getBasePosition(vector_t state) {
  Eigen::Vector3d position;
  position << state(0), state(1), 0.0;
  return position;
}

static Eigen::Quaterniond getBaseOrientation(vector_t state) {
  return Eigen::Quaterniond(Eigen::AngleAxisd(state(2), Eigen::Vector3d::UnitZ()));
}

static geometry_msgs::Vector3 getVectorMsg(const Eigen::Vector3d& vec) {
  geometry_msgs::Vector3 vecMsg;
  vecMsg.x = vec.x();
  vecMsg.y = vec.y();
  vecMsg.z = vec.z();
  return vecMsg;
}

static geometry_msgs::Quaternion getOrientationMsg(const Eigen::Quaterniond& orientation) {
  geometry_msgs::Quaternion orientationMsg;
  orientationMsg.x = orientation.x();
  orientationMsg.y = orientation.y();
  orientationMsg.z = orientation.z();
  orientationMsg.w = orientation.w();
  return orientationMsg;
}

void MobileManipulatorDummyVisualization::update(const ocs2::SystemObservation& observation, const ocs2::PrimalSolution& policy,
                                                 const ocs2::CommandData& command) {
  const ros::Time timeMsg = ros::Time::now();

  // publish world -> base transform
  const auto position = getBasePosition(observation.state);
  const auto orientation = getBaseOrientation(observation.state);
  geometry_msgs::TransformStamped world_transform;
  world_transform.header.stamp = timeMsg;
  world_transform.header.frame_id = "world";
  world_transform.child_frame_id = "base";
  world_transform.transform.translation = getVectorMsg(position);
  world_transform.transform.rotation = getOrientationMsg(orientation);
  tfBroadcaster_.sendTransform(world_transform);

  // publish joints transforms
  const auto j_arm = getArmJointPositions(observation.state);
  std::map<std::string, double> jointPositions{{"SH_ROT", j_arm(0)}, {"SH_FLE", j_arm(1)}, {"EL_FLE", j_arm(2)},
                                               {"EL_ROT", j_arm(3)}, {"WR_FLE", j_arm(4)}, {"WR_ROT", j_arm(5)}};
  robotStatePublisherPtr_->publishTransforms(jointPositions, timeMsg, "");
}

void MobileManipulatorDummyVisualization::launchVisualizerNode(ros::NodeHandle& nodeHandle) {
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
  robotStatePublisherPtr_->publishFixedTransforms("", true);
}

}  // namespace mobile_manipulator
