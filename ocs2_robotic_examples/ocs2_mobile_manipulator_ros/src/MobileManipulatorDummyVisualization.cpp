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

#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/misc/LoadStdVectorOfPair.h>
#include <ocs2_mobile_manipulator/AccessHelperFunctions.h>
#include <ocs2_mobile_manipulator/FactoryFunctions.h>
#include <ocs2_mobile_manipulator/ManipulatorModelInfo.h>
#include <ocs2_mobile_manipulator/MobileManipulatorInterface.h>
#include <ocs2_mobile_manipulator_ros/MobileManipulatorDummyVisualization.h>
#include <ocs2_ros_interfaces/common/RosMsgHelpers.h>
#include <urdf/model.h>

#include <chrono>
#include <geometry_msgs/msg/pose_array.hpp>
#include <iostream>
#include <kdl_parser/kdl_parser.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/fwd.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "rclcpp/rclcpp.hpp"

namespace ocs2 {
namespace mobile_manipulator {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename It>
void assignHeader(It firstIt, It lastIt, const std_msgs::msg::Header& header) {
  for (; firstIt != lastIt; ++firstIt) {
    firstIt->header = header;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename It>
void assignIncreasingId(It firstIt, It lastIt, int startId = 0) {
  for (; firstIt != lastIt; ++firstIt) {
    firstIt->id = startId++;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MobileManipulatorDummyVisualization::launchVisualizerNode() {
  jointPublisher_ =
      node_->create_publisher<sensor_msgs::msg::JointState>("joint_states", 1);
  stateOptimizedPublisher_ =
      node_->create_publisher<visualization_msgs::msg::MarkerArray>(
          "/mobile_manipulator/optimizedStateTrajectory", 1);
  stateOptimizedPosePublisher_ =
      node_->create_publisher<geometry_msgs::msg::PoseArray>(
          "/mobile_manipulator/optimizedPoseTrajectory", 1);
  // Get ROS parameter
  std::string taskFile = node_->get_parameter("taskFile").as_string();
  std::string urdfFile = node_->get_parameter("urdfFile").as_string();
  // read manipulator type
  ManipulatorModelType modelType = mobile_manipulator::loadManipulatorType(
      taskFile, "model_information.manipulatorModelType");
  // read the joints to make fixed
  loadData::loadStdVector<std::string>(
      taskFile, "model_information.removeJoints", removeJointNames_, false);
  // read if self-collision checking active
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  bool activateSelfCollision = true;
  loadData::loadPtreeValue(pt, activateSelfCollision, "selfCollision.activate",
                           true);
  // create pinocchio interface
  PinocchioInterface pinocchioInterface(
      mobile_manipulator::createPinocchioInterface(urdfFile, modelType,
                                                   removeJointNames_));
  // activate markers for self-collision visualization
  if (activateSelfCollision) {
    std::vector<std::pair<size_t, size_t>> collisionObjectPairs;
    loadData::loadStdVectorOfPair(taskFile,
                                  "selfCollision.collisionObjectPairs",
                                  collisionObjectPairs, true);
    PinocchioGeometryInterface geomInterface(pinocchioInterface,
                                             collisionObjectPairs);
    // set geometry visualization markers
    geometryVisualization_.reset(new GeometryInterfaceVisualization(
        std::move(pinocchioInterface), geomInterface));
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MobileManipulatorDummyVisualization::update(
    const SystemObservation& observation, const PrimalSolution& policy,
    const CommandData& command) {
  const rclcpp::Time timeStamp = node_->get_clock()->now();

  publishObservation(timeStamp, observation);
  publishTargetTrajectories(timeStamp, command.mpcTargetTrajectories_);
  publishOptimizedTrajectory(timeStamp, policy);
  if (geometryVisualization_ != nullptr) {
    geometryVisualization_->publishDistances(observation.state);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MobileManipulatorDummyVisualization::publishObservation(
    const rclcpp::Time& timeStamp, const SystemObservation& observation) {
  // publish world -> base transform
  const auto r_world_base = getBasePosition(observation.state, modelInfo_);
  const Eigen::Quaternion<scalar_t> q_world_base =
      getBaseOrientation(observation.state, modelInfo_);

  geometry_msgs::msg::TransformStamped base_tf;
  base_tf.header.stamp = timeStamp;
  base_tf.header.frame_id = "world";
  base_tf.child_frame_id = modelInfo_.baseFrame;
  base_tf.transform.translation = ros_msg_helpers::getVectorMsg(r_world_base);
  base_tf.transform.rotation = ros_msg_helpers::getOrientationMsg(q_world_base);
  tfBroadcaster_.sendTransform(base_tf);

  // publish joints transforms
  const auto j_arm = getArmJointAngles(observation.state, modelInfo_);
  sensor_msgs::msg::JointState joint_state;
  joint_state.header.stamp = node_->get_clock()->now();
  const auto dofNames_count = modelInfo_.dofNames.size();
  const auto joint_count = dofNames_count + removeJointNames_.size();
  joint_state.name.resize(joint_count);
  joint_state.position.resize(joint_count);
  for (size_t i = 0; i < dofNames_count; i++) {
    joint_state.name[i] = modelInfo_.dofNames[i];
    joint_state.position[i] = j_arm(i);
  }

  auto joint_state_index = dofNames_count;
  for (const auto& name : removeJointNames_) {
    joint_state.name[joint_state_index] = name;
    joint_state.position[joint_state_index] = 0.0;
    joint_state_index++;
  }
  jointPublisher_->publish(joint_state);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MobileManipulatorDummyVisualization::publishTargetTrajectories(
    const rclcpp::Time& timeStamp,
    const TargetTrajectories& targetTrajectories) {
  // publish command transform
  const Eigen::Vector3d eeDesiredPosition =
      targetTrajectories.stateTrajectory.back().head(3);
  Eigen::Quaterniond eeDesiredOrientation;
  eeDesiredOrientation.coeffs() =
      targetTrajectories.stateTrajectory.back().tail(4);
  geometry_msgs::msg::TransformStamped command_tf;
  command_tf.header.stamp = timeStamp;
  command_tf.header.frame_id = "world";
  command_tf.child_frame_id = "command";
  command_tf.transform.translation =
      ros_msg_helpers::getVectorMsg(eeDesiredPosition);
  command_tf.transform.rotation =
      ros_msg_helpers::getOrientationMsg(eeDesiredOrientation);
  tfBroadcaster_.sendTransform(command_tf);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MobileManipulatorDummyVisualization::publishOptimizedTrajectory(
    const rclcpp::Time& timeStamp, const PrimalSolution& policy) {
  const scalar_t TRAJECTORYLINEWIDTH = 0.005;
  const std::array<scalar_t, 3> red{0.6350, 0.0780, 0.1840};
  const std::array<scalar_t, 3> blue{0, 0.4470, 0.7410};
  const auto& mpcStateTrajectory = policy.stateTrajectory_;

  visualization_msgs::msg::MarkerArray markerArray;

  // Base trajectory
  std::vector<geometry_msgs::msg::Point> baseTrajectory;
  baseTrajectory.reserve(mpcStateTrajectory.size());
  geometry_msgs::msg::PoseArray poseArray;
  poseArray.poses.reserve(mpcStateTrajectory.size());

  // End effector trajectory
  const auto& model = pinocchioInterface_.getModel();
  auto& data = pinocchioInterface_.getData();

  std::vector<geometry_msgs::msg::Point> endEffectorTrajectory;
  endEffectorTrajectory.reserve(mpcStateTrajectory.size());
  std::for_each(mpcStateTrajectory.begin(), mpcStateTrajectory.end(),
                [&](const Eigen::VectorXd& state) {
                  pinocchio::forwardKinematics(model, data, state);
                  pinocchio::updateFramePlacements(model, data);
                  const auto eeIndex = model.getBodyId(modelInfo_.eeFrame);
                  const vector_t eePosition = data.oMf[eeIndex].translation();
                  endEffectorTrajectory.push_back(
                      ros_msg_helpers::getPointMsg(eePosition));
                });

  markerArray.markers.emplace_back(ros_msg_helpers::getLineMsg(
      std::move(endEffectorTrajectory), blue, TRAJECTORYLINEWIDTH));
  markerArray.markers.back().ns = "EE Trajectory";

  // Extract base pose from state
  std::for_each(mpcStateTrajectory.begin(), mpcStateTrajectory.end(),
                [&](const vector_t& state) {
                  // extract from observation
                  const auto r_world_base = getBasePosition(state, modelInfo_);
                  const Eigen::Quaternion<scalar_t> q_world_base =
                      getBaseOrientation(state, modelInfo_);

                  // convert to ros message
                  geometry_msgs::msg::Pose pose;
                  pose.position = ros_msg_helpers::getPointMsg(r_world_base);
                  pose.orientation =
                      ros_msg_helpers::getOrientationMsg(q_world_base);
                  baseTrajectory.push_back(pose.position);
                  poseArray.poses.push_back(std::move(pose));
                });

  markerArray.markers.emplace_back(ros_msg_helpers::getLineMsg(
      std::move(baseTrajectory), red, TRAJECTORYLINEWIDTH));
  markerArray.markers.back().ns = "Base Trajectory";

  assignHeader(markerArray.markers.begin(), markerArray.markers.end(),
               ros_msg_helpers::getHeaderMsg("world", timeStamp));
  assignIncreasingId(markerArray.markers.begin(), markerArray.markers.end());
  poseArray.header = ros_msg_helpers::getHeaderMsg("world", timeStamp);

  stateOptimizedPublisher_->publish(markerArray);
  stateOptimizedPosePublisher_->publish(poseArray);
}

}  // namespace mobile_manipulator
}  // namespace ocs2
