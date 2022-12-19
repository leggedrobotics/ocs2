//
// Created by rgrandia on 10.03.22.
//

#include "ocs2_quadruped_interface/QuadrupedTfPublisher.h"

#include <ocs2_ros_interfaces/visualization/VisualizationHelpers.h>

// Additional messages not in the helpers file
#include <geometry_msgs/PoseArray.h>

// URDF stuff
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>

#include <ocs2_switched_model_interface/core/Rotations.h>

namespace switched_model {

void QuadrupedTfPublisher::launchNode(ros::NodeHandle& nodeHandle, const std::string& descriptionName, std::vector<std::string> jointNames,
                                      std::string baseName, const std::string& tfPrefix) {
  tfPrefix_ = tfPrefix;
  jointNames_ = std::move(jointNames);
  baseName_ = std::move(baseName);

  // Load URDF model
  urdf::Model urdfModel;
  if (!urdfModel.initParam(descriptionName)) {
    std::cerr << "[QuadrupedTfPublisher] Could not read URDF from: \"" << descriptionName << "\"" << std::endl;
  } else {
    KDL::Tree kdlTree;
    kdl_parser::treeFromUrdfModel(urdfModel, kdlTree);

    robotStatePublisherPtr_.reset(new robot_state_publisher::RobotStatePublisher(kdlTree));
    robotStatePublisherPtr_->publishFixedTransforms(tfPrefix_, true);
  }
}

void QuadrupedTfPublisher::publish(ros::Time timeStamp, const vector_t& state, const std::string& worldFrame) {
  publish(timeStamp, getBasePose(state), getJointPositions(state), worldFrame);
}

void QuadrupedTfPublisher::publish(ros::Time timeStamp, const base_coordinate_t& basePose, const joint_coordinate_t& jointPositions,
                                   const std::string& worldFrame) {
  if (robotStatePublisherPtr_ != nullptr && lastTimeStamp_ != timeStamp) {
    // Joint positions
    updateJointPositions(jointPositions);
    robotStatePublisherPtr_->publishTransforms(jointPositionsMap_, timeStamp, tfPrefix_);

    // Base positions
    updateBasePose(timeStamp, basePose, worldFrame);
    tfBroadcaster_.sendTransform(baseToWorldTransform_);

    lastTimeStamp_ = timeStamp;
  }
}

void QuadrupedTfPublisher::updateJointPositions(const joint_coordinate_t& jointPositions) {
  jointPositionsMap_[jointNames_[0]] = jointPositions[0];
  jointPositionsMap_[jointNames_[1]] = jointPositions[1];
  jointPositionsMap_[jointNames_[2]] = jointPositions[2];
  jointPositionsMap_[jointNames_[3]] = jointPositions[3];
  jointPositionsMap_[jointNames_[4]] = jointPositions[4];
  jointPositionsMap_[jointNames_[5]] = jointPositions[5];
  jointPositionsMap_[jointNames_[6]] = jointPositions[6];
  jointPositionsMap_[jointNames_[7]] = jointPositions[7];
  jointPositionsMap_[jointNames_[8]] = jointPositions[8];
  jointPositionsMap_[jointNames_[9]] = jointPositions[9];
  jointPositionsMap_[jointNames_[10]] = jointPositions[10];
  jointPositionsMap_[jointNames_[11]] = jointPositions[11];
}

void QuadrupedTfPublisher::updateBasePose(ros::Time timeStamp, const base_coordinate_t& basePose, const std::string& worldFrame) {
  baseToWorldTransform_.header = ocs2::getHeaderMsg(worldFrame, timeStamp);
  baseToWorldTransform_.child_frame_id = tfPrefix_ + "/" + baseName_;

  const Eigen::Quaternion<scalar_t> q_world_base = quaternionBaseToOrigin<scalar_t>(getOrientation(basePose));
  baseToWorldTransform_.transform.rotation = ocs2::getOrientationMsg(q_world_base);
  baseToWorldTransform_.transform.translation = ocs2::getVectorMsg(getPositionInOrigin(basePose));
}

}  // namespace switched_model