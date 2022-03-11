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

void QuadrupedTfPublisher::launchNode(ros::NodeHandle& nodeHandle, const std::string& descriptionName, const std::string& tfPrefix) {
  tfPrefix_ = tfPrefix;

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
  jointPositionsMap_["LF_HAA"] = jointPositions[0];
  jointPositionsMap_["LF_HFE"] = jointPositions[1];
  jointPositionsMap_["LF_KFE"] = jointPositions[2];
  jointPositionsMap_["RF_HAA"] = jointPositions[3];
  jointPositionsMap_["RF_HFE"] = jointPositions[4];
  jointPositionsMap_["RF_KFE"] = jointPositions[5];
  jointPositionsMap_["LH_HAA"] = jointPositions[6];
  jointPositionsMap_["LH_HFE"] = jointPositions[7];
  jointPositionsMap_["LH_KFE"] = jointPositions[8];
  jointPositionsMap_["RH_HAA"] = jointPositions[9];
  jointPositionsMap_["RH_HFE"] = jointPositions[10];
  jointPositionsMap_["RH_KFE"] = jointPositions[11];
}

void QuadrupedTfPublisher::updateBasePose(ros::Time timeStamp, const base_coordinate_t& basePose, const std::string& worldFrame) {
  baseToWorldTransform_.header = ocs2::getHeaderMsg(worldFrame, timeStamp);
  baseToWorldTransform_.child_frame_id = tfPrefix_ + "/base";

  const Eigen::Quaternion<scalar_t> q_world_base = quaternionBaseToOrigin<scalar_t>(getOrientation(basePose));
  baseToWorldTransform_.transform.rotation = ocs2::getOrientationMsg(q_world_base);
  baseToWorldTransform_.transform.translation = ocs2::getVectorMsg(getPositionInOrigin(basePose));
}

}  // namespace switched_model