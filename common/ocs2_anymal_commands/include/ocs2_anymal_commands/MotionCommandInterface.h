//
// Created by Timon Kaufmann in June 2021
//

#pragma once

#include <mutex>
#include <string>
#include <vector>
#include <unordered_map>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <ocs2_msgs/mpc_observation.h>
#include <ocs2_msgs/mpc_state.h>
#include <ocs2_switched_model_interface/ros_msg_conversions/RosMsgConversions.h>

#include <ocs2_switched_model_interface/core/SwitchedModel.h>
#include <ocs2_switched_model_interface/logic/Gait.h>
#include <ocs2_switched_model_interface/logic/ModeSequenceTemplate.h>
#include <ocs2_switched_model_interface/terrain/TerrainPlane.h>

namespace switched_model {

class MotionCommandInterface {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  MotionCommandInterface(ros::NodeHandle& nodeHandle, const std::string& configFile, const std::string& robotName);

  void publishMotion(const std::pair<ocs2::TargetTrajectories, Gait>& motion);

  void getKeyboardCommand();

 private:
  void observationCallback(const ocs2_msgs::mpc_observation::ConstPtr& msg);

  void terrainCallback(const visualization_msgs::Marker::ConstPtr& msg);

  void printAnimationList() const;

  ros::Publisher targetTrajectoryPublisher_;
  ros::Publisher gaitSequencePublisher_;

  ros::Subscriber observationSubscriber_;
  std::mutex observationMutex_;
  std::unique_ptr<ocs2::SystemObservation> observationPtr_;

  ros::Subscriber terrainSubscriber_;
  std::mutex terrainMutex_;
  TerrainPlane localTerrain_;

  std::unordered_map<std::string, std::pair<ocs2::TargetTrajectories, Gait>> motionData_;
};

}  // namespace switched_model