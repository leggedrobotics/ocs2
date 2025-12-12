//
// Created by rgrandia on 14.10.21.
//

#pragma once

#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_switched_model_interface/terrain/TerrainPlane.h>

#include <ocs2_msgs/msg/mpc_observation.hpp>
#include <ocs2_msgs/msg/mpc_target_trajectories.hpp>
#include <ocs2_switched_model_msgs/msg/scheduled_gait_sequence.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "ocs2_anymal_commands/MotionCommandInterface.h"
#include "rclcpp/rclcpp.hpp"

namespace switched_model {

class MotionCommandDummy : public MotionCommandInterface {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  MotionCommandDummy(const rclcpp::Node::SharedPtr& node,
                     const std::string& configFile,
                     const std::string& robotName);
  ~MotionCommandDummy() override = default;

  void publishMotion(
      const std::pair<ocs2::TargetTrajectories, Gait>& motion) override;

 private:
  void observationCallback(
      const ocs2_msgs::msg::MpcObservation::ConstSharedPtr& msg);

  void terrainCallback(
      const visualization_msgs::msg::Marker::ConstSharedPtr& msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<ocs2_msgs::msg::MpcTargetTrajectories>::SharedPtr
      targetTrajectoryPublisher_;
  rclcpp::Publisher<ocs2_switched_model_msgs::msg::ScheduledGaitSequence>::
      SharedPtr gaitSequencePublisher_;

  rclcpp::Subscription<ocs2_msgs::msg::MpcObservation>::SharedPtr
      observationSubscriber_;
  std::mutex observationMutex_;
  std::unique_ptr<ocs2::SystemObservation> observationPtr_;

  rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr
      terrainSubscriber_;
  std::mutex terrainMutex_;
  TerrainPlane localTerrain_;
};

}  // namespace switched_model