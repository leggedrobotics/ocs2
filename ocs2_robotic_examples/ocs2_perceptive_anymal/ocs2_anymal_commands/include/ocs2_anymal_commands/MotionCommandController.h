//
// Created by rgrandia on 14.10.21.
//

#pragma once

#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_switched_model_interface/terrain/TerrainPlane.h>

#include <ocs2_msgs/msg/mpc_observation.hpp>
#include <ocs2_switched_model_msgs/srv/trajectory_request.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "ocs2_anymal_commands/MotionCommandInterface.h"

namespace switched_model {

class MotionCommandController : public MotionCommandInterface {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  MotionCommandController(const rclcpp::Node::SharedPtr& node,
                          const std::string& configFile,
                          const std::string& controllerName);
  ~MotionCommandController() override = default;

  void publishMotion(
      const std::pair<ocs2::TargetTrajectories, Gait>& motion) override;

 private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<ocs2_switched_model_msgs::srv::TrajectoryRequest>::SharedPtr
      client;
};

}  // namespace switched_model