//
// Created by Timon Kaufmann in June 2021
//

#include "ocs2_anymal_commands/MotionCommandController.h"

#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_switched_model_interface/ros_msg_conversions/RosMsgConversions.h>

#include <ocs2_switched_model_msgs/msg/scheduled_gait_sequence.hpp>

#include <stdexcept>

namespace switched_model {

MotionCommandController::MotionCommandController(
    const rclcpp::Node::SharedPtr& node, const std::string& configFile,
    const std::string& controllerName)
    : MotionCommandInterface(configFile),
      node_(node),
      client(
          node->create_client<ocs2_switched_model_msgs::srv::TrajectoryRequest>(
              controllerName + "/trajectory_request")) {}

void MotionCommandController::publishMotion(
    const std::pair<ocs2::TargetTrajectories, Gait>& motion) {
  if (!node_) {
    throw std::runtime_error(
        "[MotionCommandController::publishMotion] Node is not initialized.");
  }
  if (!client) {
    throw std::runtime_error(
        "[MotionCommandController::publishMotion] Service client is not initialized.");
  }

  Gait stance;
  stance.duration = 1.0;
  stance.modeSequence = {15};

  auto srv = std::make_shared<
      ocs2_switched_model_msgs::srv::TrajectoryRequest::Request>();
  srv->trajectory =
      ocs2::ros_msg_conversions::createTargetTrajectoriesMsg(motion.first);
  srv->gait_sequence =
      switched_model::ros_msg_conversions::toMessage({motion.second, stance});

  if (!client->wait_for_service(std::chrono::seconds(5))) {
    throw std::runtime_error(
        "[MotionCommandController::publishMotion] TrajectoryRequest service not available.");
  }

  auto future = client->async_send_request(srv);
  const auto status =
      rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(5));
  if (status != rclcpp::FutureReturnCode::SUCCESS) {
    throw std::runtime_error(
        "[MotionCommandController::publishMotion] TrajectoryRequest service call failed or timed out.");
  }
}

}  // namespace switched_model
