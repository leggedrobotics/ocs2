//
// Created by Timon Kaufmann in June 2021
//

#include "ocs2_anymal_commands/MotionCommandController.h"

#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_switched_model_interface/ros_msg_conversions/RosMsgConversions.h>

#include <ocs2_switched_model_msgs/msg/scheduled_gait_sequence.hpp>

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
  Gait stance;
  stance.duration = 1.0;
  stance.modeSequence = {15};

  auto srv = std::make_shared<
      ocs2_switched_model_msgs::srv::TrajectoryRequest::Request>();
  srv->trajectory =
      ocs2::ros_msg_conversions::createTargetTrajectoriesMsg(motion.first);
  srv->gait_sequence =
      switched_model::ros_msg_conversions::toMessage({motion.second, stance});

  client->async_send_request(srv);
}

}  // namespace switched_model