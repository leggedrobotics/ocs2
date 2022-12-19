//
// Created by Timon Kaufmann in June 2021
//

#include "ocs2_anymal_commands/MotionCommandController.h"

#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_switched_model_interface/ros_msg_conversions/RosMsgConversions.h>
#include <ocs2_switched_model_msgs/scheduled_gait_sequence.h>
#include <ocs2_switched_model_msgs/trajectory_request.h>

namespace switched_model {

MotionCommandController::MotionCommandController(ros::NodeHandle& nodeHandle, const std::string& configFile,
                                                 const std::string& controllerName)
    : MotionCommandInterface(configFile),
      client(nodeHandle.serviceClient<ocs2_switched_model_msgs::trajectory_request>(controllerName + "/trajectory_request")) {}

void MotionCommandController::publishMotion(const std::pair<ocs2::TargetTrajectories, Gait>& motion) {
  Gait stance;
  stance.duration = 1.0;
  stance.modeSequence = {15};

  ocs2_switched_model_msgs::trajectory_request srv;
  srv.request.trajectory = ocs2::ros_msg_conversions::createTargetTrajectoriesMsg(motion.first);
  srv.request.gaitSequence = switched_model::ros_msg_conversions::toMessage({motion.second, stance});

  client.call(srv);
}

}  // namespace switched_model