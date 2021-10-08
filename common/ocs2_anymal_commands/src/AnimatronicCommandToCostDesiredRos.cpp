//
// Created by Timon Kaufmann in June 2021
//

#include "ocs2_anymal_commands/AnimatronicCommandToCostDesiredRos.h"

#include <ros/package.h>

#include <ocs2_core/misc/CommandLine.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_switched_model_msgs/scheduled_gait_sequence.h>

#include "ocs2_anymal_commands/LoadMotions.h"
#include "ocs2_anymal_commands/TerrainAdaptation.h"

namespace switched_model {

AnimatronicCommandToCostDesiredRos::AnimatronicCommandToCostDesiredRos(ros::NodeHandle& nodeHandle, const std::string& configFile,
                                                                       const std::string& robotName)
    : localTerrain_() {
  std::vector<std::string> motionList;
  bool verbose = false;
  ocs2::loadData::loadStdVector(configFile, "motionList", motionList, verbose);

  const std::string motionFilesPath = ros::package::getPath("ocs2_anymal_commands") + "/config/motions/";
  for (const auto& motionName : motionList) {
    const auto csvData = readCsv(motionFilesPath + motionName + ".txt");
    motionData_.insert({motionName, readMotion(csvData)});
  }
  printAnimationList();

  // Setup ROS communication
  // Publishers
  targetTrajectoryPublisher_ = nodeHandle.advertise<ocs2_msgs::mpc_target_trajectories>(robotName + "_mpc_target", 1, false);
  gaitSequencePublisher_ =
      nodeHandle.advertise<ocs2_switched_model_msgs::scheduled_gait_sequence>(robotName + "_mpc_gait_schedule", 1, true);

  // Subsribers
  observationSubscriber_ =
      nodeHandle.subscribe(robotName + "_mpc_observation", 1, &AnimatronicCommandToCostDesiredRos::observationCallback, this);
  terrainSubscriber_ = nodeHandle.subscribe("/ocs2_anymal/localTerrain", 1, &AnimatronicCommandToCostDesiredRos::terrainCallback, this);
}

void AnimatronicCommandToCostDesiredRos::observationCallback(const ocs2_msgs::mpc_observation::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(observationMutex_);
  observationPtr_.reset(new ocs2::SystemObservation(ocs2::ros_msg_conversions::readObservationMsg(*msg)));
}

void AnimatronicCommandToCostDesiredRos::getKeyboardCommand() {
  const std::string commandMsg = "Enter the desired motion, for the list of available motions enter \"list\"";
  std::cout << commandMsg << ": ";

  auto shouldTerminate = []() { return !ros::ok() || !ros::master::check(); };
  const std::string motionCommand = ocs2::getCommandLineString(shouldTerminate);

  if (motionCommand.empty()) {
    return;
  }

  if (motionCommand == "list") {
    printAnimationList();
    return;
  }

  try {
    const auto& motion = motionData_.at(motionCommand);
    std::cout << "Executing \"" << motionCommand << "\" \n";
    publishMotion(motion);
  } catch (const std::out_of_range& e) {
    std::cout << "Motion \"" << motionCommand << "\" not found.\n";
    printAnimationList();
  }
}

void AnimatronicCommandToCostDesiredRos::terrainCallback(const visualization_msgs::Marker::ConstPtr& msg) {
  Eigen::Quaterniond orientationTerrainToWorld{msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y,
                                               msg->pose.orientation.z};

  std::lock_guard<std::mutex> lock(terrainMutex_);
  localTerrain_.positionInWorld = {msg->pose.position.x, msg->pose.position.y, msg->pose.position.z};
  localTerrain_.orientationWorldToTerrain = orientationTerrainToWorld.toRotationMatrix().transpose();
}

void AnimatronicCommandToCostDesiredRos::printAnimationList() const {
  std::cout << "\nList of available motions:\n";
  for (const auto& s : motionData_) {
    std::cout << "  * " << s.first << "\n";
  }
  std::cout << std::endl;
}

void AnimatronicCommandToCostDesiredRos::publishMotion(const std::pair<ocs2::TargetTrajectories, Gait>& motion) {
  ros::spinOnce();  // Trigger callback
  ocs2::SystemObservation observation;
  {
    std::lock_guard<std::mutex> lock(observationMutex_);
    if (observationPtr_) {
      observation = *observationPtr_;
    } else {
      std::cout << "No observation is received from the MPC node. Make sure the MPC node is running!"
                << "\n";
      return;
    }
  }
  const scalar_t startTime = observation.time + 1.0;

  Gait stance;
  stance.duration = 1.0;
  stance.modeSequence = {15};

  const auto gaitMessage = switched_model::ros_msg_conversions::toMessage(startTime, {motion.second, stance});
  auto mpcTargetTrajectoriesMsg = ocs2::ros_msg_conversions::createTargetTrajectoriesMsg(motion.first);
  for (auto& t : mpcTargetTrajectoriesMsg.timeTrajectory) {
    t += startTime;
  }

  gaitSequencePublisher_.publish(gaitMessage);
  targetTrajectoryPublisher_.publish(mpcTargetTrajectoriesMsg);
}

}  // namespace switched_model