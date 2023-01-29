#include <string>

#include <ocs2_core/Types.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesKeyboardPublisher.h>

#include "ocs2_anymal_commands/PoseCommandToCostDesiredRos.h"

int main(int argc, char* argv[]) {
  const std::string robotName = "anymal";

  const std::string filename = [&] {
    std::vector<std::string> programArgs{};
    ros::removeROSArgs(argc, argv, programArgs);
    if (programArgs.size() <= 1) {
      throw std::runtime_error("No task file specified. Aborting.");
    }
    return programArgs[1];
  }();

  // ros node handle
  ::ros::init(argc, argv, robotName + "_mpc_pose_command_node");
  ::ros::NodeHandle nodeHandle;

  // PoseCommand To TargetTrajectories
  switched_model::PoseCommandToCostDesiredRos targetPoseCommand(nodeHandle, filename);
  auto commandLineToTargetTrajectoriesFun = [&](const ocs2::vector_t& commandLineTarget, const ocs2::SystemObservation& observation) {
    return targetPoseCommand.commandLineToTargetTrajectories(commandLineTarget, observation);
  };

  // goalPose: [deltaX, deltaY, deltaZ, Roll, Pitch, deltaYaw]
  const ocs2::scalar_array_t relativeBaseLimit{10.0, 10.0, 0.2, 45.0, 45.0, 360.0};
  ocs2::TargetTrajectoriesKeyboardPublisher targetTrajectoriesKeyboardPublisher(
      nodeHandle, robotName, relativeBaseLimit, commandLineToTargetTrajectoriesFun);

  const std::string commandMsg = "Enter XYZ displacement and RollPitchYaw for the robot, separated by spaces";
  targetTrajectoriesKeyboardPublisher.publishKeyboardCommand(commandMsg);

  // Successful exit
  return 0;
}
