#include <ros/init.h>
#include <ros/package.h>
#include <ros/ros.h>

#include "ocs2_anymal_raisim/AnymalRaisimDummy.h"

#include <ocs2_anymal_models/AnymalModels.h>
#include <ocs2_anymal_mpc/AnymalInterface.h>
#include <ocs2_mpc/MPC_Settings.h>

int main(int argc, char* argv[]) {
  std::vector<std::string> programArgs{};
  ::ros::removeROSArgs(argc, argv, programArgs);
  if (programArgs.size() < 3) {
    throw std::runtime_error("No robot name and config folder specified. Aborting.");
  }
  const std::string robotName(programArgs[1]);
  const std::string configName(programArgs[2]);

  // Initialize ros node
  ros::init(argc, argv, "anymal_mrt");
  ros::NodeHandle nodeHandle;

  // Read URDF
  const std::string urdf_param = "/anymal_raisim_urdf";
  std::string urdf;
  if (!ros::param::get(urdf_param, urdf)) {
    throw ros::Exception("Error reading ros parameter: " + urdf_param);
  }

  auto anymalInterface = anymal::getAnymalInterface(anymal::stringToAnymalModel(robotName), anymal::getConfigFolder(configName));
  auto wholebodyDynamics = anymal::getWholebodyDynamics(anymal::stringToAnymalModel(robotName));

  // load settings
  const auto mpcSettings = ocs2::mpc::loadSettings(anymal::getTaskFilePath(configName));
  ocs2::RaisimRolloutSettings raisimRolloutSettings;
  raisimRolloutSettings.loadSettings(ros::package::getPath("ocs2_anymal_raisim") + "/config/raisim_rollout.info", "rollout");

  anymal::runAnymalRaisimDummy(nodeHandle, std::move(anymalInterface), *wholebodyDynamics, urdf, mpcSettings.mrtDesiredFrequency_,
                               mpcSettings.mpcDesiredFrequency_, raisimRolloutSettings);

  return 0;
}
