
#include <ros/package.h>

#include "ocs2_anymal_commands/MotionCommandController.h"
#include "ocs2_anymal_commands/MotionCommandDummy.h"

int main(int argc, char* argv[]) {
  const std::string robotName = "anymal";
  std::string motionFile = ros::package::getPath("ocs2_anymal_commands") + "/config/motions.info";
  std::cerr << "Loading motion file: " << motionFile << std::endl;

  const std::string controllerName = [&] {
    std::vector<std::string> programArgs{};
    ros::removeROSArgs(argc, argv, programArgs);
    if (programArgs.size() <= 1) {
      throw std::runtime_error("No operation mode specified. Aborting.");
    }
    return programArgs[1];
  }();

  ros::init(argc, argv, robotName + "_mpc_motion_command");
  ros::NodeHandle nodeHandle;

  std::unique_ptr<switched_model::MotionCommandInterface> motionCommandInterface;
  if (controllerName == "dummy") {
    motionCommandInterface.reset(new switched_model::MotionCommandDummy(nodeHandle, motionFile, robotName));
  } else {
    motionCommandInterface.reset(new switched_model::MotionCommandController(nodeHandle, motionFile, controllerName));
  }

  ros::Rate rate(10);
  while (ros::ok() && ros::master::check()) {
    motionCommandInterface->getKeyboardCommand();
    rate.sleep();
  }

  // Successful exit
  return 0;
}
