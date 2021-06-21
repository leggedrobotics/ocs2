#include <ros/package.h>

#include <ocs2_legged_robot_example/command/LeggedRobotModeSequenceKeyboard.h>
#include <ocs2_legged_robot_example/common/definitions.h>

using namespace ocs2;
using namespace legged_robot;

int main(int argc, char* argv[]) {
  std::string gaitFile = ros::package::getPath("ocs2_legged_robot_example") + "/config/command/gait.info";
  std::cerr << "Loading gait file: " << gaitFile << std::endl;

  ros::init(argc, argv, ROBOT_NAME_ + "_mpc_mode_schedule");
  ros::NodeHandle nodeHandle;

  LeggedRobotModeSequenceKeyboard modeSequenceCommand(nodeHandle, gaitFile, ROBOT_NAME_, true);

  while (ros::ok() && ros::master::check()) {
    modeSequenceCommand.getKeyboardCommand();
  }

  // Successful exit
  return 0;
}
