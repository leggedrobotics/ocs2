#include <ocs2_legged_robot_example/command/LeggedRobotModeSequenceKeyboard.h>

using namespace ocs2;
using namespace legged_robot;

int main(int argc, char* argv[]) {
  std::vector<std::string> programArgs{};
  ::ros::removeROSArgs(argc, argv, programArgs);
  if (programArgs.size() < 3) {
    throw std::runtime_error("No robot name or target command file specified. Aborting.");
  }
  const std::string robotName(programArgs[1]);
  const std::string gaitFile(programArgs[2]);
  std::cerr << "Loading gait file: " << gaitFile << std::endl;

  ros::init(argc, argv, robotName + "_mpc_mode_schedule");
  ros::NodeHandle nodeHandle;

  LeggedRobotModeSequenceKeyboard modeSequenceCommand(nodeHandle, gaitFile, robotName, true);

  while (ros::ok() && ros::master::check()) {
    modeSequenceCommand.getKeyboardCommand();
  }

  // Successful exit
  return 0;
}
