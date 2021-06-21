#include <ocs2_legged_robot_example/command/LeggedRobotModeSequenceKeyboard.h>

#include <ocs2_core/misc/LoadData.h>
#include <ocs2_msgs/mode_schedule.h>
#include <algorithm>
#include <atomic>
#include <thread>

namespace ocs2 {
namespace legged_robot {

LeggedRobotModeSequenceKeyboard::LeggedRobotModeSequenceKeyboard(ros::NodeHandle nodeHandle, const std::string& gaitFile,
                                                                 const std::string& robotName, bool verbose) {
  ROS_INFO_STREAM(robotName + "_mpc_mode_schedule node is setting up ...");
  ocs2::loadData::loadStdVector(gaitFile, "list", gaitList_, verbose);

  modeSequenceTemplatePublisher_ = nodeHandle.advertise<ocs2_msgs::mode_schedule>(robotName + "_mpc_mode_schedule", 1, true);

  gaitMap_.clear();
  for (const auto& gaitName : gaitList_) {
    gaitMap_.insert({gaitName, loadModeSequenceTemplate(gaitFile, gaitName, verbose)});
  }
  ROS_INFO_STREAM(robotName + "_mpc_mode_schedule command node is ready.");
}

void LeggedRobotModeSequenceKeyboard::getKeyboardCommand() {
  const std::string commadMsg = "Enter the desired gait, for the list of available gait enter \"list\"";
  std::cout << commadMsg << ": ";

  const auto gaitCommand = getCommandLine();

  if (gaitCommand.empty()) {
    return;
  }

  if (gaitCommand == "list") {
    printGaitList(gaitList_);
    return;
  }

  try {
    ModeSequenceTemplate modeSequenceTemplate = gaitMap_.at(gaitCommand);
    publishModeSequenceTemplate(modeSequenceTemplate);
  } catch (const std::out_of_range& e) {
    std::cout << "Gait \"" << gaitCommand << "\" not found.\n";
    printGaitList(gaitList_);
  }
}

std::string LeggedRobotModeSequenceKeyboard::getCommandLine() {
  std::vector<std::string> gaitCommand;

  // Set up a thread to read user inputs
  std::string line;
  std::atomic_bool lineRead{false};
  std::thread thr([&line, &lineRead]() {
    lineRead = false;
    getline(std::cin, line);
    lineRead = true;
  });

  // wait till line is read or terminate if ROS is gone.
  ros::WallRate rate(30);
  while (!lineRead) {
    if (!ros::ok() || !ros::master::check()) {
      std::terminate();  // Need to terminate thread that is still waiting for input
    }

    ros::spinOnce();

    rate.sleep();
  }
  thr.join();

  std::istringstream stream(line);
  std::string in;
  while (stream >> in) {
    gaitCommand.push_back(in);
  }

  if (gaitCommand.size() != 1) {
    std::cout << "WARNING: The command should be a single word." << std::endl;
    gaitCommand.resize(1);
    gaitCommand.front().clear();
  }

  // lower case transform
  std::transform(gaitCommand.front().begin(), gaitCommand.front().end(), gaitCommand.front().begin(), ::tolower);

  return gaitCommand.front();
}

void LeggedRobotModeSequenceKeyboard::printGaitList(const std::vector<std::string>& gaitList) const {
  std::cout << "List of available gaits:\n";
  size_t itr = 0;
  for (const auto& s : gaitList) {
    std::cout << "[" << itr++ << "]: " << s << "\n";
  }
  std::cout << std::endl;
}

void LeggedRobotModeSequenceKeyboard::publishModeSequenceTemplate(const ModeSequenceTemplate& modeSchedule) {
  modeSequenceTemplatePublisher_.publish(createModeSequenceTemplateMsg(modeSchedule));
}

}  // namespace legged_robot
}  // end of namespace ocs2
