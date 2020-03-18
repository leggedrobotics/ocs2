//
// Created by rgrandia on 18.03.20.
//

#include "ocs2_anymal_commands/ModeSequenceKeyboard.h"

#include <algorithm>
#include <thread>
#include <atomic>

#include <ocs2_core/misc/LoadData.h>

#include <ocs2_msgs/mode_schedule.h>

namespace switched_model {

ModeSequenceKeyboard::ModeSequenceKeyboard(ros::NodeHandle nodeHandle, const std::string& gaitFile, const std::string& robotName,
                                           bool verbose) {
  ROS_INFO_STREAM(robotName + "_mpc_mode_schedule node is setting up ...");
  ocs2::loadData::loadStdVector(gaitFile, "list", gaitList_, verbose);

  modeSequenceTemplatePublisher_ = nodeHandle.advertise<ocs2_msgs::mode_schedule>(robotName + "_mpc_mode_schedule", 1, true);

  gaitMap_.clear();
  for (const auto& gaitName : gaitList_) {
    gaitMap_.insert({gaitName, loadModeSequenceTemplate(gaitFile, gaitName, verbose)});
  }
  ROS_INFO_STREAM(robotName + "_mpc_mode_schedule command node is ready.");
}

void ModeSequenceKeyboard::getKeyboardCommand() {
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

std::string ModeSequenceKeyboard::getCommandLine() {
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

void ModeSequenceKeyboard::printGaitList(const std::vector<std::string>& gaitList) const {
  std::cout << "List of available gaits:\n";
  size_t itr = 0;
  for (const auto& s : gaitList) {
    std::cout << "[" << itr++ << "]: " << s << "\n";
  }
  std::cout << std::endl;
}

void ModeSequenceKeyboard::publishModeSequenceTemplate(const ModeSequenceTemplate& modeSchedule) {
  modeSequenceTemplatePublisher_.publish(createModeSequenceTemplateMsg(modeSchedule));
}

}  // namespace switched_model
