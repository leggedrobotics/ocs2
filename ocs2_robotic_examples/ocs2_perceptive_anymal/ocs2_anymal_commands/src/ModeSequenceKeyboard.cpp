//
// Created by rgrandia on 18.03.20.
//

#include "ocs2_anymal_commands/ModeSequenceKeyboard.h"

#include <ocs2_core/misc/CommandLine.h>
#include <ocs2_core/misc/LoadData.h>

#include <ocs2_msgs/msg/mode_schedule.hpp>

namespace switched_model {

ModeSequenceKeyboard::ModeSequenceKeyboard(const rclcpp::Node::SharedPtr& node,
                                           const std::string& gaitFile,
                                           const std::string& robotName,
                                           bool verbose) {
  RCLCPP_INFO_STREAM(node->get_logger(),
                     robotName + "_mpc_mode_schedule node is setting up ...");
  ocs2::loadData::loadStdVector(gaitFile, "list", gaitList_, verbose);

  modeSequenceTemplatePublisher_ =
      node->create_publisher<ocs2_msgs::msg::ModeSchedule>(
          robotName + "_mpc_mode_schedule", 1);

  gaitMap_.clear();
  for (const auto& gaitName : gaitList_) {
    gaitMap_.insert(
        {gaitName, loadModeSequenceTemplate(gaitFile, gaitName, verbose)});
  }
  RCLCPP_INFO_STREAM(node->get_logger(),
                     robotName + "_mpc_mode_schedule command node is ready.");
}

void ModeSequenceKeyboard::getKeyboardCommand() {
  const std::string commadMsg =
      "Enter the desired gait, for the list of available gait enter \"list\"";
  std::cout << commadMsg << ": ";

  auto shouldTerminate = []() { return !rclcpp::ok(); };
  const auto gaitCommand = ocs2::getCommandLineString(shouldTerminate);

  if (gaitCommand.empty()) {
    return;
  }

  if (gaitCommand == "list") {
    printGaitList(gaitList_);
    return;
  }

  try {
    ModeSequenceTemplate ModeSequenceTemplate = gaitMap_.at(gaitCommand);
    publishModeSequenceTemplate(ModeSequenceTemplate);
  } catch (const std::out_of_range& e) {
    std::cout << "Gait \"" << gaitCommand << "\" not found.\n";
    printGaitList(gaitList_);
  }
}

void ModeSequenceKeyboard::printGaitList(
    const std::vector<std::string>& gaitList) const {
  std::cout << "List of available gaits:\n";
  size_t itr = 0;
  for (const auto& s : gaitList) {
    std::cout << "[" << itr++ << "]: " << s << "\n";
  }
  std::cout << std::endl;
}

void ModeSequenceKeyboard::publishModeSequenceTemplate(
    const ModeSequenceTemplate& modeSchedule) {
  modeSequenceTemplatePublisher_->publish(
      createModeSequenceTemplateMsg(modeSchedule));
}

}  // namespace switched_model
