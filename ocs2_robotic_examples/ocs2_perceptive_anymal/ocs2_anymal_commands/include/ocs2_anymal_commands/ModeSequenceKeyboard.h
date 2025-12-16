/*
 * modeSequence_Keyboard_Quadruped.h
 *
 *  Created on: Oct 11, 2018
 *      Author: farbod
 */

#pragma once

#include <ocs2_switched_model_interface/core/SwitchedModel.h>
#include <ocs2_switched_model_interface/logic/ModeSequenceTemplate.h>

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace switched_model {

/** This class implements modeSequence communication using ROS. */
class ModeSequenceKeyboard {
 public:
  ModeSequenceKeyboard(const rclcpp::Node::SharedPtr& node,
                       const std::string& gaitFile,
                       const std::string& robotName, bool verbose = false);

  /** Prints the command line interface and responds to user input. Function
   * returns after one user input. */
  void getKeyboardCommand();

 private:
  /** Prints the list of available gaits. */
  void printGaitList(const std::vector<std::string>& gaitList) const;

  /** Publishes the mode sequence template. */
  void publishModeSequenceTemplate(const ModeSequenceTemplate& modeSchedule);

  std::vector<std::string> gaitList_;
  std::map<std::string, ModeSequenceTemplate> gaitMap_;

  rclcpp::Publisher<ocs2_msgs::msg::ModeSchedule>::SharedPtr
      modeSequenceTemplatePublisher_;
};

}  // end of namespace switched_model
