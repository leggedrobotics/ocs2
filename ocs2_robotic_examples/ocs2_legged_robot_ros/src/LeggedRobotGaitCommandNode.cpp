/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <rclcpp/rclcpp.hpp>

#include "ocs2_legged_robot_ros/gait/GaitKeyboardPublisher.h"

using namespace ocs2;
using namespace legged_robot;
static auto LOGGER = rclcpp::get_logger("LeggedRobotGaitCommandNode");

auto declareAndGetStringParam = [] (rclcpp::Node::SharedPtr &node, const std::string &param, std::string &param_value) {
  if (!node->has_parameter(param))
  {
    node->declare_parameter(param, std::string(""));
  }
  rclcpp::Parameter parameter;
  node->get_parameter(param, parameter);
  param_value = parameter.as_string();
  RCLCPP_INFO_STREAM(LOGGER, "Retrieved parameter " << param << " with value " << param_value);
};

int main(int argc, char* argv[]) {
  const std::string robotName = "legged_robot";

  // Initialize ros node
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr nodeHandle = rclcpp::Node::make_shared(robotName + "_mpc_mode_schedule");

  // Get node parameters
  std::string gaitCommandFile;
  declareAndGetStringParam(nodeHandle, "gait_command_file", gaitCommandFile);
  std::cerr << "Loading gait file: " << gaitCommandFile << std::endl;

  GaitKeyboardPublisher gaitCommand(nodeHandle, gaitCommandFile, robotName, true);

  while (rclcpp::ok()) {
    gaitCommand.getKeyboardCommand();
  }

  // Successful exit
  return 0;
}
