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

#pragma once

#include <ocs2_legged_robot/gait/ModeSequenceTemplate.h>

#include <ocs2_msgs/msg/mode_schedule.hpp>
#include <std_msgs/msg/bool.hpp>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace ocs2 {
namespace legged_robot {

/** This class implements mode_sequence communication using ROS. */
class GaitKeyboardPublisher {
 public:
  GaitKeyboardPublisher(const rclcpp::Node::SharedPtr& node,
                        const std::string& gaitFile,
                        const std::string& robotName, bool verbose = false);

  /** Prints the command line interface and responds to user input. Function
   * returns after one user input. */
  void getKeyboardCommand();

 private:
  /** Prints the list of available gaits. */
  void printGaitList(const std::vector<std::string>& gaitList) const;

  std::vector<std::string> gaitList_;
  std::map<std::string, ModeSequenceTemplate> gaitMap_;

  rclcpp::Publisher<ocs2_msgs::msg::ModeSchedule>::SharedPtr
      modeSequenceTemplatePublisher_;
};

}  // namespace legged_robot
}  // end of namespace ocs2
