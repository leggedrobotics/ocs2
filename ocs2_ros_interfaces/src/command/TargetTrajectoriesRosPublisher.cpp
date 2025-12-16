/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

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

#include "ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h"

// MPC messages
#include <ocs2_msgs/msg/mpc_target_trajectories.hpp>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
TargetTrajectoriesRosPublisher::TargetTrajectoriesRosPublisher(rclcpp::Node::SharedPtr node, const std::string& topicPrefix) : node_(std::move(node)) {
  
  publisher_ = node_->create_publisher<ocs2_msgs::msg::MpcTargetTrajectories>(topicPrefix + "_mpc_target", 1); 
  RCLCPP_INFO(node_->get_logger(), "The TargetTrajectories is publishing on %s_mpc_target topic.", topicPrefix.c_str());
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
TargetTrajectoriesRosPublisher::~TargetTrajectoriesRosPublisher() {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void TargetTrajectoriesRosPublisher::publishTargetTrajectories(const TargetTrajectories& targetTrajectories) {
  const auto mpcTargetTrajectoriesMsg = ros_msg_conversions::createTargetTrajectoriesMsg(targetTrajectories);
  publisher_->publish(mpcTargetTrajectoriesMsg);
}

}  // namespace ocs2
