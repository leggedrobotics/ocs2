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

#include "ocs2_double_integrator_ros/DoubleIntegratorDummyVisualization.h"

namespace ocs2 {
namespace double_integrator {

DoubleIntegratorDummyVisualization::DoubleIntegratorDummyVisualization(
    const rclcpp::Node::SharedPtr& node)
    : node_(node),
      jointPublisher_(node_->create_publisher<sensor_msgs::msg::JointState>(
          "joint_states", 1)) {}

void DoubleIntegratorDummyVisualization::update(
    const SystemObservation& observation, const PrimalSolution& policy,
    const CommandData& command) {
  const auto& targetTrajectories = command.mpcTargetTrajectories_;
  sensor_msgs::msg::JointState joint_state;
  joint_state.header.stamp = node_->get_clock()->now();
  joint_state.name.resize(2);
  joint_state.position.resize(2);
  joint_state.name[0] = "slider_to_cart";
  joint_state.position[0] = observation.state(0);
  joint_state.name[1] = "slider_to_target";
  joint_state.position[1] = targetTrajectories.stateTrajectory[0](0);

  jointPublisher_->publish(joint_state);
}

}  // namespace double_integrator
}  // namespace ocs2
