/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

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

#include <ocs2_mobile_manipulator/ManipulatorModelInfo.h>
#include <ocs2_mobile_manipulator/MobileManipulatorInterface.h>
#include <ocs2_ros_interfaces/mrt/DummyObserver.h>
#include <ocs2_self_collision_visualization/GeometryInterfaceVisualization.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/pose_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace ocs2 {
namespace mobile_manipulator {

class MobileManipulatorDummyVisualization final : public DummyObserver {
 public:
  MobileManipulatorDummyVisualization(
      const rclcpp::Node::SharedPtr& node,
      const MobileManipulatorInterface& interface)
      : node_(node),
        pinocchioInterface_(interface.getPinocchioInterface()),
        modelInfo_(interface.getManipulatorModelInfo()),
        tfBroadcaster_(node) {
    launchVisualizerNode();
  }

  ~MobileManipulatorDummyVisualization() override = default;

  void update(const SystemObservation& observation,
              const PrimalSolution& policy,
              const CommandData& command) override;

 private:
  void launchVisualizerNode();

  void publishObservation(const rclcpp::Time& timeStamp,
                          const SystemObservation& observation);
  void publishTargetTrajectories(const rclcpp::Time& timeStamp,
                                 const TargetTrajectories& targetTrajectories);
  void publishOptimizedTrajectory(const rclcpp::Time& timeStamp,
                                  const PrimalSolution& policy);

  rclcpp::Node::SharedPtr node_;
  PinocchioInterface pinocchioInterface_;
  const ManipulatorModelInfo modelInfo_;
  std::vector<std::string> removeJointNames_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointPublisher_;
  tf2_ros::TransformBroadcaster tfBroadcaster_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      stateOptimizedPublisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr
      stateOptimizedPosePublisher_;

  std::unique_ptr<GeometryInterfaceVisualization> geometryVisualization_;
};

}  // namespace mobile_manipulator
}  // namespace ocs2
