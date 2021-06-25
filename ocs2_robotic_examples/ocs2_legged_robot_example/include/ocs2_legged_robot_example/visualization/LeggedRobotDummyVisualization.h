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

// ocs2_legged_robot_example
#include <ocs2_legged_robot_example/LeggedRobotInterface.h>

// ocs2
#include <ocs2_ros_interfaces/mrt/DummyObserver.h>

// visualization
#include <robot_state_publisher/robot_state_publisher.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <urdf/model.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <kdl_parser/kdl_parser.hpp>

namespace ocs2 {
namespace legged_robot {

class LeggedRobotDummyVisualization final : public DummyObserver {
 public:
  explicit LeggedRobotDummyVisualization(ros::NodeHandle& nodeHandle, const PinocchioInterface& pinocchioInterface)
      : pinocchioInterface_(pinocchioInterface) {
    launchVisualizerNode(nodeHandle);
  }

  ~LeggedRobotDummyVisualization() override = default;

  void update(const ocs2::SystemObservation& observation, const ocs2::PrimalSolution& policy, const ocs2::CommandData& command) override;

 private:
  void publishOptimizedStateTrajectory(ros::Time timeStamp, const vector_array_t& mpcStateTrajectory);
  void publishBaseTransform(ros::Time timeStamp, const ocs2::SystemObservation& observation) const;
  void launchVisualizerNode(ros::NodeHandle& nodeHandle);

  std::unique_ptr<tf::TransformBroadcaster> tfBroadcasterPtr_;
  std::unique_ptr<robot_state_publisher::RobotStatePublisher> robotStatePublisherPtr_;
  std::map<std::string, scalar_t> jointPositions_;
  PinocchioInterface pinocchioInterface_;

  ros::Publisher policyDelayPublisher_;
  ros::Publisher armTorquesPublished_;
  ros::Publisher stateOptimizedPublisher_;
  scalar_t policyDelayFromObservation_ = 0.0;
  scalar_t elapsedTime_ = 0.0;
  scalar_t dt_ = 0.0025;
};

}  // namespace legged_robot
}  // namespace ocs2
