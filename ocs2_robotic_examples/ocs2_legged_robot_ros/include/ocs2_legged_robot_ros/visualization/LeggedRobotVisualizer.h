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
#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_core/Types.h>
#include <ocs2_legged_robot/common/Types.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_ros_interfaces/mrt/DummyObserver.h>
#include <ocs2_ros_interfaces/visualization/VisualizationColors.h>
#include <tf2_ros/transform_broadcaster.h>

#include <robot_state_publisher/robot_state_publisher.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "rclcpp/rclcpp.hpp"

namespace ocs2 {
namespace legged_robot {

class LeggedRobotVisualizer : public DummyObserver {
 public:
  /** Visualization settings (publicly available) */
  std::string frameId_ = "odom";  // Frame name all messages are published in
  scalar_t footMarkerDiameter_ = 0.03;  // Size of the spheres at the feet
  scalar_t footAlphaWhenLifted_ = 0.3;  // Alpha value when a foot is lifted.
  scalar_t forceScale_ = 1000.0;        // Vector scale in N/m
  scalar_t velScale_ = 5.0;             // Vector scale in m/s
  scalar_t copMarkerDiameter_ =
      0.03;  // Size of the sphere at the center of pressure
  scalar_t supportPolygonLineWidth_ =
      0.005;  // LineThickness for the support polygon
  scalar_t trajectoryLineWidth_ = 0.01;  // LineThickness for trajectories
  std::vector<Color> feetColorMap_ = {
      Color::blue, Color::orange, Color::yellow,
      Color::purple};  // Colors for markers per feet

  /**
   *
   * @param pinocchioInterface
   * @param n
   * @param maxUpdateFrequency : maximum publish frequency measured in MPC time.
   */
  LeggedRobotVisualizer(
      PinocchioInterface pinocchioInterface,
      CentroidalModelInfo centroidalModelInfo,
      const PinocchioEndEffectorKinematics& endEffectorKinematics,
      const rclcpp::Node::SharedPtr& node, scalar_t maxUpdateFrequency = 100.0);

  ~LeggedRobotVisualizer() override = default;

  void update(const SystemObservation& observation,
              const PrimalSolution& primalSolution,
              const CommandData& command) override;

  void publishTrajectory(
      const std::vector<SystemObservation>& system_observation_array,
      scalar_t speed = 1.0);

  void publishObservation(rclcpp::Time timeStamp,
                          const SystemObservation& observation);

  void publishDesiredTrajectory(rclcpp::Time timeStamp,
                                const TargetTrajectories& targetTrajectories);

  void publishOptimizedStateTrajectory(rclcpp::Time timeStamp,
                                       const scalar_array_t& mpcTimeTrajectory,
                                       const vector_array_t& mpcStateTrajectory,
                                       const ModeSchedule& modeSchedule);

 protected:
  rclcpp::Node::SharedPtr node_;

 private:
  LeggedRobotVisualizer(const LeggedRobotVisualizer&) = delete;
  void publishJointTransforms(rclcpp::Time timeStamp,
                              const vector_t& jointAngles) const;
  void publishBaseTransform(rclcpp::Time timeStamp, const vector_t& basePose);
  void publishCartesianMarkers(rclcpp::Time timeStamp,
                               const contact_flag_t& contactFlags,
                               const std::vector<vector3_t>& feetPositions,
                               const std::vector<vector3_t>& feetForces) const;

  PinocchioInterface pinocchioInterface_;
  const CentroidalModelInfo centroidalModelInfo_;
  std::unique_ptr<PinocchioEndEffectorKinematics> endEffectorKinematicsPtr_;

  tf2_ros::TransformBroadcaster tfBroadcaster_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointPublisher_;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      costDesiredBasePositionPublisher_;
  std::vector<rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr>
      costDesiredFeetPositionPublishers_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      stateOptimizedPublisher_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      currentStatePublisher_;

  scalar_t lastTime_;
  scalar_t minPublishTimeDifference_;
};

}  // namespace legged_robot
}  // namespace ocs2
