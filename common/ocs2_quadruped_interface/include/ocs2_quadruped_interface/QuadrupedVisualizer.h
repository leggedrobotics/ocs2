//
// Created by rgrandia on 13.02.19.
//

#pragma once

#include <robot_state_publisher/robot_state_publisher.h>
#include <ros/node_handle.h>
#include <tf/transform_broadcaster.h>

#include <ocs2_switched_model_interface/core/KinematicsModelBase.h>
#include <ocs2_switched_model_interface/core/SwitchedModel.h>

#include <ocs2_ros_interfaces/mrt/DummyObserver.h>
#include <ocs2_ros_interfaces/visualization/VisualizationColors.h>

#include "ocs2_quadruped_interface/QuadrupedTfPublisher.h"

namespace switched_model {

class QuadrupedVisualizer : public ocs2::DummyObserver {
 public:
  using kinematic_model_t = KinematicsModelBase<scalar_t>;

  /** Visualization settings (publicly available) */
  std::string frameId_ = "world";             // Frame name all messages are published in
  scalar_t footMarkerDiameter_ = 0.03;        // Size of the spheres at the feet
  scalar_t footAlphaWhenLifted_ = 0.3;        // Alpha value when a foot is lifted.
  scalar_t forceScale_ = 1000.0;              // Vector scale in N/m
  scalar_t velScale_ = 20.0;                  // Vector scale in m/s
  scalar_t copMarkerDiameter_ = 0.03;         // Size of the sphere at the center of pressure
  scalar_t supportPolygonLineWidth_ = 0.005;  // LineThickness for the support polygon
  scalar_t trajectoryLineWidth_ = 0.01;       // LineThickness for trajectories
  feet_array_t<ocs2::Color> feetColorMap_ = {ocs2::Color::blue, ocs2::Color::orange, ocs2::Color::yellow,
                                             ocs2::Color::purple};  // Colors for markers per feet

  /**
   *
   * @param kinematicModel
   * @param comModel
   * @param n
   * @param maxUpdateFrequency : maximum publish frequency measured in MPC time.
   */
  QuadrupedVisualizer(const kinematic_model_t& kinematicModel, std::vector<std::string> jointNames, std::string baseName,
                      ros::NodeHandle& n, scalar_t maxUpdateFrequency = 1000.0)
      : kinematicModelPtr_(kinematicModel.clone()),
        lastTime_(std::numeric_limits<scalar_t>::lowest()),
        minPublishTimeDifference_(1.0 / maxUpdateFrequency) {
    launchVisualizerNode(n, std::move(jointNames), std::move(baseName));
  };

  ~QuadrupedVisualizer() override = default;

  void update(const ocs2::SystemObservation& observation, const ocs2::PrimalSolution& primalSolution,
              const ocs2::CommandData& command) override;

  void launchVisualizerNode(ros::NodeHandle& nodeHandle, std::vector<std::string> jointNames, std::string baseName);

  void publishTrajectory(const std::vector<ocs2::SystemObservation>& system_observation_array, scalar_t speed = 1.0);

  void publishObservation(ros::Time timeStamp, const ocs2::SystemObservation& observation);

  void publishDesiredTrajectory(ros::Time timeStamp, const ocs2::TargetTrajectories& targetTrajectories) const;

  void publishOptimizedStateTrajectory(ros::Time timeStamp, const scalar_array_t& mpcTimeTrajectory,
                                       const vector_array_t& mpcStateTrajectory, const ocs2::ModeSchedule& modeSchedule) const;

 private:
  void publishCartesianMarkers(ros::Time timeStamp, const contact_flag_t& contactFlags, const feet_array_t<vector3_t>& feetPosition,
                               const feet_array_t<vector3_t>& feetForce) const;
  void publishEndEffectorPoses(ros::Time timeStamp, const feet_array_t<vector3_t>& feetPositions,
                               const feet_array_t<Eigen::Quaternion<scalar_t>>& feetOrientations) const;
  void publishCollisionSpheres(ros::Time timeStamp, const base_coordinate_t& basePose, const joint_coordinate_t& jointAngles) const;

  std::unique_ptr<kinematic_model_t> kinematicModelPtr_;

  QuadrupedTfPublisher quadrupedTfPublisher_;

  // Cost desired publisher.
  ros::Publisher costDesiredBasePositionPublisher_;
  ros::Publisher costDesiredBasePosePublisher_;
  ros::Publisher costDesiredBaseVelocityPublisher_;
  ros::Publisher costDesiredBaseAngVelocityPublisher_;
  feet_array_t<ros::Publisher> costDesiredFeetPositionPublishers_;
  feet_array_t<ros::Publisher> costDesiredFeetVelocityPublishers_;

  // State optimized publisher.
  ros::Publisher stateOptimizedPublisher_;
  ros::Publisher stateOptimizedPosePublisher_;

  // Current state publisher..
  ros::Publisher currentStatePublisher_;
  ros::Publisher currentFeetPosesPublisher_;
  ros::Publisher currentCollisionSpheresPublisher_;

  scalar_t lastTime_;
  scalar_t minPublishTimeDifference_;
};

}  // namespace switched_model
