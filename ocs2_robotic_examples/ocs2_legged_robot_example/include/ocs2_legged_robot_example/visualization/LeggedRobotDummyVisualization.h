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
