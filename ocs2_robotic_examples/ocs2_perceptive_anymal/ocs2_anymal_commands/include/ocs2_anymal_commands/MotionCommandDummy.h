//
// Created by rgrandia on 14.10.21.
//

#pragma once

#include "ocs2_anymal_commands/MotionCommandInterface.h"

#include <visualization_msgs/Marker.h>

#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_msgs/mpc_observation.h>
#include <ocs2_switched_model_interface/terrain/TerrainPlane.h>

namespace switched_model {

class MotionCommandDummy : public MotionCommandInterface {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  MotionCommandDummy(ros::NodeHandle& nodeHandle, const std::string& configFile, const std::string& robotName);
  ~MotionCommandDummy() override = default;

  void publishMotion(const std::pair<ocs2::TargetTrajectories, Gait>& motion) override;

 private:
  void observationCallback(const ocs2_msgs::mpc_observation::ConstPtr& msg);

  void terrainCallback(const visualization_msgs::Marker::ConstPtr& msg);

  ros::Publisher targetTrajectoryPublisher_;
  ros::Publisher gaitSequencePublisher_;

  ros::Subscriber observationSubscriber_;
  std::mutex observationMutex_;
  std::unique_ptr<ocs2::SystemObservation> observationPtr_;

  ros::Subscriber terrainSubscriber_;
  std::mutex terrainMutex_;
  TerrainPlane localTerrain_;
};

}  // namespace switched_model