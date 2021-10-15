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

class MotionCommandController : public MotionCommandInterface {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  MotionCommandController(ros::NodeHandle& nodeHandle, const std::string& configFile, const std::string& controllerName);
  ~MotionCommandController() override = default;

  void publishMotion(const std::pair<ocs2::TargetTrajectories, Gait>& motion) override;

 private:
  ros::ServiceClient client;
};

}  // namespace switched_model