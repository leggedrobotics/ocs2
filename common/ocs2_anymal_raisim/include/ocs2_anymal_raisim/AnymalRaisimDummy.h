#pragma once

#include <ocs2_quadruped_interface/QuadrupedInterface.h>
#include <ocs2_raisim/RaisimRolloutSettings.h>
#include <ros/node_handle.h>

namespace anymal {

void runAnymalRaisimDummy(ros::NodeHandle& nodeHandle, std::unique_ptr<switched_model::QuadrupedInterface> anymalInterface,
                          const std::string& urdf, double mrtDesiredFreq, double mpcDesiredFreq,
                          const ocs2::RaisimRolloutSettings& raisimRolloutSettings);

}
