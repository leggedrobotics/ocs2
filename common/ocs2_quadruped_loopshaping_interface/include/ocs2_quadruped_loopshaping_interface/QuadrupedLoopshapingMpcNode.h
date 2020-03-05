//
// Created by rgrandia on 17.02.20.
//

#pragma once

#include <ros/node_handle.h>

#include <ocs2_ddp/SLQ_Settings.h>
#include <ocs2_mpc/MPC_Settings.h>

#include "QuadrupedLoopshapingInterface.h"

namespace switched_model_loopshaping {

void quadrupedLoopshapingMpcNode(ros::NodeHandle& nodeHandle, const QuadrupedLoopshapingInterface& quadrupedInterface,
                                 const ocs2::MPC_Settings& mpcSettings, const ocs2::SLQ_Settings& slqSettings);

}  // namespace switched_model_loopshaping