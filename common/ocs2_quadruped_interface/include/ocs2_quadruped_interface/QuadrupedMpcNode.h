//
// Created by rgrandia on 17.02.20.
//

#pragma once

#include <ros/node_handle.h>

#include <ocs2_ddp/SlqSettings.h>
#include <ocs2_mpc/MpcSettings.h>

#include "QuadrupedInterface.h"

namespace switched_model {

void quadrupedMpcNode(ros::NodeHandle& nodeHandle, const QuadrupedInterface& quadrupedInterface, const ocs2::MpcSettings& mpcSettings,
                      const ocs2::SlqSettings& slqSettings);

}
