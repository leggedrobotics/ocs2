//
// Created by rgrandia on 17.02.20.
//

#pragma once

#include <ros/node_handle.h>

#include "QuadrupedLoopshapingInterface.h"

namespace switched_model_loopshaping {

void quadrupedLoopshapingDummyNode(ros::NodeHandle& nodeHandle, const QuadrupedLoopshapingInterface& quadrupedInterface,
                                   double mrtDesiredFrequency, double mpcDesiredFrequency);

}  // namespace switched_model_loopshaping
