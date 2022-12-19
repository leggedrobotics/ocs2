//
// Created by rgrandia on 17.02.20.
//

#pragma once

#include <ros/node_handle.h>

#include "QuadrupedInterface.h"

namespace switched_model {

void quadrupedDummyNode(ros::NodeHandle& nodeHandle, const QuadrupedInterface& quadrupedInterface, const ocs2::RolloutBase* rolloutPtr,
                        double mrtDesiredFrequency, double mpcDesiredFrequency);
}
