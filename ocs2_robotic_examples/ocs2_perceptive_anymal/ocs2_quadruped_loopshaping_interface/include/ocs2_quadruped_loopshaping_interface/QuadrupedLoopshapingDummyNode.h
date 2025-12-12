//
// Created by rgrandia on 17.02.20.
//

#pragma once

#include "rclcpp/rclcpp.hpp"

#include "QuadrupedLoopshapingInterface.h"

namespace switched_model_loopshaping {

void quadrupedLoopshapingDummyNode(const rclcpp::Node::SharedPtr &node, const QuadrupedLoopshapingInterface& quadrupedInterface,
                                   double mrtDesiredFrequency, double mpcDesiredFrequency);

}  // namespace switched_model_loopshaping
