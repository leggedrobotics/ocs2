//
// Created by rgrandia on 17.02.20.
//

#pragma once

#include "rclcpp/rclcpp.hpp"

#include "QuadrupedInterface.h"

namespace switched_model {

void quadrupedDummyNode(const rclcpp::Node::SharedPtr &node, const QuadrupedInterface& quadrupedInterface, const ocs2::RolloutBase* rolloutPtr,
                        double mrtDesiredFrequency, double mpcDesiredFrequency);
}
