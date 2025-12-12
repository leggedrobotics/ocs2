//
// Created by rgrandia on 17.02.20.
//

#pragma once

#include "rclcpp/rclcpp.hpp"

#include <ocs2_mpc/MPC_BASE.h>

#include "QuadrupedLoopshapingInterface.h"

namespace switched_model_loopshaping {

void quadrupedLoopshapingMpcNode(const rclcpp::Node::SharedPtr &node, const QuadrupedLoopshapingInterface& quadrupedInterface,
                                 std::unique_ptr<ocs2::MPC_BASE> mpcPtr);

}  // namespace switched_model_loopshaping