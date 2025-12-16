//
// Created by rgrandia on 17.02.20.
//

#pragma once

#include "rclcpp/rclcpp.hpp"

#include <ocs2_mpc/MPC_BASE.h>

#include "QuadrupedInterface.h"

namespace switched_model {

void quadrupedMpcNode(const rclcpp::Node::SharedPtr &node, const QuadrupedInterface& quadrupedInterface, std::unique_ptr<ocs2::MPC_BASE> mpcPtr);
}
