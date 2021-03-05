//
// Created by rgrandia on 17.02.20.
//

#pragma once

#include <ros/node_handle.h>

#include <ocs2_mpc/MPC_BASE.h>

#include "QuadrupedInterface.h"

namespace switched_model {

void quadrupedMpcNode(ros::NodeHandle& nodeHandle, const QuadrupedInterface& quadrupedInterface, std::unique_ptr<ocs2::MPC_BASE> mpcPtr);
}
