//
// Created by rgrandia on 17.02.20.
//

#pragma once

#include <ros/node_handle.h>

#include <ocs2_mpc/MPC_Settings.h>
#include "ocs2_sqp/MultipleShootingSolver.h"

#include "QuadrupedInterface.h"

namespace switched_model {

void quadrupedMpcNodeSqp(ros::NodeHandle& nodeHandle, const QuadrupedInterface& quadrupedInterface, const ocs2::mpc::Settings& mpcSettings,
                         const ocs2::MultipleShootingSolverSettings& sqpSettings);
}
