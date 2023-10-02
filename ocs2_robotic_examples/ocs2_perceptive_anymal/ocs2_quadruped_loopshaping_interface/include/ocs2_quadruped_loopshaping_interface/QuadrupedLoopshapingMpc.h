//
// Created by rgrandia on 06.07.21.
//

#pragma once

#include <ocs2_ddp/DDP_Settings.h>
#include <ocs2_mpc/MPC_BASE.h>
#include <ocs2_mpc/MPC_Settings.h>
#include <ocs2_sqp/SqpSettings.h>

#include "ocs2_quadruped_loopshaping_interface/QuadrupedLoopshapingInterface.h"

namespace switched_model_loopshaping {

/** Constructs an DDP MPC object */
std::unique_ptr<ocs2::MPC_BASE> getDdpMpc(const QuadrupedLoopshapingInterface& quadrupedInterface, const ocs2::mpc::Settings& mpcSettings,
                                          const ocs2::ddp::Settings& ddpSettings);

/** Constructs an SQP MPC object */
std::unique_ptr<ocs2::MPC_BASE> getSqpMpc(const QuadrupedLoopshapingInterface& quadrupedInterface, const ocs2::mpc::Settings& mpcSettings,
                                          const ocs2::sqp::Settings& sqpSettings);

}  // namespace switched_model_loopshaping