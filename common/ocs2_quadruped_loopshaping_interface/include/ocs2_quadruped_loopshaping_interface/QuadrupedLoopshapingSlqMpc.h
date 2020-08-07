//
// Created by rgrandia on 18.02.20.
//

#pragma once

#include <ocs2_ddp/DDP_Settings.h>
#include <ocs2_ddp/SLQ.h>

#include <ocs2_mpc/MPC_DDP.h>
#include <ocs2_mpc/MPC_Settings.h>

#include "ocs2_quadruped_loopshaping_interface/QuadrupedLoopshapingInterface.h"

namespace switched_model_loopshaping {

/** Constructs an SLQ object */
std::unique_ptr<ocs2::SLQ> getSlq(const QuadrupedLoopshapingInterface& quadrupedInterface, const ocs2::ddp::Settings& ddpSettings);

/** Constructs an MPC object */
std::unique_ptr<ocs2::MPC_DDP> getMpc(const QuadrupedLoopshapingInterface& quadrupedInterface, const ocs2::mpc::Settings& mpcSettings,
                                      const ocs2::ddp::Settings& ddpSettings);

}  // namespace switched_model_loopshaping
