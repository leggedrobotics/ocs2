//
// Created by rgrandia on 06.07.21.
//

#pragma once

#include <ocs2_ddp/DDP_Settings.h>
#include <ocs2_mpc/MPC_BASE.h>
#include <ocs2_mpc/MPC_Settings.h>

#include "ocs2_quadruped_loopshaping_interface/QuadrupedLoopshapingInterface.h"

namespace switched_model_loopshaping {

/** Constructs an SLQ MPC object, if no reference manager is provided, the one from the quadruped interface is taken */
std::unique_ptr<ocs2::MPC_BASE> getSlqMpc(const QuadrupedLoopshapingInterface& quadrupedInterface, const ocs2::mpc::Settings& mpcSettings,
                                          const ocs2::ddp::Settings& ddpSettings,
                                          std::shared_ptr<ocs2::ReferenceManagerInterface> alternativeReferenceManager = nullptr);

}  // namespace switched_model_loopshaping