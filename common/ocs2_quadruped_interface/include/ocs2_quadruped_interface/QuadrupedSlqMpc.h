//
// Created by rgrandia on 18.02.20.
//

#pragma once

#include <ocs2_ddp/DDP_Settings.h>
#include <ocs2_ddp/SLQ.h>

#include <ocs2_mpc/MPC_DDP.h>
#include <ocs2_mpc/MPC_Settings.h>

#include "ocs2_quadruped_interface/QuadrupedInterface.h"

namespace switched_model {

/** Constructs an SLQ object */
std::unique_ptr<ocs2::SLQ> getSlq(const QuadrupedInterface& quadrupedInterface, const ocs2::ddp::Settings& ddpSettings);

/** Constructs an MPC object */
std::unique_ptr<ocs2::MPC_DDP> getMpc(const QuadrupedInterface& quadrupedInterface, const ocs2::mpc::Settings& mpcSettings,
                                      const ocs2::ddp::Settings& ddpSettings);

}  // namespace switched_model
