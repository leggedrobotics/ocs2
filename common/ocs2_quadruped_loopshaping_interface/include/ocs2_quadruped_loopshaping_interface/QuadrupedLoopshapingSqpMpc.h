//
// Created by rgrandia on 18.02.20.
//

#pragma once

#include "ocs2_sqp/MultipleShootingMpc.h"

#include "ocs2_quadruped_loopshaping_interface/QuadrupedLoopshapingInterface.h"

namespace switched_model_loopshaping {

/** Constructs an MPC object */
std::unique_ptr<ocs2::MultipleShootingMpc> getSqpMpc(const QuadrupedLoopshapingInterface& quadrupedInterface,
                                                     const ocs2::mpc::Settings& mpcSettings,
                                                     const ocs2::multiple_shooting::Settings& sqpSettings);

}  // namespace switched_model_loopshaping
