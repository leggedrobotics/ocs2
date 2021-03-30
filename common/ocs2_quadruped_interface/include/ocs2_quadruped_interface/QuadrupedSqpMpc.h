//
// Created by shyang on 2020.12.15
//

#pragma once

#include "ocs2_sqp/MultipleShootingMpc.h"

#include "ocs2_quadruped_interface/QuadrupedInterface.h"

namespace switched_model {

/** Constructs an SQP MPC object */
std::unique_ptr<ocs2::MultipleShootingMpc> getSqpMpc(const QuadrupedInterface& quadrupedInterface, const ocs2::mpc::Settings& mpcSettings,
                                                     const ocs2::multiple_shooting::Settings& sqpSettings);

}  // namespace switched_model
