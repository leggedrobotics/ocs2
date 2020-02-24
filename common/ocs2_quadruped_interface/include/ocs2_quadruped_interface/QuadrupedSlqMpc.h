//
// Created by rgrandia on 18.02.20.
//

#pragma once

#include <ocs2_ddp/SLQ.h>
#include <ocs2_ddp/SLQ_Settings.h>

#include <ocs2_mpc/MPC_SLQ.h>
#include <ocs2_mpc/MPC_Settings.h>

#include "ocs2_quadruped_interface/QuadrupedInterface.h"

namespace switched_model {

/** Constructs an SLQ object */
std::unique_ptr<ocs2::SLQ<STATE_DIM, INPUT_DIM>> getSlq(const QuadrupedInterface& quadrupedInterface,
                                                        const ocs2::SLQ_Settings& slqSettings);

/** Constructs an MPC object */
std::unique_ptr<ocs2::MPC_SLQ<STATE_DIM, INPUT_DIM>> getMpc(const QuadrupedInterface& quadrupedInterface,
                                                            const ocs2::MPC_Settings& mpcSettings, const ocs2::SLQ_Settings& slqSettings);

}  // namespace switched_model

/**  Explicit instantiation of MPC and SLQ classes */
namespace ocs2 {
extern template class MPC_BASE<switched_model::STATE_DIM, switched_model::INPUT_DIM>;
extern template class MPC_SLQ<switched_model::STATE_DIM, switched_model::INPUT_DIM>;
extern template class Solver_BASE<switched_model::STATE_DIM, switched_model::INPUT_DIM>;
extern template class DDP_BASE<switched_model::STATE_DIM, switched_model::INPUT_DIM>;
extern template class SLQ<switched_model::STATE_DIM, switched_model::INPUT_DIM>;
}  // namespace ocs2
