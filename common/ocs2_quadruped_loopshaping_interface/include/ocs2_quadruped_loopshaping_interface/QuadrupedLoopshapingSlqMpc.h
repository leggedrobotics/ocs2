//
// Created by rgrandia on 18.02.20.
//

#pragma once

#include <ocs2_mpc/MPC_SLQ.h>

#include "ocs2_quadruped_loopshaping_interface/QuadrupedLoopshapingInterface.h"

namespace switched_model_loopshaping {

/** Constructs an SLQ object */
std::unique_ptr<ocs2::SLQ<STATE_DIM, INPUT_DIM>> getSlq(const QuadrupedLoopshapingInterface& quadrupedInterface);

/** Constructs an MPC object */
std::unique_ptr<ocs2::MPC_SLQ<STATE_DIM, INPUT_DIM>> getMpc(const QuadrupedLoopshapingInterface& quadrupedInterface);

}  // namespace switched_model_loopshaping

/**  Explicit instantiation of MPC and SLQ classes */
namespace ocs2 {
extern template class MPC_BASE<switched_model_loopshaping::STATE_DIM, switched_model_loopshaping::INPUT_DIM>;
extern template class MPC_SLQ<switched_model_loopshaping::STATE_DIM, switched_model_loopshaping::INPUT_DIM>;
extern template class Solver_BASE<switched_model_loopshaping::STATE_DIM, switched_model_loopshaping::INPUT_DIM>;
extern template class DDP_BASE<switched_model_loopshaping::STATE_DIM, switched_model_loopshaping::INPUT_DIM>;
extern template class SLQ<switched_model_loopshaping::STATE_DIM, switched_model_loopshaping::INPUT_DIM>;
}  // namespace ocs2