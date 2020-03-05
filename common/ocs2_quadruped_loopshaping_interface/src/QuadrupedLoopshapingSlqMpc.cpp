//
// Created by rgrandia on 18.02.20.
//

#include "ocs2_quadruped_loopshaping_interface/QuadrupedLoopshapingSlqMpc.h"

namespace switched_model_loopshaping {

std::unique_ptr<ocs2::SLQ<STATE_DIM, INPUT_DIM>> getSlq(const QuadrupedLoopshapingInterface& quadrupedInterface,
                                                        const ocs2::SLQ_Settings& slqSettings) {
  return std::unique_ptr<ocs2::SLQ<STATE_DIM, INPUT_DIM>>(new ocs2::SLQ<STATE_DIM, INPUT_DIM>(
      &quadrupedInterface.getRollout(), &quadrupedInterface.getDynamicsDerivatives(), quadrupedInterface.getConstraintPtr(),
      &quadrupedInterface.getCost(), &quadrupedInterface.getOperatingPoints(), slqSettings, quadrupedInterface.getLogicRulesPtr()));
}

std::unique_ptr<ocs2::MPC_SLQ<STATE_DIM, INPUT_DIM>> getMpc(const QuadrupedLoopshapingInterface& quadrupedInterface,
                                                            const ocs2::MPC_Settings& mpcSettings, const ocs2::SLQ_Settings& slqSettings) {
  if (!quadrupedInterface.modelSettings().gaitOptimization_) {
    return std::unique_ptr<ocs2::MPC_SLQ<STATE_DIM, INPUT_DIM>>(new ocs2::MPC_SLQ<STATE_DIM, INPUT_DIM>(
        &quadrupedInterface.getRollout(), &quadrupedInterface.getDynamicsDerivatives(), quadrupedInterface.getConstraintPtr(),
        &quadrupedInterface.getCost(), &quadrupedInterface.getOperatingPoints(), quadrupedInterface.getInitialPartitionTimes(), slqSettings,
        mpcSettings, quadrupedInterface.getLogicRulesPtr(), &quadrupedInterface.getInitialModeSequence()));
  } else {
    throw std::runtime_error("mpc_ocs2 not configured, set gait optimization to 0");
  }
}

}  // namespace switched_model_loopshaping

/**  Explicit instantiation of MPC and SLQ classes */
namespace ocs2 {
template class MPC_BASE<switched_model_loopshaping::STATE_DIM, switched_model_loopshaping::INPUT_DIM>;
template class MPC_SLQ<switched_model_loopshaping::STATE_DIM, switched_model_loopshaping::INPUT_DIM>;
template class Solver_BASE<switched_model_loopshaping::STATE_DIM, switched_model_loopshaping::INPUT_DIM>;
template class DDP_BASE<switched_model_loopshaping::STATE_DIM, switched_model_loopshaping::INPUT_DIM>;
template class SLQ<switched_model_loopshaping::STATE_DIM, switched_model_loopshaping::INPUT_DIM>;
}  // namespace ocs2
