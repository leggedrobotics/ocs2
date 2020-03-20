//
// Created by rgrandia on 18.02.20.
//

#include "ocs2_quadruped_interface/QuadrupedSlqMpc.h"

namespace switched_model {

std::unique_ptr<ocs2::SLQ<STATE_DIM, INPUT_DIM>> getSlq(const QuadrupedInterface& quadrupedInterface,
                                                        const ocs2::SLQ_Settings& slqSettings) {
  auto slqPtr = std::unique_ptr<ocs2::SLQ<STATE_DIM, INPUT_DIM>>(new ocs2::SLQ<STATE_DIM, INPUT_DIM>(
      &quadrupedInterface.getRollout(), &quadrupedInterface.getDynamicsDerivatives(), quadrupedInterface.getConstraintPtr(),
      &quadrupedInterface.getCost(), &quadrupedInterface.getOperatingPoints(), slqSettings));
  slqPtr->setModeScheduleManager(quadrupedInterface.getModeScheduleManagerPtr());
  return slqPtr;
}

std::unique_ptr<ocs2::MPC_SLQ<STATE_DIM, INPUT_DIM>> getMpc(const QuadrupedInterface& quadrupedInterface,
                                                            const ocs2::MPC_Settings& mpcSettings, const ocs2::SLQ_Settings& slqSettings) {
  if (!quadrupedInterface.modelSettings().gaitOptimization_) {
    auto mpcPtr = std::unique_ptr<ocs2::MPC_SLQ<STATE_DIM, INPUT_DIM>>(new ocs2::MPC_SLQ<STATE_DIM, INPUT_DIM>(
        &quadrupedInterface.getRollout(), &quadrupedInterface.getDynamicsDerivatives(), quadrupedInterface.getConstraintPtr(),
        &quadrupedInterface.getCost(), &quadrupedInterface.getOperatingPoints(), quadrupedInterface.getInitialPartitionTimes(), slqSettings,
        mpcSettings));
    mpcPtr->getSolverPtr()->setModeScheduleManager(quadrupedInterface.getModeScheduleManagerPtr());
    return mpcPtr;
  } else {
    throw std::runtime_error("mpc_ocs2 not configured, set gait optimization to 0");
  }
}

}  // namespace switched_model

/**  Explicit instantiation of MPC and SLQ classes */
namespace ocs2 {
template class MPC_BASE<switched_model::STATE_DIM, switched_model::INPUT_DIM>;
template class MPC_SLQ<switched_model::STATE_DIM, switched_model::INPUT_DIM>;
template class Solver_BASE<switched_model::STATE_DIM, switched_model::INPUT_DIM>;
template class GaussNewtonDDP<switched_model::STATE_DIM, switched_model::INPUT_DIM>;
template class SLQ<switched_model::STATE_DIM, switched_model::INPUT_DIM>;
}  // namespace ocs2
