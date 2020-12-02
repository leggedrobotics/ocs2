//
// Created by rgrandia on 18.02.20.
//

#include "ocs2_quadruped_interface/QuadrupedSlqMpc.h"

namespace switched_model {

std::unique_ptr<ocs2::SLQ> getSlq(const QuadrupedInterface& quadrupedInterface, const ocs2::ddp::Settings& ddpSettings) {
  auto slqPtr = std::unique_ptr<ocs2::SLQ>(new ocs2::SLQ(
      &quadrupedInterface.getRollout(), &quadrupedInterface.getDynamics(), quadrupedInterface.getConstraintPtr(),
      &quadrupedInterface.getCost(), &quadrupedInterface.getOperatingPoints(), ddpSettings, quadrupedInterface.getTerminalCostPtr()));
  slqPtr->setModeScheduleManager(quadrupedInterface.getModeScheduleManagerPtr());
  return slqPtr;
}

std::unique_ptr<ocs2::MPC_DDP> getMpc(const QuadrupedInterface& quadrupedInterface, const ocs2::mpc::Settings& mpcSettings,
                                      const ocs2::ddp::Settings& ddpSettings) {
  if (!quadrupedInterface.modelSettings().gaitOptimization_) {
    auto mpcPtr = std::unique_ptr<ocs2::MPC_DDP>(new ocs2::MPC_DDP(&quadrupedInterface.getRollout(), &quadrupedInterface.getDynamics(),
                                                                   quadrupedInterface.getConstraintPtr(), &quadrupedInterface.getCost(),
                                                                   &quadrupedInterface.getOperatingPoints(), ddpSettings, mpcSettings,
                                                                   quadrupedInterface.getTerminalCostPtr()));
    mpcPtr->getSolverPtr()->setModeScheduleManager(quadrupedInterface.getModeScheduleManagerPtr());
    return mpcPtr;
  } else {
    throw std::runtime_error("mpc_ocs2 not configured, set gait optimization to 0");
  }
}

}  // namespace switched_model
