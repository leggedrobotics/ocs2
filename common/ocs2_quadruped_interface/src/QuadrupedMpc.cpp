//
// Created by rgrandia on 06.07.21.
//

#include "ocs2_quadruped_interface/QuadrupedMpc.h"

#include <ocs2_mpc/MPC_DDP.h>

namespace switched_model {

std::unique_ptr<ocs2::MPC_BASE> getSlqMpc(const QuadrupedInterface& quadrupedInterface, const ocs2::mpc::Settings& mpcSettings,
                                          const ocs2::ddp::Settings& ddpSettings,
                                          std::shared_ptr<ocs2::ReferenceManagerInterface> alternativeReferenceManager) {
  std::unique_ptr<ocs2::MPC_BASE> mpcPtr(new ocs2::MPC_DDP(&quadrupedInterface.getRollout(), &quadrupedInterface.getDynamics(),
                                                           quadrupedInterface.getConstraintPtr(), &quadrupedInterface.getCost(),
                                                           &quadrupedInterface.getInitializer(), ddpSettings, mpcSettings,
                                                           quadrupedInterface.getTerminalCostPtr()));
  if (alternativeReferenceManager) {
    mpcPtr->getSolverPtr()->setReferenceManager(alternativeReferenceManager);
  } else {
    mpcPtr->getSolverPtr()->setReferenceManager(quadrupedInterface.getReferenceManagerPtr());
  }
  return mpcPtr;
}

}  // namespace switched_model