//
// Created by rgrandia on 18.02.20.
//

#include "ocs2_quadruped_loopshaping_interface/QuadrupedLoopshapingSqpMpc.h"

namespace switched_model_loopshaping {

std::unique_ptr<ocs2::MultipleShootingMpc> getSqpMpc(const QuadrupedLoopshapingInterface& quadrupedInterface,
                                                     const ocs2::mpc::Settings& mpcSettings,
                                                     const ocs2::multiple_shooting::Settings& sqpSettings) {
  auto mpcPtr = std::unique_ptr<ocs2::MultipleShootingMpc>(new ocs2::MultipleShootingMpc(
      mpcSettings, sqpSettings, &quadrupedInterface.getDynamics(), &quadrupedInterface.getCost(), &quadrupedInterface.getOperatingPoints(),
      quadrupedInterface.getConstraintPtr(), quadrupedInterface.getTerminalCostPtr()));
  mpcPtr->getSolverPtr()->setModeScheduleManager(quadrupedInterface.getModeScheduleManagerPtr());
  return mpcPtr;
}

}  // namespace switched_model_loopshaping
