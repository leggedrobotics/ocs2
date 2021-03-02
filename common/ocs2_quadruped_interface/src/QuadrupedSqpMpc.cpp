//
// Created by shyang on 2020.12.15
//

#include "ocs2_quadruped_interface/QuadrupedSqpMpc.h"

namespace switched_model {

std::unique_ptr<ocs2::MultipleShootingMpc> getSqpMpc(const QuadrupedInterface& quadrupedInterface, const ocs2::mpc::Settings& mpcSettings,
                                                     const ocs2::MultipleShootingSolverSettings& sqpSettings) {
  if (!quadrupedInterface.modelSettings().gaitOptimization_) {
    auto mpcPtr = std::unique_ptr<ocs2::MultipleShootingMpc>(new ocs2::MultipleShootingMpc(
        mpcSettings, sqpSettings, &quadrupedInterface.getDynamics(), &quadrupedInterface.getCost(), quadrupedInterface.getConstraintPtr()));
    mpcPtr->getSolverPtr()->setModeScheduleManager(quadrupedInterface.getModeScheduleManagerPtr());
    return mpcPtr;
  } else {
    throw std::runtime_error("sqp mpc_ocs2 not configured, set gait optimization to 0");
  }
}

}  // namespace switched_model
