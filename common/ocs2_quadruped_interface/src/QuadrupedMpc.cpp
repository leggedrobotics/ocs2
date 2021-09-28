//
// Created by rgrandia on 06.07.21.
//

#include "ocs2_quadruped_interface/QuadrupedMpc.h"

#include <ocs2_mpc/MPC_DDP.h>
#include <ocs2_sqp/MultipleShootingMpc.h>

namespace switched_model {

std::unique_ptr<ocs2::MPC_BASE> getDdpMpc(const QuadrupedInterface& quadrupedInterface, const ocs2::mpc::Settings& mpcSettings,
                                          const ocs2::ddp::Settings& ddpSettings) {
  std::unique_ptr<ocs2::MPC_BASE> mpcPtr(new ocs2::MPC_DDP(mpcSettings, ddpSettings, quadrupedInterface.getRollout(),
                                                           quadrupedInterface.getOptimalControlProblem(),
                                                           quadrupedInterface.getInitializer()));
  mpcPtr->getSolverPtr()->setReferenceManager(quadrupedInterface.getReferenceManagerPtr());
  return mpcPtr;
}

std::unique_ptr<ocs2::MPC_BASE> getSqpMpc(const QuadrupedInterface& quadrupedInterface, const ocs2::mpc::Settings& mpcSettings,
                                          const ocs2::multiple_shooting::Settings& sqpSettings) {
  auto mpcPtr = std::unique_ptr<ocs2::MultipleShootingMpc>(new ocs2::MultipleShootingMpc(
      mpcSettings, sqpSettings, quadrupedInterface.getOptimalControlProblem(), quadrupedInterface.getInitializer()));
  mpcPtr->getSolverPtr()->setReferenceManager(quadrupedInterface.getReferenceManagerPtr());
  return mpcPtr;
}

}  // namespace switched_model