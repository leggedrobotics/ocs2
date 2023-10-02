//
// Created by rgrandia on 06.07.21.
//

#include "ocs2_quadruped_loopshaping_interface/QuadrupedLoopshapingMpc.h"

#include <ocs2_ddp/GaussNewtonDDP_MPC.h>
#include <ocs2_sqp/SqpMpc.h>

namespace switched_model_loopshaping {

std::unique_ptr<ocs2::MPC_BASE> getDdpMpc(const QuadrupedLoopshapingInterface& quadrupedInterface, const ocs2::mpc::Settings& mpcSettings,
                                          const ocs2::ddp::Settings& ddpSettings) {
  std::unique_ptr<ocs2::MPC_BASE> mpcPtr(new ocs2::GaussNewtonDDP_MPC(mpcSettings, ddpSettings, quadrupedInterface.getRollout(),
                                                                      quadrupedInterface.getOptimalControlProblem(),
                                                                      quadrupedInterface.getInitializer()));
  mpcPtr->getSolverPtr()->setReferenceManager(quadrupedInterface.getReferenceManagerPtr());
  mpcPtr->getSolverPtr()->setSynchronizedModules({quadrupedInterface.getLoopshapingSynchronizedModule()});
  return mpcPtr;
}

std::unique_ptr<ocs2::MPC_BASE> getSqpMpc(const QuadrupedLoopshapingInterface& quadrupedInterface, const ocs2::mpc::Settings& mpcSettings,
                                          const ocs2::sqp::Settings& sqpSettings) {
  std::unique_ptr<ocs2::MPC_BASE> mpcPtr(
      new ocs2::SqpMpc(mpcSettings, sqpSettings, quadrupedInterface.getOptimalControlProblem(), quadrupedInterface.getInitializer()));
  mpcPtr->getSolverPtr()->setReferenceManager(quadrupedInterface.getReferenceManagerPtr());
  mpcPtr->getSolverPtr()->setSynchronizedModules({quadrupedInterface.getLoopshapingSynchronizedModule()});
  return mpcPtr;
}

}  // namespace switched_model_loopshaping
