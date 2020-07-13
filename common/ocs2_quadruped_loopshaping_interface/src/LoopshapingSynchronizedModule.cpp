//
// Created by rgrandia on 18.03.20.
//

#include "ocs2_quadruped_loopshaping_interface/LoopshapingSynchronizedModule.h"

namespace switched_model_loopshaping {

LoopshapingSynchronizedModule::LoopshapingSynchronizedModule(
    std::vector<std::shared_ptr<ocs2::SolverSynchronizedModule>> synchronizedModules,
    std::shared_ptr<ocs2::LoopshapingDefinition> loopshapingDefinition)
    : synchronizedModules_(std::move(synchronizedModules)), loopshapingDefinition_(std::move(loopshapingDefinition)) {}

void LoopshapingSynchronizedModule::preSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t& currentState,
                                                 const ocs2::CostDesiredTrajectories& costDesiredTrajectory) {
  vector_t systemState = loopshapingDefinition_->getSystemState(currentState);

  for (auto& module : synchronizedModules_) {
    module->preSolverRun(initTime, finalTime, systemState, costDesiredTrajectory);
  }
}

void LoopshapingSynchronizedModule::postSolverRun(const ocs2::PrimalSolution& primalSolution) {
  // TODO (rgrandia) convert primal solution.
  ocs2::PrimalSolution switchedModelPrimalSolution;
  for (auto& module : synchronizedModules_) {
    module->postSolverRun(switchedModelPrimalSolution);
  }
}

}  // namespace switched_model_loopshaping
