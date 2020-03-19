//
// Created by rgrandia on 18.03.20.
//

#include "ocs2_quadruped_loopshaping_interface/LoopshapingSynchronizedModule.h"

namespace switched_model_loopshaping {

LoopshapingSynchronizedModule::LoopshapingSynchronizedModule(
    std::vector<std::shared_ptr<switched_model_solver_module_t>> synchronizedModules,
    std::shared_ptr<ocs2::LoopshapingDefinition> loopshapingDefinition)
    : synchronizedModules_(std::move(synchronizedModules)), loopshapingDefinition_(std::move(loopshapingDefinition)) {}

void LoopshapingSynchronizedModule::preSolverRun(scalar_t initTime, scalar_t finalTime, const state_vector_t& currentState,
                                                 const ocs2::CostDesiredTrajectories& costDesiredTrajectory) {
  switched_model_solver_module_t::state_vector_t systemState;
  loopshapingDefinition_->getSystemState(currentState, systemState);

  for (auto& module : synchronizedModules_) {
    module->preSolverRun(initTime, finalTime, systemState, costDesiredTrajectory);
  }
}

void LoopshapingSynchronizedModule::postSolverRun(const primal_solution_t& primalSolution) {
  // TODO (rgrandia) convert primal solution.
  switched_model_solver_module_t::primal_solution_t switchedModelPrimalSolution;
  for (auto& module : synchronizedModules_) {
    module->postSolverRun(switchedModelPrimalSolution);
  }
}

}  // namespace switched_model_loopshaping
