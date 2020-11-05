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
  if (!synchronizedModules_.empty()) {
    const vector_t systemState = loopshapingDefinition_->getSystemState(currentState);

    for (auto& module : synchronizedModules_) {
      module->preSolverRun(initTime, finalTime, systemState, costDesiredTrajectory);
    }
  }
}

void LoopshapingSynchronizedModule::postSolverRun(const ocs2::PrimalSolution& primalSolution) {
  if (!synchronizedModules_.empty()) {
    ocs2::PrimalSolution systemPrimalSolution;
    systemPrimalSolution.timeTrajectory_ = primalSolution.timeTrajectory_;
    systemPrimalSolution.modeSchedule_ = primalSolution.modeSchedule_;
    systemPrimalSolution.stateTrajectory_.reserve(primalSolution.stateTrajectory_.size());
    systemPrimalSolution.inputTrajectory_.reserve(primalSolution.inputTrajectory_.size());
    for (size_t k = 0; k < primalSolution.stateTrajectory_.size(); ++k) {
      const auto quadrupedState = loopshapingDefinition_->getSystemState(primalSolution.stateTrajectory_[k]);
      const auto quadrupedInput = loopshapingDefinition_->getSystemInput(quadrupedState, primalSolution.inputTrajectory_[k]);
      systemPrimalSolution.stateTrajectory_.push_back(quadrupedState);
      systemPrimalSolution.inputTrajectory_.push_back(quadrupedInput);
    }

    for (auto& module : synchronizedModules_) {
      module->postSolverRun(systemPrimalSolution);
    }
  }
}

}  // namespace switched_model_loopshaping
