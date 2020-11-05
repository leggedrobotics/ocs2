//
// Created by rgrandia on 13.02.20.
//

#include "ocs2_quadruped_loopshaping_interface/QuadrupedLoopshapingDummyObserver.h"

namespace switched_model_loopshaping {

QuadrupedLoopshapingDummyObserver::QuadrupedLoopshapingDummyObserver(std::shared_ptr<ocs2::LoopshapingDefinition> loopshapingDefinition,
                                                                     std::vector<std::shared_ptr<DummyObserver>> observers)
    : loopshapingDefinition_(std::move(loopshapingDefinition)), observers_(std::move(observers)) {}

void QuadrupedLoopshapingDummyObserver::update(const ocs2::SystemObservation& observation, const ocs2::PrimalSolution& primalSolution,
                                               const ocs2::CommandData& command) {
  if (!observers_.empty()) {
    ocs2::SystemObservation quadrupedObservation;
    quadrupedObservation.time = observation.time;
    quadrupedObservation.state = loopshapingDefinition_->getSystemState(observation.state);
    quadrupedObservation.input = loopshapingDefinition_->getSystemInput(observation.state, observation.input);
    quadrupedObservation.mode = observation.mode;

    ocs2::PrimalSolution quadrupedPrimalSolution;
    quadrupedPrimalSolution.timeTrajectory_ = primalSolution.timeTrajectory_;
    quadrupedPrimalSolution.modeSchedule_ = primalSolution.modeSchedule_;
    quadrupedPrimalSolution.stateTrajectory_.reserve(primalSolution.stateTrajectory_.size());
    quadrupedPrimalSolution.inputTrajectory_.reserve(primalSolution.inputTrajectory_.size());
    for (size_t k = 0; k < primalSolution.stateTrajectory_.size(); ++k) {
      const auto quadrupedState = loopshapingDefinition_->getSystemState(primalSolution.stateTrajectory_[k]);
      const auto quadrupedInput = loopshapingDefinition_->getSystemInput(quadrupedState, primalSolution.inputTrajectory_[k]);
      quadrupedPrimalSolution.stateTrajectory_.push_back(quadrupedState);
      quadrupedPrimalSolution.inputTrajectory_.push_back(quadrupedInput);
    }

    for (auto& observer : observers_) {
      observer->update(quadrupedObservation, quadrupedPrimalSolution, command);
    }
  }
}

}  // namespace switched_model_loopshaping
