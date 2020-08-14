//
// Created by rgrandia on 13.02.20.
//

#include "ocs2_quadruped_loopshaping_interface/QuadrupedLoopshapingVisualizer.h"

namespace switched_model_loopshaping {

QuadrupedLoopshapingVisualizer::QuadrupedLoopshapingVisualizer(std::shared_ptr<ocs2::LoopshapingDefinition> loopshapingDefinition,
                                                               std::unique_ptr<switched_model::QuadrupedVisualizer> quadrupedVisualizer)
    : loopshapingDefinition_(std::move(loopshapingDefinition)), quadrupedVisualizer_(std::move(quadrupedVisualizer)) {}

void QuadrupedLoopshapingVisualizer::update(const ocs2::SystemObservation& observation, const ocs2::PrimalSolution& primalSolution,
                                            const ocs2::CommandData& command) {
  ocs2::SystemObservation quadrupedObservation;
  quadrupedObservation.time = observation.time;
  quadrupedObservation.state = loopshapingDefinition_->getSystemState(observation.state);
  quadrupedObservation.input = loopshapingDefinition_->getSystemInput(observation.state, observation.input);
  quadrupedObservation.mode = observation.mode;

  const auto quadrupedStateTrajectory = [&] {
    vector_array_t quadrupedStateTrajectory{};
    quadrupedStateTrajectory.reserve(primalSolution.stateTrajectory_.size());
    for (const auto& loopshapingState : primalSolution.stateTrajectory_) {
      vector_t quadrupedState = loopshapingDefinition_->getSystemState(loopshapingState);
      quadrupedStateTrajectory.push_back(quadrupedState);
    }
    return quadrupedStateTrajectory;
  }();

  const auto timeStamp = ros::Time(observation.time);
  quadrupedVisualizer_->publishObservation(timeStamp, quadrupedObservation);
  quadrupedVisualizer_->publishDesiredTrajectory(timeStamp, command.mpcCostDesiredTrajectories_);
  quadrupedVisualizer_->publishOptimizedStateTrajectory(timeStamp, primalSolution.timeTrajectory_, quadrupedStateTrajectory,
                                                        primalSolution.modeSchedule_);
}

}  // namespace switched_model_loopshaping
