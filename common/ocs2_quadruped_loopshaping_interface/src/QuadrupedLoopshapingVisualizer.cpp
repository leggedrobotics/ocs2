//
// Created by rgrandia on 13.02.20.
//

#include "ocs2_quadruped_loopshaping_interface/QuadrupedLoopshapingVisualizer.h"

namespace switched_model_loopshaping {

QuadrupedLoopshapingVisualizer::QuadrupedLoopshapingVisualizer(std::shared_ptr<ocs2::LoopshapingDefinition> loopshapingDefinition,
                                                               std::unique_ptr<switched_model::QuadrupedVisualizer> quadrupedVisualizer)
    : loopshapingDefinition_(std::move(loopshapingDefinition)), quadrupedVisualizer_(std::move(quadrupedVisualizer)) {}

void QuadrupedLoopshapingVisualizer::update(const system_observation_t& observation, const primal_solution_t& primalSolution,
                                            const command_data_t& command) {
  switched_model::QuadrupedVisualizer::system_observation_t quadrupedObservation;
  quadrupedObservation.time() = observation.time();
  loopshapingDefinition_->getSystemState(observation.state(), quadrupedObservation.state());
  loopshapingDefinition_->getSystemInput(observation.state(), observation.input(), quadrupedObservation.input());
  quadrupedObservation.subsystem() = observation.subsystem();

  const auto quadrupedStateTrajectory = [&] {
    switched_model::QuadrupedVisualizer::state_vector_array_t quadrupedStateTrajectory{};
    quadrupedStateTrajectory.reserve(primalSolution.stateTrajectory_.size());
    for (const auto& loopshapingState : primalSolution.stateTrajectory_) {
      switched_model::QuadrupedVisualizer::state_vector_t quadrupedState;
      loopshapingDefinition_->getSystemState(loopshapingState, quadrupedState);
      quadrupedStateTrajectory.push_back(quadrupedState);
    }
    return quadrupedStateTrajectory;
  }();

  const auto timeStamp = ros::Time(observation.time());
  quadrupedVisualizer_->publishObservation(timeStamp, quadrupedObservation);
  quadrupedVisualizer_->publishDesiredTrajectory(timeStamp, command.mpcCostDesiredTrajectories_);
  quadrupedVisualizer_->publishOptimizedStateTrajectory(timeStamp, primalSolution.timeTrajectory_, quadrupedStateTrajectory,
                                                        primalSolution.modeSchedule_);
}

}  // namespace switched_model_loopshaping
