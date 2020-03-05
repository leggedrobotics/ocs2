//
// Created by rgrandia on 13.02.20.
//

#include "ocs2_quadruped_loopshaping_interface/QuadrupedLoopshapingXppVisualizer.h"

namespace switched_model_loopshaping {

QuadrupedLoopshapingXppVisualizer::QuadrupedLoopshapingXppVisualizer(
    std::shared_ptr<ocs2::LoopshapingDefinition> loopshapingDefinition,
    std::unique_ptr<switched_model::QuadrupedVisualizer> quadrupedXppVisualizer)
    : loopshapingDefinition_(std::move(loopshapingDefinition)), quadrupedXppVisualizer_(std::move(quadrupedXppVisualizer)) {}

void QuadrupedLoopshapingXppVisualizer::update(const system_observation_t& observation, const primal_solution_t& primalSolution,
                                               const command_data_t& command) {
  switched_model::QuadrupedVisualizer::system_observation_t quadrupedObservation;
  quadrupedObservation.time() = observation.time();
  loopshapingDefinition_->getSystemState(observation.state(), quadrupedObservation.state());
  loopshapingDefinition_->getSystemInput(observation.state(), observation.input(), quadrupedObservation.input());
  quadrupedObservation.subsystem() = observation.subsystem();

  const auto timeStamp = ros::Time(observation.time());
  quadrupedXppVisualizer_->publishObservation(timeStamp, quadrupedObservation);
}

}  // namespace switched_model_loopshaping
