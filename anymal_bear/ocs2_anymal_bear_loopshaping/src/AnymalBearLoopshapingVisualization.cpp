//
// Created by rgrandia on 13.02.20.
//

#include "ocs2_anymal_bear_loopshaping/AnymalBearLoopshapingVisualization.h"

namespace anymal {

AnymalBearLoopshapingVisualization::AnymalBearLoopshapingVisualization(
    std::shared_ptr<ocs2::LoopshapingDefinition> loopshapingDefinition,
    std::unique_ptr<switched_model::QuadrupedXppVisualizer<12>> anymalBearDummyVisualization)
    : loopshapingDefinition_(std::move(loopshapingDefinition)), anymalBearDummyVisualization_(std::move(anymalBearDummyVisualization)) {}

void AnymalBearLoopshapingVisualization::update(const system_observation_t& observation, const primal_solution_t& primalSolution,
                                                const command_data_t& command) {
  switched_model::QuadrupedXppVisualizer<12>::system_observation_t anymalBearObservation;
  anymalBearObservation.time() = observation.time();
  loopshapingDefinition_->getSystemState(observation.state(), anymalBearObservation.state());
  loopshapingDefinition_->getSystemInput(observation.state(), observation.input(), anymalBearObservation.input());
  anymalBearObservation.subsystem() = observation.subsystem();

  anymalBearDummyVisualization_->publishObservation(anymalBearObservation);
}

}  // namespace anymal
