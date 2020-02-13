//
// Created by rgrandia on 13.02.20.
//

#pragma once

// Loopshaping
#include <ocs2_core/loopshaping/Loopshaping.h>

#include <ocs2_quadruped_interface/QuadrupedXppVisualizer.h>

namespace anymal {

class AnymalBearLoopshapingVisualization : public ocs2::DummyObserver<48, 24> {
 public:
  AnymalBearLoopshapingVisualization(std::shared_ptr<ocs2::LoopshapingDefinition> loopshapingDefinition,
                                     std::unique_ptr<switched_model::QuadrupedXppVisualizer<12>> anymalBearDummyVisualization);

  void update(const system_observation_t& observation, const primal_solution_t& primalSolution, const command_data_t& command) override;

 private:
  std::shared_ptr<ocs2::LoopshapingDefinition> loopshapingDefinition_;
  std::unique_ptr<switched_model::QuadrupedXppVisualizer<12>> anymalBearDummyVisualization_;
};

}  // namespace anymal
