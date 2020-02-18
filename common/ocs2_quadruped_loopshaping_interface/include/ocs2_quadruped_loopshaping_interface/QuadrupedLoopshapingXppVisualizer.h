//
// Created by rgrandia on 13.02.20.
//

#pragma once

// Loopshaping
#include <ocs2_core/loopshaping/Loopshaping.h>

#include <ocs2_quadruped_interface/QuadrupedXppVisualizer.h>

#include <ocs2_quadruped_loopshaping_interface/LoopshapingDimensions.h>

namespace switched_model_loopshaping {

class QuadrupedLoopshapingXppVisualizer : public ocs2::DummyObserver<STATE_DIM, INPUT_DIM> {
 public:
  QuadrupedLoopshapingXppVisualizer(std::shared_ptr<ocs2::LoopshapingDefinition> loopshapingDefinition,
                                    std::unique_ptr<switched_model::QuadrupedXppVisualizer> quadrupedXppVisualizer);

  void update(const system_observation_t& observation, const primal_solution_t& primalSolution, const command_data_t& command) override;

 private:
  std::shared_ptr<ocs2::LoopshapingDefinition> loopshapingDefinition_;
  std::unique_ptr<switched_model::QuadrupedXppVisualizer> quadrupedXppVisualizer_;
};

}  // namespace switched_model_loopshaping