//
// Created by rgrandia on 13.02.20.
//

#pragma once

// Loopshaping
#include <ocs2_core/loopshaping/Loopshaping.h>

#include <ocs2_quadruped_interface/QuadrupedXppVisualizer.h>

namespace switched_model {

class QuadrupedLoopshapingXppVisualizer : public ocs2::DummyObserver<48, 24> {
 public:
  QuadrupedLoopshapingXppVisualizer(std::shared_ptr<ocs2::LoopshapingDefinition> loopshapingDefinition,
                                    std::unique_ptr<QuadrupedXppVisualizer> quadrupedXppVisualizer);

  void update(const system_observation_t& observation, const primal_solution_t& primalSolution, const command_data_t& command) override;

 private:
  std::shared_ptr<ocs2::LoopshapingDefinition> loopshapingDefinition_;
  std::unique_ptr<QuadrupedXppVisualizer> quadrupedXppVisualizer_;
};

}  // namespace switched_model