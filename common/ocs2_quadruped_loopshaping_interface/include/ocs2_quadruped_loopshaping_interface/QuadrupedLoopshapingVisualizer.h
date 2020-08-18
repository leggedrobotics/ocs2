//
// Created by rgrandia on 13.02.20.
//

#pragma once

// Loopshaping
#include <ocs2_core/loopshaping/Loopshaping.h>

#include <ocs2_quadruped_interface/QuadrupedVisualizer.h>

#include <ocs2_quadruped_loopshaping_interface/LoopshapingDimensions.h>

namespace switched_model_loopshaping {

class QuadrupedLoopshapingVisualizer : public ocs2::DummyObserver {
 public:
  QuadrupedLoopshapingVisualizer(std::shared_ptr<ocs2::LoopshapingDefinition> loopshapingDefinition,
                                 std::unique_ptr<switched_model::QuadrupedVisualizer> quadrupedVisualizer);

  void update(const ocs2::SystemObservation& observation, const ocs2::PrimalSolution& primalSolution,
              const ocs2::CommandData& command) override;

 private:
  std::shared_ptr<ocs2::LoopshapingDefinition> loopshapingDefinition_;
  std::unique_ptr<switched_model::QuadrupedVisualizer> quadrupedVisualizer_;
};

}  // namespace switched_model_loopshaping