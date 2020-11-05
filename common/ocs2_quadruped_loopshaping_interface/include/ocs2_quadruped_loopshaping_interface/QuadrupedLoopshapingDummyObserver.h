//
// Created by rgrandia on 13.02.20.
//

#pragma once

// Loopshaping
#include <ocs2_core/loopshaping/Loopshaping.h>

#include <ocs2_ros_interfaces/mrt/DummyObserver.h>

#include <ocs2_quadruped_loopshaping_interface/LoopshapingDimensions.h>

namespace switched_model_loopshaping {

class QuadrupedLoopshapingDummyObserver : public ocs2::DummyObserver {
 public:
  QuadrupedLoopshapingDummyObserver(std::shared_ptr<ocs2::LoopshapingDefinition> loopshapingDefinition,
                                    std::vector<std::shared_ptr<DummyObserver>> observers);

  void update(const ocs2::SystemObservation& observation, const ocs2::PrimalSolution& primalSolution,
              const ocs2::CommandData& command) override;

 private:
  std::shared_ptr<ocs2::LoopshapingDefinition> loopshapingDefinition_;
  std::vector<std::shared_ptr<DummyObserver>> observers_;
};

}  // namespace switched_model_loopshaping