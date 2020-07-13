//
// Created by rgrandia on 19.03.20.
//

#pragma once

#include <ocs2_core/loopshaping/LoopshapingDefinition.h>
#include <ocs2_switched_model_interface/core/SwitchedModel.h>
#include <ocs2_switched_model_interface/logic/SwitchedModelModeScheduleManager.h>

#include "ocs2_quadruped_loopshaping_interface/LoopshapingDimensions.h"

namespace switched_model_loopshaping {

class LoopshapingModeScheduleManager : public ocs2::ModeScheduleManager {
  using Base = ocs2::ModeScheduleManager;

 public:
  /** Mode schedule manager that is wrapped with loopshaping */
  std::shared_ptr<switched_model::SwitchedModelModeScheduleManager> modeScheduleManager_;

  LoopshapingModeScheduleManager(std::shared_ptr<switched_model::SwitchedModelModeScheduleManager> modeScheduleManager,
                                 std::shared_ptr<ocs2::LoopshapingDefinition> loopshapingDefinition);

  ~LoopshapingModeScheduleManager() override = default;

  void preSolverRunImpl(scalar_t initTime, scalar_t finalTime, const vector_t& currentState,
                        const ocs2::CostDesiredTrajectories& costDesiredTrajectory, ocs2::ModeSchedule& modeSchedule) override;

 private:
  std::shared_ptr<ocs2::LoopshapingDefinition> loopshapingDefinition_;
};

}  // namespace switched_model_loopshaping
