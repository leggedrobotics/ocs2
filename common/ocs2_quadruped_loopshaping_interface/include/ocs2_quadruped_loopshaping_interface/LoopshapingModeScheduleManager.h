//
// Created by rgrandia on 19.03.20.
//

#pragma once

#include <ocs2_core/loopshaping/LoopshapingDefinition.h>
#include <ocs2_switched_model_interface/core/SwitchedModel.h>
#include <ocs2_switched_model_interface/logic/SwitchedModelModeScheduleManager.h>

#include "ocs2_quadruped_loopshaping_interface/LoopshapingDimensions.h"

namespace switched_model_loopshaping {

class LoopshapingModeScheduleManager : public ocs2::ModeScheduleManager<STATE_DIM, INPUT_DIM> {
  using Base = ocs2::ModeScheduleManager<STATE_DIM, INPUT_DIM>;

 public:
  using switched_mode_schedule_manager_t = switched_model::SwitchedModelModeScheduleManager;

  /** Mode schedule manager that is wrapped with loopshaping */
  std::shared_ptr<switched_mode_schedule_manager_t> modeScheduleManager_;

  LoopshapingModeScheduleManager(std::shared_ptr<switched_mode_schedule_manager_t> modeScheduleManager,
                                 std::shared_ptr<ocs2::LoopshapingDefinition> loopshapingDefinition);

  ~LoopshapingModeScheduleManager() override = default;

  void preSolverRunImpl(scalar_t initTime, scalar_t finalTime, const state_vector_t& currentState,
                        const ocs2::CostDesiredTrajectories& costDesiredTrajectory, ocs2::ModeSchedule& modeSchedule) override;

 private:
  std::shared_ptr<ocs2::LoopshapingDefinition> loopshapingDefinition_;
};

}  // namespace switched_model_loopshaping
