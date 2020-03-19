//
// Created by rgrandia on 19.03.20.
//

#include "ocs2_quadruped_loopshaping_interface/LoopshapingModeScheduleManager.h"

namespace switched_model_loopshaping {

LoopshapingModeScheduleManager::LoopshapingModeScheduleManager(std::shared_ptr<switched_mode_schedule_manager_t> modeScheduleManager,
                                                               std::shared_ptr<ocs2::LoopshapingDefinition> loopshapingDefinition)
    : Base(modeScheduleManager->getModeSchedule()),
      modeScheduleManager_(std::move(modeScheduleManager)),
      loopshapingDefinition_(std::move(loopshapingDefinition)) {}

void LoopshapingModeScheduleManager::preSolverRunImpl(scalar_t initTime, scalar_t finalTime, const state_vector_t& currentState,
                                                      const ocs2::CostDesiredTrajectories& costDesiredTrajectory,
                                                      ocs2::ModeSchedule& modeSchedule) {
  switched_mode_schedule_manager_t::state_vector_t systemState;
  loopshapingDefinition_->getSystemState(currentState, systemState);
  modeScheduleManager_->preSolverRun(initTime, finalTime, systemState, costDesiredTrajectory);
  modeSchedule = modeScheduleManager_->getModeSchedule();
}

}  // namespace switched_model_loopshaping
