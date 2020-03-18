//
// Created by rgrandia on 18.03.20.
//

#include "ocs2_switched_model_interface/logic/SwitchedModelModeScheduleManager.h"

#include "ocs2_switched_model_interface/core/MotionPhaseDefinition.h"

namespace switched_model {

SwitchedModelModeScheduleManager::SwitchedModelModeScheduleManager(std::shared_ptr<GaitSchedule> gaitSchedulePtr)
    : Base(ocs2::ModeSchedule()), gaitSchedulePtr_(std::move(gaitSchedulePtr)) {}

contact_flag_t SwitchedModelModeScheduleManager::getContactFlags(scalar_t time) const {
  return modeNumber2StanceLeg(this->getModeSchedule().modeAtTime(time));
}

void SwitchedModelModeScheduleManager::preSolverRunImpl(scalar_t initTime, scalar_t finalTime, const state_vector_t& currentState,
                                                        const ocs2::CostDesiredTrajectories& costDesiredTrajectory,
                                                        ocs2::ModeSchedule& modeSchedule) {
  const auto timeHorizon = finalTime - initTime;
  modeSchedule = gaitSchedulePtr_->getModeSchedule(initTime - timeHorizon, finalTime + timeHorizon);
}

}  // namespace switched_model
