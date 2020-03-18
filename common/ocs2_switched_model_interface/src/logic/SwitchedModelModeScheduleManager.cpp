//
// Created by rgrandia on 18.03.20.
//

#include "ocs2_switched_model_interface/logic/SwitchedModelModeScheduleManager.h"

#include "ocs2_switched_model_interface/core/MotionPhaseDefinition.h"

namespace switched_model {

SwitchedModelModeScheduleManager::SwitchedModelModeScheduleManager(ocs2::ModeSchedule modeSchedule) : Base(std::move(modeSchedule)),
logicRulesPtr_(new SwitchedModelLogicRulesBase()){}

contact_flag_t SwitchedModelModeScheduleManager::getContactFlags(scalar_t time) const {
  return modeNumber2StanceLeg(this->getModeSchedule().modeAtTime(time));
}

void SwitchedModelModeScheduleManager::preSolverRunImpl(scalar_t initTime, scalar_t finalTime, const state_vector_t& currentState,
                                                        const ocs2::CostDesiredTrajectories& costDesiredTrajectory,
                                                        ocs2::ModeSchedule& modeSchedule) {
  // rewind logic rules
  if (modeSchedule.eventTimes.back() < finalTime) {
    const auto timeHorizon = finalTime - initTime;
    const auto gaitDuration = modeSequenceTemplate_.switchingTimes.back() - modeSequenceTemplate_.switchingTimes.front();
    logicRulesPtr_->rewind(initTime - gaitDuration, finalTime + timeHorizon, modeSequenceTemplate_, modeSchedule);
  }
}

}  // namespace switched_model
