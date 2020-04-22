//
// Created by rgrandia on 18.03.20.
//

#include "ocs2_switched_model_interface/logic/SwitchedModelModeScheduleManager.h"

#include "ocs2_switched_model_interface/core/MotionPhaseDefinition.h"

namespace switched_model {

SwitchedModelModeScheduleManager::SwitchedModelModeScheduleManager(GaitSchedule gaitSchedule, SwingTrajectoryPlanner swingTrajectory)
    : Base(ocs2::ModeSchedule()),
      gaitSchedulePtr_(std::make_shared<LockableGaitSchedule>(std::move(gaitSchedule))),
      swingTrajectoryPtr_(std::make_shared<SwingTrajectoryPlanner>(std::move(swingTrajectory))),
      terrainPtr_(std::make_shared<ocs2::Lockable<TerrainPlane>>(TerrainPlane())) {}

contact_flag_t SwitchedModelModeScheduleManager::getContactFlags(scalar_t time) const {
  return modeNumber2StanceLeg(this->getModeSchedule().modeAtTime(time));
}

void SwitchedModelModeScheduleManager::preSolverRunImpl(scalar_t initTime, scalar_t finalTime, const state_vector_t& currentState,
                                                        const ocs2::CostDesiredTrajectories& costDesiredTrajectory,
                                                        ocs2::ModeSchedule& modeSchedule) {
  const auto timeHorizon = finalTime - initTime;
  {
    std::lock_guard<LockableGaitSchedule> lock(*gaitSchedulePtr_);
    gaitSchedulePtr_->advanceToTime(initTime);
    modeSchedule = gaitSchedulePtr_->getModeSchedule(2.0 * timeHorizon);
  }

  {
    std::lock_guard<ocs2::Lockable<TerrainPlane>> lock(*terrainPtr_);
    swingTrajectoryPtr_->update(initTime, finalTime, currentState, modeSchedule, *terrainPtr_);
  }
}

}  // namespace switched_model
