#include <ocs2_legged_robot_example/logic/SwitchedModelModeScheduleManager.h>

namespace ocs2 {
namespace legged_robot {

SwitchedModelModeScheduleManager::SwitchedModelModeScheduleManager(std::shared_ptr<GaitSchedule> gaitSchedulePtr,
                                                                   std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr)
    : Base(ocs2::ModeSchedule()), gaitSchedulePtr_(std::move(gaitSchedulePtr)), swingTrajectoryPtr_(std::move(swingTrajectoryPtr)) {}

SwitchedModelModeScheduleManager::SwitchedModelModeScheduleManager(const SwitchedModelModeScheduleManager& rhs)
    : Base(ocs2::ModeSchedule()), gaitSchedulePtr_(rhs.gaitSchedulePtr_), swingTrajectoryPtr_(rhs.swingTrajectoryPtr_) {}

contact_flag_t SwitchedModelModeScheduleManager::getContactFlags(scalar_t time) const {
  return modeNumber2StanceLeg(this->getModeSchedule().modeAtTime(time));
}

void SwitchedModelModeScheduleManager::preSolverRunImpl(scalar_t initTime, scalar_t finalTime, const vector_t& currentState,
                                                        const ocs2::CostDesiredTrajectories& costDesiredTrajectory,
                                                        ocs2::ModeSchedule& modeSchedule) {
  const auto timeHorizon = finalTime - initTime;
  modeSchedule = gaitSchedulePtr_->getModeSchedule(initTime - timeHorizon, finalTime + timeHorizon);

  const scalar_t terrainHeight = 0.0;
  swingTrajectoryPtr_->update(modeSchedule, terrainHeight);
}

}  // namespace legged_robot
}  // namespace ocs2
