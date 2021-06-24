#include <ocs2_legged_robot_example/initialization/LeggedRobotInitializer.h>

#include <ocs2_core/misc/LinearInterpolation.h>
#include <ocs2_legged_robot_example/common/utils.h>

namespace ocs2 {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LeggedRobotInitializer::LeggedRobotInitializer(const SwitchedModelModeScheduleManager& modeScheduleManager)
    : modeScheduleManagerPtr_(&modeScheduleManager) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LeggedRobotInitializer* LeggedRobotInitializer::clone() const {
  return new LeggedRobotInitializer(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LeggedRobotInitializer::compute(scalar_t time, const vector_t& state, scalar_t nextTime, vector_t& input, vector_t& nextState) {
  const auto contactFlags = modeScheduleManagerPtr_->getContactFlags(time);
  input = weightCompensatingInputs(ROBOT_TOTAL_MASS_, contactFlags);
  nextState = state;
  nextState.head<ocs2::legged_robot::BASE_DOF_NUM_>() = vector_t::Zero(ocs2::legged_robot::BASE_DOF_NUM_);
}

}  // namespace legged_robot
}  // namespace ocs2
