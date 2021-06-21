#include <ocs2_legged_robot_example/initialization/LeggedRobotInitializer.h>

#include <ocs2_core/misc/LinearInterpolation.h>

namespace ocs2 {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LeggedRobotInitializer::LeggedRobotInitializer(std::shared_ptr<const SwitchedModelModeScheduleManager> modeScheduleManagerPtr)
    : Base(), modeScheduleManagerPtr_(std::move(modeScheduleManagerPtr)) {
  if (!modeScheduleManagerPtr_) {
    throw std::runtime_error("[LeggedRobotInitializer] ModeScheduleManager cannot be a nullptr");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LeggedRobotInitializer::LeggedRobotInitializer(const LeggedRobotInitializer& rhs)
    : Base(rhs), modeScheduleManagerPtr_(rhs.modeScheduleManagerPtr_) {}

LeggedRobotInitializer* LeggedRobotInitializer::clone() const {
  return new LeggedRobotInitializer(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LeggedRobotInitializer::compute(scalar_t time, const vector_t& state, scalar_t nextTime, vector_t& input, vector_t& nextState) {
  const auto contactFlags = modeScheduleManagerPtr_->getContactFlags(time);

  input = computeInputOperatingPoints(contactFlags, state);

  nextState = state;
  nextState.head<ocs2::legged_robot::BASE_DOF_NUM_>() = vector_t::Zero(ocs2::legged_robot::BASE_DOF_NUM_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
input_vector_t LeggedRobotInitializer::computeInputOperatingPoints(const contact_flag_t& contactFlags,
                                                                   const state_vector_t& nominalState) const {
  // Distribute total mass equally over active stance legs.
  input_vector_t inputs = input_vector_t::Zero();

  const scalar_t totalWeight = ROBOT_TOTAL_MASS_ * 9.81;
  size_t numStanceLegs(0);

  for (size_t i = 0; i < FOOT_CONTACTS_NUM_; i++) {
    if (contactFlags[i]) {
      ++numStanceLegs;
    }
  }

  if (numStanceLegs > 0) {
    for (size_t i = 0; i < FOOT_CONTACTS_NUM_; i++) {
      if (contactFlags[i]) {
        inputs(3 * i + 2) = totalWeight / numStanceLegs;
      }
    }
  }

  return inputs;
}

}  // namespace legged_robot
}  // namespace ocs2
