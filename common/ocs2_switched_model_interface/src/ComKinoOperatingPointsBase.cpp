#include "ocs2_switched_model_interface/initialization/ComKinoOperatingPointsBase.h"

#include "ocs2_switched_model_interface/core/MotionPhaseDefinition.h"

namespace switched_model {

ComKinoOperatingPointsBase::ComKinoOperatingPointsBase(const com_model_t& comModel,
                                                       std::shared_ptr<const SwitchedModelModeScheduleManager> modeScheduleManagerPtr)
    : comModelPtr_(comModel.clone()), modeScheduleManagerPtr_(std::move(modeScheduleManagerPtr)) {
  if (!modeScheduleManagerPtr_) {
    throw std::runtime_error("[ComKinoOperatingPointsBase] Mode schedule manager cannot be a nullptr");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ComKinoOperatingPointsBase::ComKinoOperatingPointsBase(const ComKinoOperatingPointsBase& rhs)
    : Base(rhs), comModelPtr_(rhs.comModelPtr_->clone()), modeScheduleManagerPtr_(rhs.modeScheduleManagerPtr_) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ComKinoOperatingPointsBase* ComKinoOperatingPointsBase::clone() const {
  return new ComKinoOperatingPointsBase(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
input_vector_t ComKinoOperatingPointsBase::computeInputOperatingPoints(contact_flag_t contactFlags) const {
  // Distribute total mass equally over active stance legs.
  input_vector_t inputs = input_vector_t::Zero();

  const scalar_t totalMass = comModelPtr_->totalMass() * 9.81;
  size_t numStanceLegs(0);

  for (size_t i = 0; i < NUM_CONTACT_POINTS; i++) {
    if (contactFlags[i]) {
      ++numStanceLegs;
    }
  }

  if (numStanceLegs > 0) {
    for (size_t i = 0; i < NUM_CONTACT_POINTS; i++) {
      if (contactFlags[i]) {
        inputs(3 * i + 2) = totalMass / numStanceLegs;
      }
    }
  }

  return inputs;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void ComKinoOperatingPointsBase::getSystemOperatingTrajectories(const vector_t& initialState, scalar_t startTime, scalar_t finalTime,
                                                                scalar_array_t& timeTrajectory, vector_array_t& stateTrajectory,
                                                                vector_array_t& inputTrajectory, bool concatOutput) {
  const auto midTime = 0.5 * (startTime + finalTime);
  const auto contactFlags = modeScheduleManagerPtr_->getContactFlags(midTime);
  const auto inputOperatingPoint = computeInputOperatingPoints(contactFlags);

  if (!concatOutput) {
    timeTrajectory.clear();
    stateTrajectory.clear();
    inputTrajectory.clear();
  }

  timeTrajectory.push_back(startTime);
  timeTrajectory.push_back(finalTime);

  stateTrajectory.push_back(initialState);
  stateTrajectory.push_back(initialState);

  inputTrajectory.push_back(inputOperatingPoint);
  inputTrajectory.push_back(inputOperatingPoint);
}

}  // namespace switched_model
