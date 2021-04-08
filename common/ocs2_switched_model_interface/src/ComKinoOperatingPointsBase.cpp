#include <ocs2_core/misc/LinearInterpolation.h>

#include "ocs2_switched_model_interface/initialization/ComKinoOperatingPointsBase.h"

#include "ocs2_switched_model_interface/core/Rotations.h"

#include "ocs2_switched_model_interface/core/MotionPhaseDefinition.h"

namespace switched_model {

ComKinoOperatingPointsBase::ComKinoOperatingPointsBase(const com_model_t& comModel,
                                                       const SwitchedModelModeScheduleManager& modeScheduleManager)
    : comModelPtr_(comModel.clone()), modeScheduleManagerPtr_(&modeScheduleManager) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ComKinoOperatingPointsBase::ComKinoOperatingPointsBase(const ComKinoOperatingPointsBase& rhs)
    : ocs2::SystemOperatingTrajectoriesBase(rhs),
      comModelPtr_(rhs.comModelPtr_->clone()),
      modeScheduleManagerPtr_(rhs.modeScheduleManagerPtr_) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ComKinoOperatingPointsBase* ComKinoOperatingPointsBase::clone() const {
  return new ComKinoOperatingPointsBase(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
input_vector_t ComKinoOperatingPointsBase::computeInputOperatingPoints(const contact_flag_t& contactFlags,
                                                                       const state_vector_t& nominalState) const {
  return weightCompensatingInputs(*comModelPtr_, contactFlags, getOrientation(getComPose(nominalState)));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void ComKinoOperatingPointsBase::getSystemOperatingTrajectories(const vector_t& initialState, scalar_t startTime, scalar_t finalTime,
                                                                scalar_array_t& timeTrajectory, vector_array_t& stateTrajectory,
                                                                vector_array_t& inputTrajectory, bool concatOutput) {
  const auto midTime = 0.5 * (startTime + finalTime);
  const auto contactFlags = modeScheduleManagerPtr_->getContactFlags(midTime);

  vector_t state = vector_t::Zero(STATE_DIM);
  if (!timeTrajectory.empty()) {
    state = ocs2::LinearInterpolation::interpolate(midTime, timeTrajectory, stateTrajectory);
  }
  const auto inputOperatingPoint = computeInputOperatingPoints(contactFlags, state);

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
