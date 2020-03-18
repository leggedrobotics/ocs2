#include <ocs2_switched_model_interface/logic/GaitSchedule.h>

#include <ocs2_core/misc/Lookup.h>

namespace switched_model {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
GaitSchedule::GaitSchedule(ocs2::ModeSchedule initModeSchedule, scalar_t phaseTransitionStanceTime)
    : modeSchedule_(std::move(initModeSchedule)), phaseTransitionStanceTime_(phaseTransitionStanceTime) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaitSchedule::insertModeSequenceTemplate(const ModeSequenceTemplate& modeSequenceTemplate, scalar_t startTime,
                                                             scalar_t finalTime) {
  modeSequenceTemplate_ = modeSequenceTemplate;
  auto& eventTimes = modeSchedule_.eventTimes;
  auto& modeSequence = modeSchedule_.modeSequence;

  // find the index on which the new gait should be added
  const size_t index = std::lower_bound(eventTimes.begin(), eventTimes.end(), startTime) - eventTimes.begin();

  // delete the old logic from the index
  if (index < eventTimes.size()) {
    eventTimes.erase(eventTimes.begin() + index, eventTimes.end());
    modeSequence.erase(modeSequence.begin() + index + 1, modeSequence.end());
  }

  // add an intermediate stance phase
  scalar_t phaseTransitionStanceTime = phaseTransitionStanceTime_;
  if (!modeSequence.empty() && modeSequence.back() == ModeNumber::STANCE) {
    phaseTransitionStanceTime = 0.0;
  }

  const double magicNumber = 0.001;
  if (phaseTransitionStanceTime > magicNumber) {
    eventTimes.push_back(startTime);
    modeSequence.push_back(ModeNumber::STANCE);
  }

  // tile the mode sequence template from startTime+phaseTransitionStanceTime to finalTime.
  tileModeSequenceTemplate(startTime + phaseTransitionStanceTime, finalTime);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ocs2::ModeSchedule GaitSchedule::getModeSchedule(scalar_t lowerBoundTime, scalar_t upperBoundTime) {
  auto& eventTimes = modeSchedule_.eventTimes;
  auto& modeSequence = modeSchedule_.modeSequence;
  const size_t index = std::lower_bound(eventTimes.begin(), eventTimes.end(), lowerBoundTime) - eventTimes.begin();

  if (index > 0) {
    // delete the old logic from index and set the default start phase to stance
    eventTimes.erase(eventTimes.begin(), eventTimes.begin() + index - 1);  // keep the one before the last to make it stance
    modeSequence.erase(modeSequence.begin(), modeSequence.begin() + index - 1);

    // set the default initial phase
    modeSequence.front() = ModeNumber::STANCE;
  }

  // tiling start time
  scalar_t tilingStartTime = eventTimes.back();

  // delete the last default stance phase
  eventTimes.erase(eventTimes.end() - 1, eventTimes.end());
  modeSequence.erase(modeSequence.end() - 1, modeSequence.end());

  // tile the template logic
  tileModeSequenceTemplate(tilingStartTime, upperBoundTime);
  return modeSchedule_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaitSchedule::tileModeSequenceTemplate(scalar_t startTime, scalar_t finalTime) {
  auto& eventTimes = modeSchedule_.eventTimes;
  auto& modeSequence = modeSchedule_.modeSequence;
  const auto& templateTimes = modeSequenceTemplate_.switchingTimes;
  const auto& templateModeSequence = modeSequenceTemplate_.modeSequence;
  const size_t numTemplateSubsystems = modeSequenceTemplate_.modeSequence.size();

  // If no template subsystem is defined, the last subsystem should continue for ever
  if (numTemplateSubsystems == 0) {
    return;
  }

  if (templateModeSequence.size() != templateModeSequence.size() + 1) {
    throw std::runtime_error(
        "The number of the subsystems in the user-defined template should be equal to "
        "the number of the template switching times minus 1.");
  }

  if (!eventTimes.empty() && startTime <= eventTimes.back()) {
    throw std::runtime_error("The initial time for template-tiling is not greater than the last event time.");
  }

  // add a initial time
  eventTimes.push_back(startTime);

  // concatenate from index
  while (eventTimes.back() < finalTime) {
    for (size_t i = 0; i < templateModeSequence.size(); i++) {
      modeSequence.push_back(templateModeSequence[i]);
      scalar_t deltaTime = templateTimes[i + 1] - templateTimes[i];
      eventTimes.push_back(eventTimes.back() + deltaTime);
    }  // end of i loop
  }    // end of while loop

  // default final phase
  modeSequence.push_back(ModeNumber::STANCE);
}

}  // namespace switched_model
