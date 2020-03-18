#include <ocs2_switched_model_interface/logic/SwitchedModelLogicRulesBase.h>

namespace switched_model {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SwitchedModelLogicRulesBase::SwitchedModelLogicRulesBase(std::shared_ptr<feet_planner_t> feetPlannerPtr, scalar_t phaseTransitionStanceTime)
    : feetPlannerPtr_(std::move(feetPlannerPtr)), phaseTransitionStanceTime_(phaseTransitionStanceTime) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SwitchedModelLogicRulesBase::SwitchedModelLogicRulesBase(const SwitchedModelLogicRulesBase& rhs)
    : feetPlannerPtr_(rhs.feetPlannerPtr_),
      phaseTransitionStanceTime_(rhs.phaseTransitionStanceTime_),
      feetReferencePtrStock_(rhs.feetReferencePtrStock_.size()),  // shallow copy: points to the same asset
      feetReferenceUpdatedStock_(rhs.feetReferenceUpdatedStock_.size(), false) {}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SwitchedModelLogicRulesBase::getMotionPhaseLogics(size_t index, const ocs2::ModeSchedule& modeSchedule, contact_flag_t& contactFlags,
                                                       std::array<const foot_cpg_t*, 4>& feetReferencePtr) {
  contactFlags = getContactFlags(index, modeSchedule);

  // plan feetReferencePtrStock_[index] if it is not yet updated
  std::lock_guard<std::mutex> lock(feetReferenceUpdateMutex_);
  if (!feetReferenceUpdatedStock_[index]) {
    feetReferencePtrStock_[index] = feetPlannerPtr_->planSingleMode(index, modeSchedule);
    feetReferenceUpdatedStock_[index] = true;
  }

  for (size_t i = 0; i < feetReferencePtrStock_[index].size(); i++) {
    feetReferencePtr[i] = feetReferencePtrStock_[index][i].get();
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
const typename SwitchedModelLogicRulesBase::feet_planner_t& SwitchedModelLogicRulesBase::getFeetPlanner() const {
  return *feetPlannerPtr_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SwitchedModelLogicRulesBase::insertModeSequenceTemplate(const logic_template_type& modeSequenceTemplate, const scalar_t& startTime,
                                                             const scalar_t& finalTime) {
  // find the index on which the new gait should be added
  const size_t index = std::lower_bound(eventTimes().begin(), eventTimes().end(), startTime) - eventTimes().begin();

  // delete the old logic from the index
  if (index < eventTimes().size()) {
    eventTimes().erase(eventTimes().begin() + index, eventTimes().end());
    subsystemsSequence().erase(subsystemsSequence().begin() + index + 1, subsystemsSequence().end());
  }

  // add an intermediate stance phase
  scalar_t phaseTransitionStanceTime = phaseTransitionStanceTime_;
  if (subsystemsSequence().size() > 0 && subsystemsSequence().back() == ModeNumber::STANCE) phaseTransitionStanceTime = 0.0;

  if (phaseTransitionStanceTime > 0.001) {
    eventTimes().push_back(startTime);
    subsystemsSequence().push_back(ModeNumber::STANCE);
  }

  // tile the mode sequence template from startTime+phaseTransitionStanceTime to finalTime.
  tileModeSequenceTemplate(modeSequenceTemplate, startTime + phaseTransitionStanceTime, finalTime);

  // update the internal variables
  update();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SwitchedModelLogicRulesBase::rewind(const scalar_t& lowerBoundTime, const scalar_t& upperBoundTime) {
  const size_t index = std::lower_bound(eventTimes().begin(), eventTimes().end(), lowerBoundTime) - eventTimes().begin();

  if (index > 0) {
    // delete the old logic from index and set the default start phase to stance
    eventTimes().erase(eventTimes().begin(), eventTimes().begin() + index - 1);  // keep the one before the last to make it stance
    subsystemsSequence().erase(subsystemsSequence().begin(), subsystemsSequence().begin() + index - 1);

    // set the default initial phase
    subsystemsSequence().front() = ModeNumber::STANCE;
  }

  // tiling start time
  scalar_t tilingStartTime = eventTimes().back();

  // delete the last default stance phase
  eventTimes().erase(eventTimes().end() - 1, eventTimes().end());
  subsystemsSequence().erase(subsystemsSequence().end() - 1, subsystemsSequence().end());

  // tile the template logic
  tileModeSequenceTemplate(modeSequenceTemplate(), tilingStartTime, upperBoundTime);

  // update the internal variables
  update();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SwitchedModelLogicRulesBase::tileModeSequenceTemplate(const logic_template_type& modeSequenceTemplate, const scalar_t& startTime,
                                                           const scalar_t& finalTime) {
  const size_t numTemplateSubsystems = modeSequenceTemplate.templateSubsystemsSequence_.size();

  // If no template subsystem is defined, the last subsystem should continue for ever
  if (numTemplateSubsystems == 0) {
    return;
  }

  if (modeSequenceTemplate.templateSwitchingTimes_.size() != numTemplateSubsystems + 1) {
    throw std::runtime_error(
        "The number of the subsystems in the user-defined template should be equal to "
        "the number of the template switching times minus 1.");
  }

  if (!eventTimes().empty() && startTime <= eventTimes().back()) {
    throw std::runtime_error("The initial time for template-tiling is not greater than the last event time.");
  }

  // add a initial time
  eventTimes().push_back(startTime);

  // concatenate from index
  while (eventTimes().back() < finalTime) {
    for (size_t i = 0; i < modeSequenceTemplate.templateSubsystemsSequence_.size(); i++) {
      subsystemsSequence().push_back(modeSequenceTemplate.templateSubsystemsSequence_[i]);
      scalar_t deltaTime = modeSequenceTemplate.templateSwitchingTimes_[i + 1] - modeSequenceTemplate.templateSwitchingTimes_[i];
      eventTimes().push_back(eventTimes().back() + deltaTime);
    }  // end of i loop

  }  // end of while loop

  // default final phase
  subsystemsSequence().push_back(ModeNumber::STANCE);
}

}  // namespace switched_model
