/*
 * SwitchedModelLogicRulesBase.h
 *
 *  Created on: Mar 14, 2018
 *      Author: Farbod
 */

namespace switched_model {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, class cpg_t, size_t STATE_DIM, size_t INPUT_DIM>
SwitchedModelLogicRulesBase<JOINT_COORD_SIZE,cpg_t,STATE_DIM,INPUT_DIM>::SwitchedModelLogicRulesBase(
		const feet_planner_ptr_t& feetPlannerPtr)

	: BASE()
	, feetPlannerPtr_(feetPlannerPtr)  // shallow copy: points to the same asset
{}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, class cpg_t, size_t STATE_DIM, size_t INPUT_DIM>
SwitchedModelLogicRulesBase<JOINT_COORD_SIZE,cpg_t,STATE_DIM,INPUT_DIM>::SwitchedModelLogicRulesBase(
		const SwitchedModelLogicRulesBase& rhs)

	: BASE(rhs)
	, feetPlannerPtr_(rhs.feetPlannerPtr_)
	, contactFlagsStock_(rhs.contactFlagsStock_)
	, feetReferencePtrStock_(rhs.feetReferencePtrStock_.size())	// shallow copy: points to the same asset
	, feetReferenceUpdatedStock_(rhs.feetReferenceUpdatedStock_.size(), false)
	, endEffectorStateConstraints_(rhs.endEffectorStateConstraints_)
{}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, class cpg_t, size_t STATE_DIM, size_t INPUT_DIM>
SwitchedModelLogicRulesBase<JOINT_COORD_SIZE,cpg_t,STATE_DIM,INPUT_DIM>&
	SwitchedModelLogicRulesBase<JOINT_COORD_SIZE,cpg_t,STATE_DIM,INPUT_DIM>::operator=(
		SwitchedModelLogicRulesBase&& other) {

	if (this != &other) {
		// base class
		BASE::operator=(std::move(other));

		feetPlannerPtr_ 		= std::move(other.feetPlannerPtr_);
		contactFlagsStock_ 		= std::move(other.contactFlagsStock_);
		feetReferencePtrStock_	= std::move(other.feetReferencePtrStock_);
		feetReferenceUpdatedStock_ 	 = std::move(other.feetReferenceUpdatedStock_);
		endEffectorStateConstraints_ = std::move(other.endEffectorStateConstraints_);
	}

	return *this;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, class cpg_t, size_t STATE_DIM, size_t INPUT_DIM>
SwitchedModelLogicRulesBase<JOINT_COORD_SIZE,cpg_t,STATE_DIM,INPUT_DIM>&
	SwitchedModelLogicRulesBase<JOINT_COORD_SIZE,cpg_t,STATE_DIM,INPUT_DIM>::operator=(
		const SwitchedModelLogicRulesBase& other) {

	if (this != &other) {
		// base class
		BASE::operator=(other);

		feetPlannerPtr_				= other.feetPlannerPtr_;
		contactFlagsStock_ 			= other.contactFlagsStock_;
		feetReferencePtrStock_		= other.feetReferencePtrStock_;
		feetReferenceUpdatedStock_ 	= other.feetReferenceUpdatedStock_;
		endEffectorStateConstraints_= other.endEffectorStateConstraints_;
	}

	return *this;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, class cpg_t, size_t STATE_DIM, size_t INPUT_DIM>
void SwitchedModelLogicRulesBase<JOINT_COORD_SIZE,cpg_t,STATE_DIM,INPUT_DIM>::setMotionConstraints(
		const size_array_t& subsystemsSequence,
		const scalar_array_t& eventTimes,
		const std::vector<EndEffectorConstraintBase::ConstPtr>& endEffectorStateConstraints /*
		= std::vector<EndEffectorConstraintBase::ConstPtr>()*/) {

	endEffectorStateConstraints_ = endEffectorStateConstraints;

	// this will also class the update method
	BASE::setModeSequence(subsystemsSequence, eventTimes);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, class cpg_t, size_t STATE_DIM, size_t INPUT_DIM>
void SwitchedModelLogicRulesBase<JOINT_COORD_SIZE,cpg_t,STATE_DIM,INPUT_DIM>::update() {

	const size_t numSubsystems = BASE::subsystemsSequence_.size();

	contactFlagsStock_.resize(numSubsystems);
	for (size_t i=0; i<numSubsystems; i++)
		contactFlagsStock_[i] = modeNumber2StanceLeg(BASE::subsystemsSequence_[i]);

	feetReferencePtrStock_.resize(numSubsystems);
	feetReferenceUpdatedStock_.resize(numSubsystems);
	for (size_t i=0; i<numSubsystems; i++)
		feetReferenceUpdatedStock_[i] = false;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, class cpg_t, size_t STATE_DIM, size_t INPUT_DIM>
const std::vector<typename SwitchedModelLogicRulesBase<JOINT_COORD_SIZE,cpg_t,STATE_DIM,INPUT_DIM>::contact_flag_t>&
	SwitchedModelLogicRulesBase<JOINT_COORD_SIZE,cpg_t,STATE_DIM,INPUT_DIM>::getContactFlagsSequence() const {

	return contactFlagsStock_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, class cpg_t, size_t STATE_DIM, size_t INPUT_DIM>
void SwitchedModelLogicRulesBase<JOINT_COORD_SIZE,cpg_t,STATE_DIM,INPUT_DIM>::getContactFlags(
		const size_t& index,
		contact_flag_t& contactFlags) const {

	if (index >= contactFlagsStock_.size())
		throw std::runtime_error("The requested index refers to an out-of-bound motion phase.");

	contactFlags = contactFlagsStock_[index];
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, class cpg_t, size_t STATE_DIM, size_t INPUT_DIM>
void SwitchedModelLogicRulesBase<JOINT_COORD_SIZE,cpg_t,STATE_DIM,INPUT_DIM>::getMotionPhaseLogics(
		const size_t& index,
		contact_flag_t& contactFlags,
		std::array<const cpg_t*, 4>& feetReferencePtr) const {

	if (index >= contactFlagsStock_.size())
		throw std::runtime_error("The requested index refers to an out-of-bound motion phase.");

	contactFlags = contactFlagsStock_[index];

	// plan feetReferencePtrStock_[index] if it is not yet updated
	if (feetReferenceUpdatedStock_[index] == false) {
		std::lock_guard<std::mutex> lock(feetReferenceUpdateMutex_);	// avoids racing

		if (feetReferenceUpdatedStock_[index] == false) {
			feetPlannerPtr_->planSingleMode(index, BASE::subsystemsSequence_, BASE::eventTimes_, feetReferencePtrStock_[index]);
			feetReferenceUpdatedStock_[index] = true;
		}
	}

	for (size_t i=0; i<feetReferencePtrStock_[index].size(); i++)
		feetReferencePtr[i] = feetReferencePtrStock_[index][i].get();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, class cpg_t, size_t STATE_DIM, size_t INPUT_DIM>
const std::vector<EndEffectorConstraintBase::ConstPtr>*
	SwitchedModelLogicRulesBase<JOINT_COORD_SIZE,cpg_t,STATE_DIM,INPUT_DIM>::getEndEffectorStateConstraintsPtr() const {

	return &endEffectorStateConstraints_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, class cpg_t, size_t STATE_DIM, size_t INPUT_DIM>
typename SwitchedModelLogicRulesBase<JOINT_COORD_SIZE,cpg_t,STATE_DIM,INPUT_DIM>::feet_planner_t&
	SwitchedModelLogicRulesBase<JOINT_COORD_SIZE,cpg_t,STATE_DIM,INPUT_DIM>::getFeetPlanner() {

	return *feetPlannerPtr_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, class cpg_t, size_t STATE_DIM, size_t INPUT_DIM>
void SwitchedModelLogicRulesBase<JOINT_COORD_SIZE,cpg_t,STATE_DIM,INPUT_DIM>::setModeSequenceTemplate(
		const logic_template_type& modeSequenceTemplate) {

	modeSequenceTemplate_ = modeSequenceTemplate;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, class cpg_t, size_t STATE_DIM, size_t INPUT_DIM>
void SwitchedModelLogicRulesBase<JOINT_COORD_SIZE,cpg_t,STATE_DIM,INPUT_DIM>::insertModeSequenceTemplate(
		const scalar_t& startTime,
		const scalar_t& finalTime) {

	// find the index on which the new gait should be added
	const size_t index =
			std::lower_bound(BASE::eventTimes_.begin(), BASE::eventTimes_.end(), startTime) - BASE::eventTimes_.begin();

	// delete the old logic from the index
	if (index < BASE::eventTimes_.size()) {
		BASE::eventTimes_.erase(BASE::eventTimes_.begin()+index, BASE::eventTimes_.end());
		BASE::subsystemsSequence_.erase(BASE::subsystemsSequence_.begin()+index+1, BASE::subsystemsSequence_.end());
	}

	// add an intermediate stance phase
	scalar_t phaseTransitionStanceTime = 0.4;
	if (BASE::subsystemsSequence_.size()>0 && BASE::subsystemsSequence_.back()==ModeNumber::STANCE)
		phaseTransitionStanceTime = 0.0;

	if (phaseTransitionStanceTime > 0.0) {
		BASE::eventTimes_.push_back(startTime);
		BASE::subsystemsSequence_.push_back(ModeNumber::STANCE);
	}

	// tile the mode sequence template from startTime+phaseTransitionStanceTime to finalTime.
	tileModeSequenceTemplate(startTime+phaseTransitionStanceTime, finalTime);

	// update the internal variables
	update();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, class cpg_t, size_t STATE_DIM, size_t INPUT_DIM>
void SwitchedModelLogicRulesBase<JOINT_COORD_SIZE,cpg_t,STATE_DIM,INPUT_DIM>::rewind(
		const scalar_t& lowerBoundTime,
		const scalar_t& upperBoundTime) {

	const size_t index =
			std::lower_bound(BASE::eventTimes_.begin(), BASE::eventTimes_.end(), lowerBoundTime) - BASE::eventTimes_.begin();

	if (index > 0) {
		// delete the old logic from index and set the default start phase to stance
		BASE::eventTimes_.erase(BASE::eventTimes_.begin(), BASE::eventTimes_.begin()+index-1);  // keep the one before the last to make it stance
		BASE::subsystemsSequence_.erase(BASE::subsystemsSequence_.begin(), BASE::subsystemsSequence_.begin()+index-1);

		// set the default initial phase
		BASE::subsystemsSequence_.front() = ModeNumber::STANCE;
	}

	// tiling start time
	scalar_t tilingStartTime = BASE::eventTimes_.back();

	// delete the last default stance phase
	BASE::eventTimes_.erase(BASE::eventTimes_.end()-1, BASE::eventTimes_.end());
	BASE::subsystemsSequence_.erase(BASE::subsystemsSequence_.end()-1, BASE::subsystemsSequence_.end());

	// tile the template logic
	tileModeSequenceTemplate(tilingStartTime, upperBoundTime);

	// update the internal variables
	update();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, class cpg_t, size_t STATE_DIM, size_t INPUT_DIM>
void SwitchedModelLogicRulesBase<JOINT_COORD_SIZE,cpg_t,STATE_DIM,INPUT_DIM>::tileModeSequenceTemplate(
		const scalar_t& startTime,
		const scalar_t& finalTime) {

	const size_t numTemplateSubsystems = modeSequenceTemplate_.templateSubsystemsSequence_.size();

	// If no template subsystem is defined, the last subsystem should continue for ever
	if (numTemplateSubsystems == 0) {
		return;
	}

	if (modeSequenceTemplate_.templateSwitchingTimes_.size() != numTemplateSubsystems+1) {
		throw std::runtime_error("The number of the subsystems in the user-defined template should be equal to "
				"the number of the template switching times minus 1.");
	}

	if (startTime <= BASE::eventTimes_.back())
		throw std::runtime_error("The initial time for template-tiling is not greater than the last event time.");

	// add a initial time
	BASE::eventTimes_.push_back(startTime);

	// concatenate from index
	while (BASE::eventTimes_.back() < finalTime) {

		for (size_t i=0; i<modeSequenceTemplate_.templateSubsystemsSequence_.size(); i++) {

			BASE::subsystemsSequence_.push_back(modeSequenceTemplate_.templateSubsystemsSequence_[i]);
			scalar_t deltaTime = modeSequenceTemplate_.templateSwitchingTimes_[i+1] - modeSequenceTemplate_.templateSwitchingTimes_[i];
			BASE::eventTimes_.push_back(BASE::eventTimes_.back()+deltaTime);
		}  // end of i loop

	}  // end of while loop

	// default final phase
	BASE::subsystemsSequence_.push_back(ModeNumber::STANCE);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, class cpg_t, size_t STATE_DIM, size_t INPUT_DIM>
void SwitchedModelLogicRulesBase<JOINT_COORD_SIZE,cpg_t,STATE_DIM,INPUT_DIM>::adjustController(
		const scalar_array_t& eventTimes,
		const scalar_array_t& controllerEventTimes,
		controller_array_t& controllerStock) {

	trajectorySpreadingController_.adjustController(
			BASE::eventTimes(),
			controllerEventTimes,
			controllerStock);
}

} // namespace switched_model


