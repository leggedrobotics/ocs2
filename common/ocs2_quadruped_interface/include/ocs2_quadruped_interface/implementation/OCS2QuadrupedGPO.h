/*
 * OCS2QuadrupedGPO.h
 *
 *  Created on: Mar 15, 2018
 *      Author: farbod
 */

namespace switched_model {


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
OCS2QuadrupedGPO<JOINT_COORD_SIZE>::OCS2QuadrupedGPO(
		const quadruped_interface_ptr_t& ocs2QuadrupedInterfacePtr,
		const std::string& robotName /*robot*/)

: ocs2QuadrupedInterfacePtr_(ocs2QuadrupedInterfacePtr),
  robotName_(robotName),
  mpcSettings_(ocs2QuadrupedInterfacePtr->getMpcSettings()),
  slqPtr_(ocs2QuadrupedInterfacePtr->getSLQPtr())
{
	//get mpc settings
	if (mpcSettings_.recedingHorizon_==false)
		mpcSettings_.rosMsgTimeWindow_ = 1e+6;

	reset();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void OCS2QuadrupedGPO<JOINT_COORD_SIZE>::reset() {

	userDefinedGait_ = false;
	logicRulesIsUpdated_ = false;

	eventTimesTemplate_.clear();
	phaseSequenceTemplate_.clear();

	slqRewindCounterCopy_ = slqPtr_->getRewindCounter();

	// logic rule initialization
//	feet_z_planner_ptr_t feetZPlannerPtr(
//			new feet_z_planner_t(ocs2QuadrupedInterfacePtr_->getModelSettings().swingLegLiftOff_, 1.0 /*swingTimeScale*/) );
//	optimizedLogicRules_ = logic_rules_t(feetZPlannerPtr);
	optimizedLogicRules_ = *slqPtr_->getLogicRulesPtr();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void OCS2QuadrupedGPO<JOINT_COORD_SIZE>::sigintHandler(int sig)  {

	std::cerr << "Shutting GPO node. " << std::endl;
	ros::shutdown();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void OCS2QuadrupedGPO<JOINT_COORD_SIZE>::launchNodes(int argc, char* argv[]) {

	// reset counters and variables
	reset();

	// display
	std::cerr << "ROS nodes are setting up ..."<< std::endl;

	// setup ROS
	ros::init(argc, argv, robotName_+"_gpo", ros::init_options::NoSigintHandler);
	signal(SIGINT, OCS2QuadrupedGPO::sigintHandler);
	ros::NodeHandle nodeHandler;

	// user defined gait subscriber
	userDefinedGaitSubscriber_ = nodeHandler.subscribe(robotName_+"_user_gait", 1, &OCS2QuadrupedGPO::userDefinedGaitCallback, this);

	ros::spinOnce();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void OCS2QuadrupedGPO<JOINT_COORD_SIZE>::userDefinedGaitCallback (
		const ocs2_ros_msg::gait_sequence::ConstPtr& msg) {

	userDefinedGait_ = extractTemplateMotionSequence(*msg,
			eventTimesTemplate_, phaseSequenceTemplate_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
template <class ContainerAllocator>
bool OCS2QuadrupedGPO<JOINT_COORD_SIZE>::extractTemplateMotionSequence(
		const ocs2_ros_msg::gait_sequence_<ContainerAllocator>& gaitSequenceMsg,
		scalar_array_t& eventTimes,
		size_array_t& phaseSequence) const {

	const size_t numSubsystems = gaitSequenceMsg.phaseSequence.size();
	if (gaitSequenceMsg.eventTimes.size() != numSubsystems)
		throw std::runtime_error("The template-gait message should have equal sizes for eventTimes and phaseSequence.");

	scalar_array_t eventTimesTemp(numSubsystems);
	size_array_t phaseSequenceTemp(numSubsystems);

	for (size_t i=0; i<numSubsystems; i++) {
		eventTimesTemp[i]    = gaitSequenceMsg.eventTimes[i];
		phaseSequenceTemp[i] = gaitSequenceMsg.phaseSequence[i];
	}

	bool isLogicUpdated = false;

	if (eventTimesTemp != eventTimes){
		eventTimes = std::move(eventTimesTemp);
		isLogicUpdated = true;
	}
	if (phaseSequenceTemp != phaseSequence){
		phaseSequence = std::move(phaseSequenceTemp);
		isLogicUpdated = true;
	}

	return isLogicUpdated;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
typename OCS2QuadrupedGPO<JOINT_COORD_SIZE>::scalar_t OCS2QuadrupedGPO<JOINT_COORD_SIZE>::computeLogicFinalTime(
		const scalar_array_t& partitioningTimes) const {

	if (partitioningTimes.empty() == true)
		return 0.0;
	else
		return partitioningTimes.back() + 0/3 *(partitioningTimes.back()-partitioningTimes.front());
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void OCS2QuadrupedGPO<JOINT_COORD_SIZE>::run() {

	/******************************************************************************************
	 * Data logging from SLQ
	 ******************************************************************************************/
	// lock data in SLQ and copy the nessary ones
	// slqPtr_->dataLatch(true);
	// GSLQ copies data from SLQ (if possible in multi-thread way)

	partitioningTimes_ = slqPtr_->partitioningTimes_;

	const size_t rewindFirstIndex = slqPtr_->getRewindCounter() - slqRewindCounterCopy_;

	logicFinalTime_ = computeLogicFinalTime(partitioningTimes_);
	const scalar_t startTimeToModifyGait = slqPtr_->getFinalTime();

	// slqPtr_->dataLatch(false);

	/******************************************************************************************
	 * Rewind if the SLQ has been rewound.
	 ******************************************************************************************/
	if (rewindFirstIndex>0) {
		slqRewindCounterCopy_ += rewindFirstIndex;

		// rewind the internal logicRules
		scalar_array_t& eventTimes = optimizedLogicRules_.eventTimes();
		size_array_t& gaitSequence = optimizedLogicRules_.getMotionPhasesSequence();

		const size_t index = std::lower_bound(eventTimes.begin(), eventTimes.end(), partitioningTimes_.front()) - eventTimes.begin();

		if (index > 0) {

			// delete the old logic from index and set the default start phase to stance
			eventTimes.erase(eventTimes.begin(), eventTimes.begin()+index-1);  // keep the one before the last to make it stance
			gaitSequence.erase(gaitSequence.begin(), gaitSequence.begin()+index-1);

			// set the default initial phase
			gaitSequence.begin() = ModeNumber::STANCE;

			// delete the last default stance phase
			gaitSequence.erase(gaitSequence.end()-1, gaitSequence.end());

			while (eventTimes.back() < logicFinalTime_) {
				for (size_t i=0; i<eventTimesTemplate_.size(); i++) {
					gaitSequence.puch_back(phaseSequenceTemplate_[i]);
					eventTimes.puch_back(eventTimesTemplate_[i]+eventTimes.back());
				}
			}

			// default final phase
			gaitSequence.puch_back(ModeNumber::STANCE);

			// logicRules are updated
			logicRulesIsUpdated_ = true;
		}

		// rewind GSLQ by rewindFirstIndex
		// TODO: do something
	}

	/******************************************************************************************
	 * If user has defined gait preference
	 ******************************************************************************************/
	if (userDefinedGait_==true) {

		userDefinedGait_ = false;

		// find index on which the new gait should be added
		scalar_array_t& eventTimes = optimizedLogicRules_.eventTimes();
		size_array_t& gaitSequence = optimizedLogicRules_.getMotionPhasesSequence();
		size_t index = std::lower_bound(eventTimes.begin(), eventTimes.end(), startTimeToModifyGait) - eventTimes.begin();

		// delete the old logic from the index
		if (index < eventTimes.size()) {
			eventTimes.erase(eventTimes.begin()+index, eventTimes.end());
			gaitSequence.erase(gaitSequence.begin()+index+1, gaitSequence.end());
		}

		// add a stance phase
		eventTimes.puch_back(startTimeToModifyGait);
		gaitSequence.puch_back(ModeNumber::STANCE);
		eventTimes.puch_back(startTimeToModifyGait+phaseTransitionStanceTime_);

		// concatenate from index
		while (eventTimes.back() < logicFinalTime_) {
			for (size_t i=0; i<eventTimesTemplate_.size(); i++) {
				gaitSequence.puch_back(phaseSequenceTemplate_[i]);
				eventTimes.puch_back(eventTimesTemplate_[i]+eventTimes.back());
			}
		}

		// default final phase
		gaitSequence.puch_back(ModeNumber::STANCE);

		// logicRules are updated
		logicRulesIsUpdated_ = true;
	}

	/******************************************************************************************
	 * Run GSLQ from initial logic optimizedLogicRules_
	 ******************************************************************************************/
	// TODO: do something


	/******************************************************************************************
	 * Get updated logicRules
	 ******************************************************************************************/
	// TODO: do something and update logicRulesIsUpdated_


	/******************************************************************************************
	 * Set LogicRuls
	 ******************************************************************************************/
	if (logicRulesIsUpdated_==true) {

		slqPtr_->setLogicRules(optimizedLogicRules_);
	}


	/******************************************************************************************
	 * Wait at least for an iteration of SLQ to finish
	 ******************************************************************************************/
	if (logicRulesIsUpdated_==true) {
		// TODO: do something
	}


	logicRulesIsUpdated_ = false;
}


} // end of namespace switched_model




