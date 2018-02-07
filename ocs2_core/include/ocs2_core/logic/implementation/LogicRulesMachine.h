/*
 * LogicRulesMachine.h
 *
 *  Created on: Dec 19, 2017
 *      Author: farbod
 */

namespace ocs2{

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/*
 * Set a new logic rules class
 * @param [in] logicRules: The new logic rules class
 */
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void LogicRulesMachine<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::setLogicRules(const LOGIC_RULES_T& logicRules) {

	logicRulesModified_ = true;
	logicRules_ = logicRules;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/*
 * Get the active logic rules class
 * @return active logic rules class
 */
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
const LOGIC_RULES_T& LogicRulesMachine<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getLogicRules() const {

	return logicRulesInUse_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/*
 * Gets the switching times associated to the partition number index.
 * @param [in] index: index of the time partition.
 * @return
 */
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
const typename LogicRulesMachine<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::scalar_array_t&
	LogicRulesMachine<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getSwitchingTimes(size_t index) const {

	return eventTimesStock_[index];
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/*
 * Gets the IDs of the switched systems associated to the partition number index.
 * @param [in] index: index of the time partition
 * @return
 */
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
const typename LogicRulesMachine<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::size_array_t&
	LogicRulesMachine<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getSwitchedSystemIDs(size_t index) const {

	return switchedSystemIDsStock_[index];
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/*
 * Returns a Lambda expression which can be used to find the current active subsystem's ID.
 *
 * @param partitionIndex: index of the time partition
 * @return Lambda expression
 */
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
std::function<size_t(typename LogicRulesMachine<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::scalar_t)>
	LogicRulesMachine<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getHandleToFindActiveSubsystemID(
		size_t partitionIndex) const {

	const size_array_t& switchedSystemIDs = switchedSystemIDsStock_[index];
	size_t numSubsystems = switchedSystemIDs.size();

	scalar_array_t switchingTimes;
	switchingTimes.reserve(1+numSubsystems);

	// add the partition's start time
	switchingTimes.push_back(partitioningTimes_[partitionIndex]);
	// add the intermediate switching times
	for (const scalar_t& t: this->getSwitchingTimes(partitionIndex))
		switchingTimes.push_back(t);
	// only add the partition's final time if there is no intermediate switching time at the end
	if (switchingTimes.size() == numSubsystems)
		switchingTimes.push_back(partitioningTimes_[partitionIndex+1]);
	else
		switchingTimes[numSubsystems] = partitioningTimes_[partitionIndex+1];

	// return Lambda expression
	return [switchingTimes, switchedSystemIDs](scalar_t time) {
		static int guessedIndex_ = 0;
		int index = findActiveIntervalIndex(switchingTimes, time, guessedIndex_);

		if (index < 0 || index >= switchedSystemIDs.size())
			throw std::runtime_error("The enquiry time refers to an out-of-bound subsystem.");

		guessedIndex_ = index;

		return switchedSystemIDs[index];
	};
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/*
 * Updates the active logic rules based on the last set value (using LogicRulesMachine::setLogicRules) and
 * adjusts the controller based on LOGIC_RULES_T::adjustController() routine. Moreover, it recomputes the
 * distribution of the switched systems over the time partitions.
 *
 * @param [in] partitioningTimes: Vector of time partitions.
 * @param controllerStock: The controller which will be adjusted.
 */
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void LogicRulesMachine<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::updateLogicRules(
		const scalar_array_t& partitioningTimes,
		controller_array_t& controllerStock) {

	// if logic rules is modified
	if (logicRulesModified_ == true)  {

		logicRulesInUse_ = std::move(logicRules_);

		// adjust controller
		for (auto controllerItr=controllerStock.begin(); controllerItr!=controllerStock.end(); ++controllerItr)
			if (controllerItr->time_.empty()==false)
				logicRulesInUse_.adjustController(*controllerItr);
	}

	// if partitioningTimes is updated
	if (logicRulesModified_==true || partitioningTimes_!=partitioningTimes) {

		if (partitioningTimes.size()<2)
			throw std::runtime_error("Time partitioning vector should include at least the start and final time.");

		numPartitionings_ = partitioningTimes.size()-1;
		partitioningTimes_ = partitioningTimes;
		// recomputes the distribution of the switched systems over the time partitions.
		findSwitchedSystemsDistribution(partitioningTimes, eventTimesStock_, switchedSystemIDsStock_);
	}

	logicRulesModified_ = false;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/*
 * Find distribution of the switched systems over the time partitions.
 * @param [in] partitioningTimes: Vector of time partitions.
 * @param [out] eventTimesStock: Distribution of the event times over partitions.
 * @param [out] switchedSystemIDsStock: Distribution of the switched system over partitions identified by their index.
 */
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void LogicRulesMachine<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::findSwitchedSystemsDistribution(
		const scalar_array_t& partitioningTimes,
		std::vector<scalar_array_t>& eventTimesStock,
		std::vector<size_array_t>& switchedSystemIDsStock) {

	const scalar_array_t& switchingTimes = getLogicRules().logicRulesSwitchingTimes();
	const size_t numSubsystems = switchingTimes.size()+1;

	/*
	 * Finding the distribution of the event's indeces over partitions.
	 */
	int lastIndex = 0;
	int firstActiveSwitchinTimeIndex = -1;
	std::vector<size_array_t> eventIndecesStock(numPartitionings_, size_array_t(0));
	for (size_t i=0; i<switchingTimes.size(); i++) {

		const double& ti = switchingTimes[i];

		int index = findActiveIntervalIndex(partitioningTimes, ti, lastIndex);

		if (index < 0 || index==numPartitionings_)  continue;

		// At the very first active index
		if (firstActiveSwitchinTimeIndex==-1)  {
			// if switch happens at the startTime ignore it
			if (ti < partitioningTimes.front() + OCS2NumericTraits<double>::limit_epsilon())  continue;

			// save the very first active index
			firstActiveSwitchinTimeIndex = i;
		}

		eventIndecesStock[index].push_back(i);
		lastIndex = index;
	}  // end of i loop

	/*
	 * the fist active susbsystem
	 */
	size_t currActiveSubsystemIndex;
	if (firstActiveSwitchinTimeIndex != -1)  // the normal case
	{
		currActiveSubsystemIndex = firstActiveSwitchinTimeIndex;
	}
	else // there are 3 case: no swich, all switches before or after the current time interval
	{
		if (switchingTimes.empty()==false && switchingTimes.back() < partitioningTimes.front()+OCS2NumericTraits<double>::limit_epsilon())
			currActiveSubsystemIndex = numSubsystems-1;
		else
			currActiveSubsystemIndex = 0;
	}

	/*
	 * Finding the distributions of the event's times and switched systems over partitions based on eventIndecesStock.
	 */
	eventTimesStock = std::vector<scalar_array_t>(numPartitionings_, scalar_array_t(0));  // a subsystem might not have any switches
	switchedSystemIDsStock = std::vector<size_array_t>(numPartitionings_, size_array_t(1,0));  // a subsystem has been assigned at least one system ID
	for (size_t i=0; i<numPartitionings_; i++) {

		switchedSystemIDsStock[i][0] = currActiveSubsystemIndex;

		for (size_t j=0; j<eventIndecesStock[i].size(); j++) {

			size_t& index = eventIndecesStock[i][j];
			eventTimesStock[i].push_back( switchingTimes[index] );

			// since a switch happended the next switched system should be pushed to this partitions switched systems list
			// unless the swich happend at the end of the time partition
			currActiveSubsystemIndex++;
			if (partitioningTimes[i+1] - eventTimesStock[i].back() > OCS2NumericTraits<double>::limit_epsilon()) {
				switchedSystemIDsStock[i].push_back(currActiveSubsystemIndex);
			}

		} // end of j loop
	}  // end of i loop

}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/*
 * Displays switched systems distribution over the time partitions.
 */
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void LogicRulesMachine<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::displaySwitchedSystemsDistribution() {

	std::cerr << std::endl << "Event Times:\n\t {";
	for (auto& t: getLogicRules().logicRulesSwitchingTimes())
		std::cerr << t << ", ";
	if (!getLogicRules().logicRulesSwitchingTimes().empty())  std::cerr << "\b\b";
	std::cerr << "}" << std::endl;

	std::cerr << "Subsystem IDs:\n\t {";
	for (size_t i=0; i<=getLogicRules().logicRulesSwitchingTimes().size() ;i++)
		std::cerr << i << ", ";
	std::cerr << "\b\b}" << std::endl;

	std::cerr << "Partitioning intervals:\n\t ";
	for (size_t i=0; i<numPartitionings_; i++) {
		std::cerr << i << ":[" << partitioningTimes_[i] << "," << partitioningTimes_[i+1] << "]  ";
	}
	std::cerr << std::endl;

	size_t itr = 0;

	std::cerr << "Event times distribution for partitions:\n\t ";
	for (auto& events : eventTimesStock_) {  // printing
		std::cerr << itr << ":{";
		itr++;

		for (auto& ti : events) {
			std::cerr << ti << ", ";
		}
		if (!events.empty())  std::cerr << "\b\b";
		std::cerr << "},  ";
	}
	std::cerr << std::endl;

	itr = 0;
	std::cerr << "Subsystem IDs distribution for partitions:\n\t ";
	for (auto& switchedSystemIDs : switchedSystemIDsStock_) {  // printing
		std::cerr << itr << ":{";
		itr++;

		for (auto& i : switchedSystemIDs) {
			std::cerr << i << ", ";
		}
		if (!switchedSystemIDs.empty())  std::cerr << "\b\b";
		std::cerr << "},  ";
	}
	std::cerr << std::endl << std::endl;
}

} // namespace ocs2
