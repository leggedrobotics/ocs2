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
LOGIC_RULES_T& LogicRulesMachine<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getLogicRules() {

	return logicRulesInUse_;
}

template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
const LOGIC_RULES_T& LogicRulesMachine<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getLogicRules() const {

	return logicRulesInUse_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/*
 * Get the pointer to the active logic rules class
 *
 * @return pointer to active logic rules class
 */
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
LOGIC_RULES_T* LogicRulesMachine<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getLogicRulesPtr() {

	return &logicRulesInUse_;
}

template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
const LOGIC_RULES_T* LogicRulesMachine<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getLogicRulesPtr() const {

	return &logicRulesInUse_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/*
 * Gets the event times associated to the partition number index.
 * @param [in] index: index of the time partition.
 * @return
 */
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
const typename LogicRulesMachine<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::scalar_array_t&
	LogicRulesMachine<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getEventTimes(size_t index) const {

	return eventTimesStock_[index];
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/*
 * Gets the partitioning times.
 * @return const reference to partitioning times.
 */
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
const typename LogicRulesMachine<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::scalar_array_t&
	LogicRulesMachine<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getPartitioningTimes() const {

	return partitioningTimes_;
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
 * Gets the switching times associated to the partition number index.
 * @param [in] index: index of the time partition
 * @return
 */
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
const typename LogicRulesMachine<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::scalar_array_t&
	LogicRulesMachine<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getSwitchingTimes(size_t index) const {

	return switchingTimesStock_[index];
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

	const size_array_t& switchedSystemIDs = switchedSystemIDsStock_[partitionIndex];
	const scalar_array_t& switchingTimes = switchingTimesStock_[partitionIndex];

	// return Lambda expression
	return [switchingTimes, switchedSystemIDs](scalar_t time) {
		static int guessedIndex_ = 0;
		int index = findActiveIntervalIndex(switchingTimes, time, guessedIndex_);

		if (index < 0 || index >= switchedSystemIDs.size())
			throw std::runtime_error("The enquiry time refers to an out-of-range subsystem.");

		guessedIndex_ = index;

		return switchedSystemIDs[index];
	};
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/*
 * Returns the number of subsystems in the partition.
 *
 * @param partitionIndex: index of the time partition
 * @return Number of subsystems
 */
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
size_t LogicRulesMachine<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getNumSubsystems(size_t partitionIndex) const {

	return switchedSystemIDsStock_[partitionIndex].size();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/*
 * Returns the number of event in the partition.
 *
 * @param partitionIndex: index of the time partition
 * @return Number of events
 */
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
size_t LogicRulesMachine<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getNumEvents(size_t partitionIndex) const  {

	return eventTimesStock_[partitionIndex].size();
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

	// if logic rules is modified update the logic
	if (logicRulesModified_ == true)
		logicRulesInUse_ = std::move(logicRules_);

	// if partitioningTimes is updated
	if (logicRulesModified_==true || partitioningTimes_!=partitioningTimes) {

		if (partitioningTimes.size()<2)
			throw std::runtime_error("Time partitioning vector should include at least the start and final time.");

		numPartitionings_ = partitioningTimes.size()-1;
		partitioningTimes_ = partitioningTimes;
		// recomputes the distribution of the switched systems over the time partitions.
		findSwitchedSystemsDistribution(partitioningTimes, eventTimesStock_, switchingTimesStock_,
				switchedSystemIDsStock_);
	}

	// adjust controller
	if (logicRulesModified_ == true && controllerStock.size()>0) {
		// adjust controller
		for (auto controllerItr=controllerStock.begin(); controllerItr!=controllerStock.end(); ++controllerItr)
			if (controllerItr->time_.empty()==false)
				this->getLogicRules().adjustController(*controllerItr);
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
 * @param [out] switchingTimesStock: Distribution of the switching times over partitions.
 * @param [out] switchedSystemIDsStock: Distribution of the switched system over partitions identified by their index.
 */
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void LogicRulesMachine<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::findSwitchedSystemsDistribution(
		const scalar_array_t& partitioningTimes,
		std::vector<scalar_array_t>& eventTimesStock,
		std::vector<scalar_array_t>& switchingTimesStock,
		std::vector<size_array_t>& switchedSystemIDsStock) {

	const scalar_array_t& allEventTimes = this->getLogicRules().eventTimes();
	const size_t numSubsystems = allEventTimes.size()+1;

	/*
	 * Finding the distribution of the event's indeces over partitions.
	 */
	int lastIndex = 0;
	int firstActiveSwitchinTimeIndex = -1;
	std::vector<size_array_t> eventIndecesStock(numPartitionings_, size_array_t(0));
	for (size_t i=0; i<allEventTimes.size(); i++) {

		const scalar_t& ti = allEventTimes[i];

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
		if (allEventTimes.empty()==false && allEventTimes.back() < partitioningTimes.front()+OCS2NumericTraits<double>::limit_epsilon())
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
			eventTimesStock[i].push_back( allEventTimes[index] );

			// since a switch happended the next switched system should be pushed to this partitions switched systems list
			// unless the swich happend at the end of the time partition
			currActiveSubsystemIndex++;
			if (partitioningTimes[i+1] - eventTimesStock[i].back() > OCS2NumericTraits<double>::limit_epsilon()) {
				switchedSystemIDsStock[i].push_back(currActiveSubsystemIndex);
			}

		} // end of j loop
	}  // end of i loop

	/*
	 * calculates switching times
	 */
	switchingTimesStock.resize(numPartitionings_);
	for (size_t i=0; i<numPartitionings_; i++) {

		const size_t numSubsystems = switchedSystemIDsStock_[i].size();
		scalar_array_t& switchingTimes = switchingTimesStock[i];

		switchingTimes.clear();
		switchingTimes.reserve(1+numSubsystems);

		// add the partition's start time
		switchingTimes.push_back(partitioningTimes_[i]);
		// add the intermediate switching times
		for (const scalar_t& t: eventTimesStock_[i])
			switchingTimes.push_back(t);
		// only add the partition's final time if there is no intermediate switching time at the end
		if (switchingTimes.size() == numSubsystems)
			switchingTimes.push_back(partitioningTimes_[i+1]);
		else
			switchingTimes[numSubsystems] = partitioningTimes_[i+1];
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
	for (auto& t: this->getLogicRules().eventTimes())
		std::cerr << t << ", ";
	if (!this->getLogicRules().eventTimes().empty())  std::cerr << "\b\b";
	std::cerr << "}" << std::endl;

	std::cerr << "Subsystem IDs:\n\t {";
	for (size_t i=0; i<=this->getLogicRules().eventTimes().size() ;i++)
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
