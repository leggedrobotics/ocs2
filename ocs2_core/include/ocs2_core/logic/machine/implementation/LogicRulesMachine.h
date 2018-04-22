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
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void LogicRulesMachine<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::setLogicRules(const LOGIC_RULES_T& logicRules) {

	logicRulesUpdated();
	logicRulesModifiedOffline_ = true;

	logicRules_ = logicRules;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void LogicRulesMachine<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::logicRulesUpdated() {

	logicRulesModified_ = true;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
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
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
const typename LogicRulesMachine<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::scalar_array_t&
	LogicRulesMachine<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getEventTimes(size_t index) const {

	return eventTimesStock_[index];
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
const typename LogicRulesMachine<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::size_array_t&
	LogicRulesMachine<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getEventCounters(size_t index) const {

	return eventCountersStock_[index];
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
const typename LogicRulesMachine<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::scalar_array_t&
	LogicRulesMachine<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getSwitchingTimes(size_t index) const {

	return switchingTimesStock_[index];
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
const typename LogicRulesMachine<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::scalar_array_t&
	LogicRulesMachine<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getPartitioningTimes() const {

	return partitioningTimes_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
std::function<size_t(typename LogicRulesMachine<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::scalar_t)>
	LogicRulesMachine<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getHandleToFindActiveEventCounter(
		size_t partitionIndex) const {

	const size_array_t& eventCounters = eventCountersStock_[partitionIndex];
	const scalar_array_t& switchingTimes = switchingTimesStock_[partitionIndex];

	// return Lambda expression
	int guessedIndex = 0;
	return [&eventCounters, &switchingTimes, guessedIndex](scalar_t time) mutable {
		int index = findActiveIntervalIndex(switchingTimes, time, guessedIndex);

		if (index < 0 || index >= eventCounters.size())
			throw std::runtime_error("The enquiry time" + std::to_string(time) + "refers to an out-of-range subsystem.");

		guessedIndex = index;

		return eventCounters[index];
	};
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
size_t LogicRulesMachine<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getNumEventCounters(size_t partitionIndex) const {

	return eventCountersStock_[partitionIndex].size();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
size_t LogicRulesMachine<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getNumEvents(size_t partitionIndex) const  {

	return eventTimesStock_[partitionIndex].size();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void LogicRulesMachine<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::updateLogicRules(
		const scalar_array_t& partitioningTimes,
		controller_array_t& controllerStock) {

	// if logic rules is modified update the logic
	if (logicRulesModifiedOffline_ == true) {
		logicRulesInUse_ = std::move(logicRules_);
		logicRulesModifiedOffline_ = false;
	}

	// if partitioningTimes is updated
	if (logicRulesModified_==true || partitioningTimes_!=partitioningTimes) {

		if (partitioningTimes.size()<2)
			throw std::runtime_error("Time partitioning vector should include at least the start and final time.");

		numPartitionings_ = partitioningTimes.size()-1;
		partitioningTimes_ = partitioningTimes;
		// recomputes the distribution of the switched systems over the time partitions.
		findEventsDistribution(partitioningTimes, eventTimesStock_, switchingTimesStock_,
				eventCountersStock_);
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
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void LogicRulesMachine<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::findEventsDistribution(
		const scalar_array_t& partitioningTimes,
		std::vector<scalar_array_t>& eventTimesStock,
		std::vector<scalar_array_t>& switchingTimesStock,
		std::vector<size_array_t>& eventCountersStock) {

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
	else // there are 4 cases: no event, all events are before or after the current time interval or the time Partitions are in between two event times.
	{
		if (allEventTimes.empty()==true) {
			currActiveSubsystemIndex = 0;

		} else {
			if (allEventTimes.back() < partitioningTimes.front()+OCS2NumericTraits<double>::limit_epsilon())
				currActiveSubsystemIndex = numSubsystems-1;

			else if (allEventTimes.front() >= partitioningTimes.back())
				currActiveSubsystemIndex = 0;

			else { // last case
				currActiveSubsystemIndex = findActiveIntervalIndex(allEventTimes, partitioningTimes.front(), 0) + 1;
			}
		}
	}

	/*
	 * Finding the distributions of the event's times and switched systems over partitions based on eventIndecesStock.
	 */
	eventTimesStock = std::vector<scalar_array_t>(numPartitionings_, scalar_array_t(0));  // a subsystem might not have any switches
	eventCountersStock = std::vector<size_array_t>(numPartitionings_, size_array_t(1,0));  // a subsystem has been assigned at least one system ID
	for (size_t i=0; i<numPartitionings_; i++) {

		eventCountersStock[i][0] = currActiveSubsystemIndex;

		for (size_t j=0; j<eventIndecesStock[i].size(); j++) {

			size_t& index = eventIndecesStock[i][j];
			eventTimesStock[i].push_back( allEventTimes[index] );

			// since a switch happended the next switched system should be pushed to this partitions switched systems list
			// unless the swich happend at the end of the time partition
			currActiveSubsystemIndex++;
			if (partitioningTimes[i+1] - eventTimesStock[i].back() > OCS2NumericTraits<double>::limit_epsilon()) {
				eventCountersStock[i].push_back(currActiveSubsystemIndex);
			}

		} // end of j loop
	}  // end of i loop

	/*
	 * calculates switching times
	 */
	switchingTimesStock.resize(numPartitionings_);
	for (size_t i=0; i<numPartitionings_; i++) {

		const size_t numSubsystems = eventCountersStock_[i].size();
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
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void LogicRulesMachine<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::display() const {

	this->getLogicRules().display();

	size_t itr;

	itr = 0;
	std::cerr << "Switching times distribution for partitions:\n\t ";
	for (auto& switchingTimes : switchingTimesStock_) {  // printing
		std::cerr << itr << ":{";
		itr++;

		for (auto& ti : switchingTimes) {
			std::cerr << ti << ", ";
		}
		if (!switchingTimes.empty())  std::cerr << "\b\b";
		std::cerr << "},  ";
	}
	std::cerr << std::endl;

	itr = 0;
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
	std::cerr << "Event counters distribution for partitions:\n\t ";
	for (auto& eventCounters : eventCountersStock_) {  // printing
		std::cerr << itr << ":{";
		itr++;

		for (auto& i : eventCounters) {
			std::cerr << i << ", ";
		}
		if (!eventCounters.empty())  std::cerr << "\b\b";
		std::cerr << "},  ";
	}
	std::cerr << std::endl;
}

} // namespace ocs2
