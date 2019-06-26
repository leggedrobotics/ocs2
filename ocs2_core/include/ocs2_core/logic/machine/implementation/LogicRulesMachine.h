/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class LOGIC_RULES_T>
void LogicRulesMachine<LOGIC_RULES_T>::setLogicRules(const LOGIC_RULES_T& logicRules) {

	logicRulesUpdated();
	newLogicRulesInBuffer_ = true;

	logicRulesBuffer_ = logicRules;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class LOGIC_RULES_T>
void LogicRulesMachine<LOGIC_RULES_T>::logicRulesUpdated() {

	logicRulesModified_ = true;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class LOGIC_RULES_T>
LOGIC_RULES_T* LogicRulesMachine<LOGIC_RULES_T>::getLogicRulesPtr() {

	return &logicRules_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class LOGIC_RULES_T>
const LOGIC_RULES_T* LogicRulesMachine<LOGIC_RULES_T>::getLogicRulesPtr() const {

	return &logicRules_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class LOGIC_RULES_T>
const typename LogicRulesMachine<LOGIC_RULES_T>::scalar_array_t&
	LogicRulesMachine<LOGIC_RULES_T>::getEventTimes(size_t index) const {

	return eventTimesStock_[index];
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class LOGIC_RULES_T>
const typename LogicRulesMachine<LOGIC_RULES_T>::size_array_t&
	LogicRulesMachine<LOGIC_RULES_T>::getEventCounters(size_t index) const {

	return eventCountersStock_[index];
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class LOGIC_RULES_T>
const typename LogicRulesMachine<LOGIC_RULES_T>::scalar_array_t&
	LogicRulesMachine<LOGIC_RULES_T>::getSwitchingTimes(size_t index) const {

	return switchingTimesStock_[index];
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class LOGIC_RULES_T>
const typename LogicRulesMachine<LOGIC_RULES_T>::scalar_array_t&
	LogicRulesMachine<LOGIC_RULES_T>::getPartitioningTimes() const {

	return partitioningTimes_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class LOGIC_RULES_T>
std::function<size_t(typename LogicRulesMachine<LOGIC_RULES_T>::scalar_t)>
	LogicRulesMachine<LOGIC_RULES_T>::getHandleToFindActiveEventCounter(
		size_t partitionIndex) const {

	const size_array_t& eventCounters = eventCountersStock_[partitionIndex];
	const scalar_array_t& switchingTimes = switchingTimesStock_[partitionIndex];

	// return Lambda expression
	int guessedIndex = 0;
	return [&eventCounters, &switchingTimes, guessedIndex](scalar_t time) mutable {
		int index = findActiveIntervalIndex(switchingTimes, time, guessedIndex);

		if (index < 0 || index >= eventCounters.size()) {
			throw std::runtime_error("The enquiry time" + std::to_string(time) + "refers to an out-of-range subsystem.");
		}

		guessedIndex = index;

		return eventCounters[index];
	};
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class LOGIC_RULES_T>
size_t LogicRulesMachine<LOGIC_RULES_T>::getNumEventCounters(size_t partitionIndex) const {

	return eventCountersStock_[partitionIndex].size();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class LOGIC_RULES_T>
size_t LogicRulesMachine<LOGIC_RULES_T>::getNumEvents(size_t partitionIndex) const  {

	return eventTimesStock_[partitionIndex].size();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class LOGIC_RULES_T>
const size_t& LogicRulesMachine<LOGIC_RULES_T>::getNumPartitions() const  {

	return numPartitions_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class LOGIC_RULES_T>
bool LogicRulesMachine<LOGIC_RULES_T>::updateLogicRules(
		const scalar_array_t& partitioningTimes) {

	// if logic rules is modified update the logic
	if (newLogicRulesInBuffer_) {
		logicRules_ = std::move(logicRulesBuffer_);
		newLogicRulesInBuffer_ = false;
	}

	// if partitioningTimes is updated
	if (logicRulesModified_ || partitioningTimes_!=partitioningTimes) {

		if (partitioningTimes.size()<2) {
			throw std::runtime_error("Time partitioning vector should include at least the start and final time.");
		}

		numPartitions_ = partitioningTimes.size()-1;
		partitioningTimes_ = partitioningTimes;
		// recomputes the distribution of the switched systems over the time partitions.
		findEventsDistribution(partitioningTimes, eventTimesStock_, switchingTimesStock_,
				eventCountersStock_);
	}

	// return true if logic rule was updated no the partition or any other conditions
	if (logicRulesModified_) {
		logicRulesModified_ = false;
		return true;
	} else {
		return false;
	}
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class LOGIC_RULES_T>
void LogicRulesMachine<LOGIC_RULES_T>::findEventsDistribution(
		const scalar_array_t& partitioningTimes,
		std::vector<scalar_array_t>& eventTimesStock,
		std::vector<scalar_array_t>& switchingTimesStock,
		std::vector<size_array_t>& eventCountersStock) {

	const scalar_array_t& allEventTimes = this->getLogicRulesPtr()->eventTimes();
	const size_t numSubsystems = allEventTimes.size()+1;

	/*
	 * Finding the distribution of the event's indices over partitions.
	 */
	int lastIndex = 0;
	int firstActiveSwitchinTimeIndex = -1;
	std::vector<size_array_t> eventIndecesStock(numPartitions_, size_array_t(0));
	for (size_t i=0; i<allEventTimes.size(); i++) {

		const scalar_t& ti = allEventTimes[i];

		int index = findActiveIntervalIndex(partitioningTimes, ti, lastIndex);

		if (index < 0 || index==numPartitions_) {  continue;
		}

		// At the very first active index
		if (firstActiveSwitchinTimeIndex==-1)  {
			// if switch happens at the startTime ignore it
			if (ti < partitioningTimes.front() + OCS2NumericTraits<scalar_t>::limit_epsilon()) {  continue;
			}

			// save the very first active index
			firstActiveSwitchinTimeIndex = i;
		}

		eventIndecesStock[index].push_back(i);
		lastIndex = index;
	}  // end of i loop

	/*
	 * the fist active subsystem
	 */
	size_t currActiveSubsystemIndex;
	if (firstActiveSwitchinTimeIndex != -1)  // the normal case
	{
		currActiveSubsystemIndex = firstActiveSwitchinTimeIndex;
	}
	else // there are 4 cases: no event, all events are before or after the current time interval or the time Partitions are in between two event times.
	{
		if (allEventTimes.empty()) {
			currActiveSubsystemIndex = 0;

		} else {
			if (allEventTimes.back() < partitioningTimes.front()+OCS2NumericTraits<scalar_t>::limit_epsilon()) {
				currActiveSubsystemIndex = numSubsystems-1;
			} else if (allEventTimes.front() >= partitioningTimes.back()) {
				currActiveSubsystemIndex = 0;
			} else { // last case
				currActiveSubsystemIndex = findActiveIntervalIndex(allEventTimes, partitioningTimes.front(), 0) + 1;
			}
		}
	}

	/*
	 * Finding the distributions of the event's times and switched systems over partitions based on eventIndecesStock.
	 */
	eventTimesStock = std::vector<scalar_array_t>(numPartitions_, scalar_array_t(0));  // a subsystem might have no switches
	eventCountersStock = std::vector<size_array_t>(numPartitions_, size_array_t{0});  // a subsystem has been assigned at least one system ID
	for (size_t i=0; i<numPartitions_; i++) {

		eventCountersStock[i][0] = currActiveSubsystemIndex;

		for (size_t j=0; j<eventIndecesStock[i].size(); j++) {

			size_t& index = eventIndecesStock[i][j];
			eventTimesStock[i].push_back( allEventTimes[index] );

			// since a switch happened the next switched system should be pushed to this partitions switched systems list
			// unless the switch happened at the end of the time partition
			currActiveSubsystemIndex++;
			if (partitioningTimes[i+1] - eventTimesStock[i].back() > OCS2NumericTraits<scalar_t>::limit_epsilon()) {
				eventCountersStock[i].push_back(currActiveSubsystemIndex);
			}

		} // end of j loop
	}  // end of i loop

	/*
	 * calculates switching times
	 */
	switchingTimesStock.resize(numPartitions_);
	for (size_t i=0; i<numPartitions_; i++) {

		const size_t numSubsystems = eventCountersStock_[i].size();
		scalar_array_t& switchingTimes = switchingTimesStock[i];

		switchingTimes.clear();
		switchingTimes.reserve(1+numSubsystems);

		// add the partition's start time
		switchingTimes.push_back(partitioningTimes_[i]);
		// add the intermediate switching times
		for (const scalar_t& t: eventTimesStock_[i]) {
			switchingTimes.push_back(t);
		}
		// only add the partition's final time if there is no intermediate switching time at the end
		if (switchingTimes.size() == numSubsystems) {
			switchingTimes.push_back(partitioningTimes_[i+1]);
		} else {
			switchingTimes[numSubsystems] = partitioningTimes_[i+1];
		}
	}  // end of i loop

}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class LOGIC_RULES_T>
void LogicRulesMachine<LOGIC_RULES_T>::display() const {

	this->getLogicRulesPtr()->display();

	size_t itr;

	itr = 0;
	std::cerr << "Time partitions:\n\t ";
	for (size_t i=0; i<partitioningTimes_.size()-1; i++) {  // printing
		std::cerr << itr << ":[";
		itr++;
		std::cerr << partitioningTimes_[i] << ", " << partitioningTimes_[i+1];
		std::cerr << "],  ";
	}
	std::cerr << std::endl;

	itr = 0;
	std::cerr << "Switching times distribution for partitions:\n\t ";
	for (const auto& switchingTimes : switchingTimesStock_) {  // printing
		std::cerr << itr << ":{";
		itr++;

		for (const auto& ti : switchingTimes) {
			std::cerr << ti << ", ";
		}
		if (!switchingTimes.empty()) {  std::cerr << "\b\b";
		}
		std::cerr << "},  ";
	}
	std::cerr << std::endl;

	itr = 0;
	std::cerr << "Event times distribution for partitions:\n\t ";
	for (const auto& events : eventTimesStock_) {  // printing
		std::cerr << itr << ":{";
		itr++;

		for (const auto& ti : events) {
			std::cerr << ti << ", ";
		}
		if (!events.empty()) {  std::cerr << "\b\b";
		}
		std::cerr << "},  ";
	}
	std::cerr << std::endl;

	itr = 0;
	std::cerr << "Event counters distribution for partitions:\n\t ";
	for (const auto& eventCounters : eventCountersStock_) {  // printing
		std::cerr << itr << ":{";
		itr++;

		for (const auto& i : eventCounters) {
			std::cerr << i << ", ";
		}
		if (!eventCounters.empty()) {  std::cerr << "\b\b";
		}
		std::cerr << "},  ";
	}
	std::cerr << std::endl;
}

} // namespace ocs2
