/*
 * HybridLogicRulesMachine.h
 *
 *  Created on: Mar 23, 2018
 *      Author: farbod
 */

namespace ocs2{

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class logic_rules_template_t>
void HybridLogicRulesMachine<STATE_DIM, INPUT_DIM, logic_rules_template_t>::setupLogicMachine(
		const scalar_array_t& partitioningTimes,
		const scalar_t& initTime,
		const size_t& initActivePartition,
		const size_t& initSubsystemID) {

	//**********************************************//
	//****************** LogicRule *****************//
	//**********************************************//
	HybridLogicRules<STATE_DIM,INPUT_DIM>& logicRules = BASE::getLogicRules();

	// delete the future event information
	size_t index = std::lower_bound(logicRules.eventTimes().begin(), logicRules.eventTimes().end(), initTime) - logicRules.eventTimes().begin();
	logicRules.eventTimes().erase(logicRules.eventTimes().begin()+index, logicRules.eventTimes().end());
	logicRules.subsystemsSequence().erase(logicRules.subsystemsSequence().begin()+index+1, logicRules.subsystemsSequence().end());

	// if the current subsystem is not the same as expected.
	if (logicRules.subsystemsSequence().empty()) {
		logicRules.subsystemsSequence().push_back(initSubsystemID);

	} else if (logicRules.subsystemsSequence().back()!=initSubsystemID) {
		logicRules.eventTimes().push_back(initTime);
		logicRules.subsystemsSequence().push_back(initSubsystemID);
	}
	// update logicRules
	logicRules.update();

	size_t initEventCounter = logicRules.subsystemsSequence().size()-1;

	//**********************************************//
	//**************** LogicMachine ****************//
	//**********************************************//
	const size_t numPartitionings = partitioningTimes.size()-1;

	BASE::logicRulesModified_ = false;
	BASE::numPartitionings_   = numPartitionings;
	BASE::partitioningTimes_  = partitioningTimes;

	BASE::eventTimesStock_.resize(numPartitionings);
	BASE::eventCountersStock_.resize(numPartitionings);
	BASE::switchingTimesStock_.resize(numPartitionings);

	// for the future partitions
	for (size_t i=initActivePartition+1; i<numPartitionings; i++) {
		BASE::eventTimesStock_[i].clear();
		BASE::eventCountersStock_[i].clear();
		BASE::switchingTimesStock_[i] = scalar_array_t {partitioningTimes[i], partitioningTimes[i+1]};
	}  // end of i loop

	// for the initActivePartition partition
	if (BASE::eventTimesStock_[initActivePartition].empty()==true) {

		BASE::eventTimesStock_[initActivePartition].clear();
		BASE::eventCountersStock_[initActivePartition] = size_array_t {initEventCounter};
		BASE::switchingTimesStock_[initActivePartition] = scalar_array_t {partitioningTimes[initActivePartition], partitioningTimes[initActivePartition+1]};

	} else {
		size_t index = std::lower_bound(BASE::eventTimesStock_[initActivePartition].begin(), BASE::eventTimesStock_[initActivePartition].end(), initTime) -
				BASE::eventTimesStock_[initActivePartition].begin();

		BASE::eventTimesStock_[initActivePartition].erase(
				BASE::eventTimesStock_[initActivePartition].begin()+index, BASE::eventTimesStock_[initActivePartition].end());

		BASE::eventCountersStock_[initActivePartition].erase(
				BASE::eventCountersStock_[initActivePartition].begin()+index+1, BASE::eventCountersStock_[initActivePartition].end());

		BASE::switchingTimesStock_[initActivePartition].erase(
				BASE::switchingTimesStock_[initActivePartition].begin()+index+1, BASE::switchingTimesStock_[initActivePartition].end());

		if (BASE::eventCountersStock_[initActivePartition].back() != initEventCounter) {
			BASE::eventTimesStock_[initActivePartition].push_back(initTime);
			BASE::eventCountersStock_[initActivePartition].push_back(initEventCounter);
			BASE::switchingTimesStock_[initActivePartition].push_back(initTime);
		}
		BASE::switchingTimesStock_[initActivePartition].push_back(partitioningTimes[initActivePartition+1]);
	}
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class logic_rules_template_t>
void HybridLogicRulesMachine<STATE_DIM, INPUT_DIM, logic_rules_template_t>::initLogicMachine(
		const size_t& partitionIndex) {

	if (BASE::eventCountersStock_[partitionIndex].empty()==true) {
		if (partitionIndex==0)
			throw std::runtime_error("ocs2::setupLogicMachine function should have been called before.");
		if (BASE::eventCountersStock_[partitionIndex-1].empty()==true)
			throw std::runtime_error("ocs2::initLogicMachine function should have been called for the previous partition first.");

		BASE::eventCountersStock_[partitionIndex].push_back(BASE::getLogicRules().subsystemsSequence().size()-1);
	}
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class logic_rules_template_t>
void HybridLogicRulesMachine<STATE_DIM, INPUT_DIM, logic_rules_template_t>::push_back(
		const size_t& partitionIndex,
		const scalar_t& eventTime,
		const size_t& subsystemID) {

	BASE::getLogicRules().eventTimes().push_back(eventTime);
	BASE::getLogicRules().subsystemsSequence().push_back(subsystemID);
	BASE::getLogicRules().update();

	BASE::eventTimesStock_[partitionIndex].push_back(eventTime);

	BASE::eventCountersStock_[partitionIndex].push_back(BASE::getLogicRules().subsystemsSequence().size()-1);

	BASE::switchingTimesStock_[partitionIndex].push_back(BASE::switchingTimesStock_[partitionIndex].back());
	*(BASE::switchingTimesStock_[partitionIndex].end()-2) = eventTime;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class logic_rules_template_t>
void HybridLogicRulesMachine<STATE_DIM, INPUT_DIM, logic_rules_template_t>::display() const {

	BASE::display();

	size_t itr = 0;
	std::cerr << "Event Subsystem ID distribution for partitions:\n\t ";
	for (auto& eventCounters : BASE::eventCountersStock_) {  // printing
		std::cerr << itr << ":{";
		itr++;

		for (auto& i : eventCounters) {
			std::cerr << BASE::getLogicRules().subsystemsSequence()[i] << ", ";
		}
		if (!eventCounters.empty())  std::cerr << "\b\b";
		std::cerr << "},  ";
	}
	std::cerr << std::endl;
}

} // namespace ocs2


