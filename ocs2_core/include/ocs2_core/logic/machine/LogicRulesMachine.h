/*
 * LogicRulesMachine.h
 *
 *  Created on: Nov 28, 2017
 *      Author: farbod
 */

#ifndef LOGICRULESMACHINE_OCS2_H_
#define LOGICRULESMACHINE_OCS2_H_

#include <utility>
#include <memory>
#include <iostream>
#include <string>
#include <functional>
#include <type_traits>

#include "ocs2_core/Dimensions.h"
#include "ocs2_core/OCS2NumericTraits.h"
#include "ocs2_core/misc/FindActiveIntervalIndex.h"
#include "ocs2_core/logic/rules/LogicRulesBase.h"

namespace ocs2{

/**
 * The logic rules machine class
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 * @tparam LOGIC_RULES_T: logical rule type.
 */
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
class LogicRulesMachine
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	static_assert(std::is_base_of<LogicRulesBase<STATE_DIM, INPUT_DIM>, LOGIC_RULES_T>::value,
			"LOGIC_RULES_T must inherit from LogicRulesBase");

	typedef std::shared_ptr<LogicRulesMachine<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>> Ptr;

	typedef Dimensions<STATE_DIM, INPUT_DIM> 		DIMENSIONS;
	typedef typename DIMENSIONS::size_array_t 		size_array_t;
	typedef typename DIMENSIONS::scalar_t		 	scalar_t;
	typedef typename DIMENSIONS::scalar_array_t 	scalar_array_t;
	typedef typename DIMENSIONS::controller_t 		controller_t;
	typedef typename DIMENSIONS::controller_array_t controller_array_t;

	/**
	 * Default constructor.
	 */
	LogicRulesMachine()
	: logicRulesModified_(false),
	  logicRulesModifiedOffline_(false),
	  numPartitionings_(1),
	  partitioningTimes_{-10000,10000},
	  eventTimesStock_(1,scalar_array_t(0)),
	  switchingTimesStock_(1,partitioningTimes_),
	  eventCountersStock_(1,size_array_t(1,0))
	{}

	/**
	 * Constructor.
	 *
	 * @param logicRules: The logic rules class.
	 */
	LogicRulesMachine(const LOGIC_RULES_T& logicRules)
	: logicRules_(logicRules),
	  logicRulesInUse_(logicRules),
	  logicRulesModified_(false),
	  logicRulesModifiedOffline_(false),
	  numPartitionings_(1),
	  partitioningTimes_{-10000,10000},
	  eventTimesStock_(1,scalar_array_t(0)),
	  switchingTimesStock_(1,partitioningTimes_),
	  eventCountersStock_(1,size_array_t(1,0))
	{}

	/**
	 * Default destructor.
	 */
	virtual ~LogicRulesMachine() = default;

	/**
	 * Copy constructor
	 */
	LogicRulesMachine(const LogicRulesMachine& rhs) = default;

	/**
	 * Move assignment
	 */
	LogicRulesMachine& operator=(LogicRulesMachine&& other) = default;

	/**
	 * Assignment
	 */
	LogicRulesMachine& operator=(const LogicRulesMachine& other) = default;

	/**
	 * Sets a new logic rules class.
	 *
	 * @param [in] logicRules: The new logic rules class
	 */
	void setLogicRules(const LOGIC_RULES_T& logicRules);

	/**
	 * This causes that logicMachine updates itself in the next call of the SLQ::run().
	 *
	 */
	void logicRulesUpdated();

	/**
	 * Get the active logic rules class
	 * @return active logic rules class
	 */
	LOGIC_RULES_T& getLogicRules();

	/**
	 * Get the active logic rules class
	 * @return active logic rules class
	 */
	const LOGIC_RULES_T& getLogicRules() const;

	/**
	 * Get the pointer to the active logic rules class
	 *
	 * @return pointer to active logic rules class
	 */
	LOGIC_RULES_T* getLogicRulesPtr();

	/**
	 * Get the pointer to the active logic rules class
	 *
	 * @return pointer to active logic rules class
	 */
	const LOGIC_RULES_T* getLogicRulesPtr() const;

	/**
	 * Gets the event times associated to the partition number index.
	 *
	 * @param [in] partitionIndex: index of the time partition.
	 * @return
	 */
	const scalar_array_t& getEventTimes(size_t partitionIndex) const;

	/**
	 * Gets the event counters associated to the partition number index.
	 * @param [in] partitionIndex: index of the time partition
	 * @return
	 */
	const size_array_t& getEventCounters(size_t partitionIndex) const;

	/**
	 * Gets the switching times associated to the partition number index.
	 * @param [in] index: index of the time partition
	 * @return
	 */
	const scalar_array_t& getSwitchingTimes(size_t index) const;

	/**
	 * Gets the partitioning times.
	 * @return const reference to partitioning times.
	 */
	const scalar_array_t& getPartitioningTimes() const;

	/**
	 * Returns the number of event counters in the partition.
	 *
	 * @param partitionIndex: index of the time partition
	 * @return Number of event counters.
	 */
	size_t getNumEventCounters(size_t partitionIndex) const;

	/**
	 * Returns the number of event in the partition.
	 *
	 * @param partitionIndex: index of the time partition
	 * @return Number of events
	 */
	size_t getNumEvents(size_t partitionIndex) const;

	/**
	 * Returns a Lambda expression which can be used to find the current active event counter.
	 *
	 * @param partitionIndex: index of the time partition
	 * @return Lambda expression
	 */
	std::function<size_t(scalar_t)> getHandleToFindActiveEventCounter(size_t partitionIndex) const;

	/**
	 * Updates the active logic rules based on the last set value (using LogicRulesMachine::setLogicRules) and
	 * adjusts the controller based on LOGIC_RULES_T::adjustController() routine. Moreover, it recomputes the
	 * distribution of the switched systems over the time partitions.
	 *
	 * @param [in] partitioningTimes: Vector of time partitions.
	 * @param [out] controllerStock: The controller which will be adjusted.
	 */
	virtual void updateLogicRules(
			const scalar_array_t& partitioningTimes,
			controller_array_t& controllerStock);

	/**
	 * Find distribution of the events over the time partitions.
	 *
	 * @param [in] partitioningTimes: Vector of time partitions.
	 * @param [out] eventTimesStock: Distribution of the event times over partitions.
	 * @param [out] switchingTimesStock: Distribution of the switching times over partitions.
	 * @param [out] eventCountersStock: Distribution of the event counter over partitions.
	 */
	void findEventsDistribution(
			const scalar_array_t& partitioningTimes,
			std::vector<scalar_array_t>& eventTimesStock,
			std::vector<scalar_array_t>& switchingTimesStock,
			std::vector<size_array_t>& eventCountersStock);

	/**
	 * Displays switched systems distribution over the time partitions.
	 */
	virtual void display() const;


protected:
	LOGIC_RULES_T logicRules_;
	LOGIC_RULES_T logicRulesInUse_;
	bool logicRulesModified_;
	bool logicRulesModifiedOffline_;

	size_t numPartitionings_;
	scalar_array_t partitioningTimes_;

	std::vector<scalar_array_t> eventTimesStock_;
	std::vector<scalar_array_t> switchingTimesStock_;
	std::vector<size_array_t> eventCountersStock_;
};

} // namespace ocs2

#include "implementation/LogicRulesMachine.h"

#endif /* LOGICRULESMACHINE_OCS2_H_ */
