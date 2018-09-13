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

#ifndef LOGICRULESMACHINE_OCS2_H_
#define LOGICRULESMACHINE_OCS2_H_

#include <utility>
#include <memory>
#include <atomic>
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
 * The logic rules machine class.
 * Note that if logic rules are modified through get methods (e.g. getLogicRules),
 * user should call logicRulesUpdated(); otherwise the changes may not become effective.
 *
 * @tparam LOGIC_RULES_T: logical rule type.
 */
template <class LOGIC_RULES_T>
class LogicRulesMachine
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	static_assert(std::is_base_of<LogicRulesBase, LOGIC_RULES_T>::value,
			"LOGIC_RULES_T must inherit from LogicRulesBase");

	typedef std::shared_ptr<LogicRulesMachine<LOGIC_RULES_T>> Ptr;

	typedef Dimensions<0, 0> DIMENSIONS;
	typedef DIMENSIONS::size_array_t   size_array_t;
	typedef DIMENSIONS::scalar_t       scalar_t;
	typedef DIMENSIONS::scalar_array_t scalar_array_t;

	/**
	 * Default constructor.
	 */
	LogicRulesMachine()
	: logicRulesModified_(false)
	, newLogicRulesInBuffer_(false)
	, numPartitions_(1)
	, partitioningTimes_{-10000,10000}
	, eventTimesStock_(1,scalar_array_t(0))
	, switchingTimesStock_(1,partitioningTimes_)
	, eventCountersStock_(1,size_array_t{0})
	{}

	/**
	 * Constructor.
	 *
	 * @param logicRules: The logic rules class.
	 */
	LogicRulesMachine(const LOGIC_RULES_T& logicRules)
	: logicRules_(logicRules)
	, logicRulesBuffer_(logicRules)
	, logicRulesModified_(false)
	, newLogicRulesInBuffer_(false)
	, numPartitions_(1)
	, partitioningTimes_{-10000,10000}
	, eventTimesStock_(1,scalar_array_t(0))
	, switchingTimesStock_(1,partitioningTimes_)
	, eventCountersStock_(1,size_array_t{0})
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
	 * @return constant reference to partitioning times.
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
	 * Returns the number of the partitions.
	 *
	 * @return Number of the partitions.
	 */
	const size_t& getnumPartitions() const;

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
	 * @return true if the logic rules was actually updated.
	 */
	virtual bool updateLogicRules(
			const scalar_array_t& partitioningTimes);

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
	LOGIC_RULES_T logicRulesBuffer_;
	std::atomic<bool> logicRulesModified_;
	std::atomic<bool> newLogicRulesInBuffer_;

	size_t numPartitions_;
	scalar_array_t partitioningTimes_;

	std::vector<scalar_array_t> eventTimesStock_;
	std::vector<scalar_array_t> switchingTimesStock_;
	std::vector<size_array_t> eventCountersStock_;

public:
	friend class EventTimeIndexer;
};

} // namespace ocs2

#include "implementation/LogicRulesMachine.h"

#endif /* LOGICRULESMACHINE_OCS2_H_ */
