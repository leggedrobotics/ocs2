/*
 * EventTimeIndexer.h
 *
 *  Created on: Jul 2, 2018
 *      Author: farbod
 */

#ifndef EVENTTIMEINDEXER_OCS2_H_
#define EVENTTIMEINDEXER_OCS2_H_

#include <vector>
#include <algorithm>

#include <ocs2_core/logic/machine/LogicRulesMachine.h>

namespace ocs2{

class EventTimeIndexer
{
public:
	typedef std::vector<size_t>       size_array_t;
	typedef std::vector<size_array_t> size_array2_t;
	typedef std::vector<int>          int_array_t;
	typedef std::vector<int_array_t>  int_array2_t;

	/**
	 * Default constructor.
	 */
	EventTimeIndexer()
	: numEventTimes_(0)
	, numSubsystems_(numEventTimes_+1)
    , subsystemsIndeces_(1, size_array_t{0})
	, partitionsDistribution_(1, int_array_t{0})
	{}

	/**
	 * Default destructor.
	 */
	~EventTimeIndexer() = default;

	/**
	 * Sets the class.
	 *
	 * @tparam STATE_DIM: Dimension of the state space.
	 * @tparam INPUT_DIM: Dimension of the control input space.
	 * @tparam LOGIC_RULES_T: logical rule type.
	 * @param [in] logicRulesMachine: instance of the LogicRulesMachine.
	 */
	template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
	void set(const LogicRulesMachine<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>& logicRulesMachine);

	/**
	 * Gets the total number of events.
	 *
	 * @return total number of events.
	 */
	const size_t& numEventTimes() const;

	/**
	 * Gets the total number of subsystems.
	 *
	 * @return total number of subsystems.
	 */
	const size_t& numSubsystems() const;

	/**
	 * Gets the distribution of the partitions for each subsystem.
	 *
	 * @param [in] subsystemIndex: index of the subsystem.
	 * @return The distribution of the partitions for each subsystem
	 */
	const int_array_t& partitionsDistribution(const size_t& subsystemIndex) const;

	/**
	 * Gets the concatenated indeces of the subsystem over different partitions.
	 *
	 * @param [in] subsystemIndex: index of the subsystem.
	 * @return The concatenated indeces of the subsystem over different partitions.
	 */
	const size_array_t& subsystemIndeces(const size_t& subsystemIndex) const;

	/**
	 * Displays time partitions distribution over subsystems.
	 */
	void display() const;

protected:
	/**
	 * Finds the distribution of the partitions for each subsystem.
	 *
	 * @param [in] numSubsystems: The total number of subsystems
	 * @param [in] eventCountersStock: The distribution of the subsystems for each partition.
	 * @param [out] partitionsDistribution: The distribution of the partitions for each subsystem.
	 * @param [out] subsystemsIndeces: The concatenated indeces of the subsystems over different partitions.
	 */
	void findPartitionsDistribution(
			const size_t& numSubsystems,
			const size_array2_t& eventCountersStock,
			int_array2_t& partitionsDistribution,
			size_array2_t& subsystemsIndeces) const;

private:
	size_t numEventTimes_;
	size_t numSubsystems_;

	size_array2_t subsystemsIndeces_;
	int_array2_t partitionsDistribution_;
};

} // namespace ocs2

#include "implementation/EventTimeIndexer.h"

#endif /* EVENTTIMEINDEXER_OCS2_H_ */


