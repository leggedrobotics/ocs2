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

#ifndef EVENTTIMEINDEXER_OCS2_H_
#define EVENTTIMEINDEXER_OCS2_H_

#include <vector>
#include <utility>
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

	typedef std::pair<size_t, size_t>            range_t;
	typedef std::pair<int, range_t>              partition_range_t;
	typedef std::vector<partition_range_t>       partition_range_array_t;
	typedef std::vector<partition_range_array_t> partition_range_array2_t;

	/**
	 * Default constructor.
	 */
	EventTimeIndexer()
	: numEventTimes_(0)
	, numSubsystems_(numEventTimes_+1)
	, numPartitions_(1)
	, numSubsystemsStock_{1}
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
	 * @param [in] logicRulesMachine: Instance of the LogicRulesMachine.
	 */
	void set(const LogicRulesMachine& logicRulesMachine);

	/**
	 * Gets the total number of events.
	 *
	 * @return The total number of events.
	 */
	const size_t& numEventTimes() const;

	/**
	 * Gets the total number of subsystems.
	 *
	 * @return The total number of subsystems.
	 */
	const size_t& numSubsystems() const;

	/**
	 * Gets the number of subsystems in the given partition.
	 *
	 * @param partitionIndex: Partition index.
	 * @return The number of subsystems in the partition.
	 */
	const size_t& numPartitionSubsystems(const size_t& partitionIndex) const;

	/**
	 * Gets the distribution of the partitions for each subsystem.
	 *
	 * @param [in] subsystemIndex: Index of the subsystem.
	 * @return The distribution of the partitions for each subsystem
	 */
	const int_array_t& partitionsDistribution(const size_t& subsystemIndex) const;

	/**
	 * Gets the concatenated indices of the subsystem over different partitions.
	 *
	 * @param [in] subsystemIndex: Index of the subsystem.
	 * @return The concatenated Indices of the subsystem over different partitions.
	 */
	const size_array_t& subsystemIndeces(const size_t& subsystemIndex) const;

	/**
	 * Finds the active time intervals of a subsystem across time partitions.
	 *
	 * @param [in] subsystemIndex: Index of the subsystem.
	 * @param [in] timeTrajectoriesStock: Array of trajectories containing the time trajectory stamp of a roll-out.
	 * @param [in] eventsPastTheEndIndecesStock: Array of indices containing past-the-end index of events trigger.
	 * @param [out] subsystemRange: Array of partition_range_t which indicates the partitions on which the
	 * subsystem is active. Moreover it determines the begin and the end indices of the partition's time intervals
	 * where the subsystem is active.
	 *
	 */
	template <typename SCALAR_T>
	void findSubsystemActiveTimeIntervals(
			const size_t& subsystemIndex,
			const std::vector<std::vector<SCALAR_T>>& timeTrajectoriesStock,
			const size_array2_t& eventsPastTheEndIndecesStock,
			partition_range_array_t& subsystemRange) const;

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
	 * @param [out] subsystemsIndeces: The concatenated indices of the subsystems over different partitions.
	 */
	void findPartitionsDistribution(
			const size_t& numSubsystems,
			const size_array2_t& eventCountersStock,
			int_array2_t& partitionsDistribution,
			size_array2_t& subsystemsIndeces) const;

private:
	size_t numEventTimes_;
	size_t numSubsystems_;
	size_t numPartitions_;

	size_array_t numSubsystemsStock_;

	size_array2_t subsystemsIndeces_;
	int_array2_t partitionsDistribution_;
};

} // namespace ocs2

#include "implementation/EventTimeIndexer.h"

#endif /* EVENTTIMEINDEXER_OCS2_H_ */


