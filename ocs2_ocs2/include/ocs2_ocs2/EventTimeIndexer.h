/*
 * EventTimeIndexer.h
 *
 *  Created on: Jul 2, 2018
 *      Author: farbod
 */

#ifndef EVENTTIMEINDEXER_OCS2_H_
#define EVENTTIMEINDEXER_OCS2_H_

#include <vector>

#include <ocs2_core/logic/machine/LogicRulesMachine.h>

namespace ocs2{

class EventTimeIndexer
{
public:
	typedef std::vector<size_t>       size_array_t;
	typedef std::vector<size_array_t> size_array2_t;

	EventTimeIndexer()
	: numEventTimes_(0)
    , dataEndIterator_(0)
	, dataEndPartition_(0)
	, dataBeginIterator_(0)
	, dataBeginPartition_(0)
	, initActiveEventTime_(0)
	, finalActiveEventTime_(0)
	{}

	~EventTimeIndexer() = default;

	const size_t& dataEndIterator(const size_t& eventTimeIndex) const {
		checkIndex(eventTimeIndex);
		return dataEndIterator_[eventTimeIndex];
	}

	const size_t& dataEndPartition(const size_t& eventTimeIndex) const {
		checkIndex(eventTimeIndex);
		return dataEndPartition_[eventTimeIndex];
	}

	const size_t& dataBeginIterator(const size_t& eventTimeIndex) const {
		checkIndex(eventTimeIndex);
		return dataBeginIterator_[eventTimeIndex];
	}

	const size_t& dataBeginPartition(const size_t& eventTimeIndex) const {
		checkIndex(eventTimeIndex);
		return dataBeginPartition_[eventTimeIndex];
	}

	/**
	 *
	 *
	 * @tparam SCALAR_T: Scalar type.
	 * @tparam STATE_DIM: Dimension of the state space.
	 * @tparam INPUT_DIM: Dimension of the control input space.
	 * @tparam LOGIC_RULES_T: Logic Rules type (default NullLogicRules).
	 */
	template <typename SCALAR_T, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
	void set(
			const SCALAR_T& initTime,
			const SCALAR_T& finalTime,
			const size_t& initActivePartition,
			const size_t& finalActivePartition,
			const size_array2_t& eventsPastTheEndIndecesStock,
			const LogicRulesMachine<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>& logicRulesMachine) {

		// number of time partitions
		const size_t& numPartitioningTimes = logicRulesMachine.getNumPartitioningTimes();

		// active events lower range
		auto findInitActiveEventCounter = logicRulesMachine.getHandleToFindActiveEventCounter(
				initActivePartition);
		const size_t initActiveEvent = findInitActiveEventCounter(initTime);
		// active evets upper range
		auto findFinalActiveEventCounter = logicRulesMachine.getHandleToFindActiveEventCounter(
				finalActivePartition);
		const size_t finalActiveEvent = findInitActiveEventCounter(finalTime);

		// total number of time events
		numEventTimes_ = 0;
		for (size_t i=0; i<numPartitioningTimes; i++)
			numEventTimes_ += logicRulesMachine.getNumEvents(i);

		// resizing
		dataEndIterator_.resize(numEventTimes_);
		dataEndPartition_.resize(numEventTimes_);
		dataBeginIterator_.resize(numEventTimes_);
		dataBeginPartition_.resize(numEventTimes_);

		size_t partitionIndex = 0;

		for (size_t j=0; j+1<numEventTimes_; j++) {

			// begining of the subsystem
			size_t index = 0;
			for (size_t i=partitionIndex; i<numPartitioningTimes; i++)
				for (size_t k=0; k<logicRulesMachine.getEventCounters(i).size(); k++)
					if (logicRulesMachine.getEventCounters(i)[k]==j) {
						partitionIndex = i;
						index = k;
						break;
					}

			dataBeginPartition_[j] = partitionIndex;
			if (initActivePartition<=partitionIndex && partitionIndex<=finalActivePartition)
				dataBeginIterator_[j] = eventsPastTheEndIndecesStock[partitionIndex][index];

			// end of the subsystem
			if (index+1<logicRulesMachine.getEventCounters(partitionIndex).size()) {
				dataEndPartition_[j] = partitionIndex;
				dataEndIterator_[j] = eventsPastTheEndIndecesStock[partitionIndex][index+1];

			} else {
				for (size_t i=partitionIndex+1; i<numPartitioningTimes; i++)
					for (size_t k=0; k<logicRulesMachine.getEventCounters(i).size(); k++)
						if (logicRulesMachine.getEventCounters(i)[k]==j) {
							partitionIndex = i;
							break;
						}
			}

			dataEndPartition_[j] = partitionIndex;
			dataEndIterator_[j] = eventsPastTheEndIndecesStock[partitionIndex][0];
		}  // end of j loop

	}

protected:
	void checkIndex(const size_t& eventTimeIndex) const {
		if (eventTimeIndex+1==numEventTimes_)
			throw std::runtime_error("The final event time is not accessible in EventTimeIterator.");
	}

private:
	size_t numEventTimes_;

	size_t dataEndIterator_;
	size_t dataEndPartition_;
	size_t dataBeginIterator_;
	size_t dataBeginPartition_;

	size_t initActiveEventTime_;
	size_t finalActiveEventTime_;

	std::function<size_t(scalar_t)>
	std::function<size_t(scalar_t)>
};

} // namespace ocs2

#endif /* EVENTTIMEINDEXER_OCS2_H_ */


