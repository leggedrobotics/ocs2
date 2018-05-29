/*
 * HybridLogicRulesMachine.h
 *
 *  Created on: Mar 23, 2018
 *      Author: farbod
 */

#ifndef HYBRIDLOGICRULESMACHINE_OCS2_H_
#define HYBRIDLOGICRULESMACHINE_OCS2_H_

#include <algorithm>

#include "ocs2_core/logic/machine/LogicRulesMachine.h"
#include "ocs2_core/logic/rules/HybridLogicRules.h"

namespace ocs2{

/**
 * This class allows to construct the Logic Rules Machine on fly. This class is employed in the State-Triggered SLQ forward pass.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 * @tparam LOGIC_RULES_T: Logic Rules type.
 */
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
class HybridLogicRulesMachine : public LogicRulesMachine<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	static_assert(std::is_base_of<HybridLogicRules<STATE_DIM, INPUT_DIM>, LOGIC_RULES_T>::value,
			"LOGIC_RULES_T must inherit from HybridLogicRules");

	typedef LogicRulesMachine<STATE_DIM, INPUT_DIM, LOGIC_RULES_T> BASE;
	typedef typename BASE::scalar_t 		scalar_t;
	typedef typename BASE::scalar_array_t 	scalar_array_t;
	typedef typename BASE::size_array_t 	size_array_t;

	/**
	 * Default constructor
	 */
	HybridLogicRulesMachine()
	: BASE()
	{}

	/**
	 * Constructor.
	 *
	 * @param logicRules: The logic rules class.
	 */
	HybridLogicRulesMachine(const LOGIC_RULES_T& logicRules)
	: BASE(logicRules)
	{}

	/**
	 * Default destructor.
	 */
	virtual ~HybridLogicRulesMachine() = default;

	/**
	 * Copy constructor
	 */
	HybridLogicRulesMachine(const HybridLogicRulesMachine& rhs) = default;

	/**
	 * Move assignment
	 */
	HybridLogicRulesMachine& operator=(HybridLogicRulesMachine&& other) = default;

	/**
	 * Assignment
	 */
	HybridLogicRulesMachine& operator=(const HybridLogicRulesMachine& other) = default;

	/**
	 * Displays switched systems distribution over the time partitions.
	 */
	virtual void display() const override;

	/**
	 * Sets up the LogicMachine. This method should be called before building-up the logic machine.
	 *
	 * @param [in] partitioningTimes: The time partition.
	 * @param [in] initTime: The initial time.
	 * @param [in] initActivePartition: The initial active time partition.
	 * @param [in] initSubsystemID: The initial active subsystem's ID.
	 */
	void setupLogicMachine(
			const scalar_array_t& partitioningTimes,
			const scalar_t& initTime,
			const size_t& initActivePartition,
			const size_t& initSubsystemID);

	/**
	 * This method should be called for each partiton once, before calling push_back method
	 *
	 * @param [in] partitionIndex: The current active time partition.
	 */
	void initLogicMachine(const size_t& partitionIndex);

	/**
	 *  Adds a new event at the end of the current logic ruls.
	 *
	 * @param [in] partitionIndex: The current active time partition.
	 * @param [in] eventTime: The time of the new event.
	 * @param [in] subsystemID: The triggered subsystem
	 */
	void push_back(const size_t& partitionIndex,
			const scalar_t& eventTime,
			const size_t& subsystemID);

protected:

};

} // namespace ocs2

#include "implementation/HybridLogicRulesMachine.h"

#endif /* HYBRIDLOGICRULESMACHINE_OCS2_H_ */
