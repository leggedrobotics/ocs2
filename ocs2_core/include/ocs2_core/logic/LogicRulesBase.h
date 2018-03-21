/*
 * LogicRulesBase.h
 *
 *  Created on: Nov 28, 2017
 *      Author: farbod
 */

#ifndef OGICRULESBASE_OCS2_H_
#define OGICRULESBASE_OCS2_H_

#include <memory>
#include <Eigen/StdVector>
#include <Eigen/Dense>
#include <vector>

#include "ocs2_core/Dimensions.h"

namespace ocs2{

/**
 * Null logic rules template
 */
class NullLogicRulesTemplate {};

/**
 * Logic rules base class
 */
template <size_t STATE_DIM, size_t INPUT_DIM, class logic_rules_template_t = NullLogicRulesTemplate>
class LogicRulesBase
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef logic_rules_template_t LogicRulesTemplate;

	typedef typename Dimensions<STATE_DIM, INPUT_DIM>::size_array_t size_array_t;
	typedef typename Dimensions<STATE_DIM, INPUT_DIM>::scalar_t scalar_t;
	typedef typename Dimensions<STATE_DIM, INPUT_DIM>::scalar_array_t scalar_array_t;
	typedef typename Dimensions<STATE_DIM, INPUT_DIM>::controller_t controller_t;
	typedef typename Dimensions<STATE_DIM, INPUT_DIM>::controller_array_t controller_array_t;

	/**
	 * Default constructor
	 */
	LogicRulesBase()
	: eventTimes_(0)
	{}

	/**
	 * Constructor
	 */
	LogicRulesBase(const scalar_array_t& eventTimes)
	: eventTimes_(eventTimes)
	{}

	/**
	 * Copy constructor
	 */
	LogicRulesBase(const LogicRulesBase& rhs)
	: eventTimes_(rhs.eventTimes_)
	{}

	/**
	 * Destructor
	 */
	virtual ~LogicRulesBase() = default;

	/**
	 * Move assignment
	 */
	LogicRulesBase& operator=(LogicRulesBase&& other) = default;

	/**
	 * Assignment
	 */
	LogicRulesBase& operator=(const LogicRulesBase& other) = default;

	/**
	 * Rewinds the class. This methid is only called in the MPC class.
	 *
	 * @param [in] lowerBoundTime: The smallest time for which the logicRules should be defined.
	 * @param [in] upperBoundTime: The greates time for which the logicRules should be defined.
	 */
	virtual void rewind(
			const scalar_t& lowerBoundTime,
			const scalar_t& upperBoundTime) = 0;

	/**
	 * Used in the SLQ-MPC method to set a new user defined logic.
	 *
	 * @param [in] lowerBoundTime: The smallest time for which the new logicRules template can become effective.
	 * @param [in] upperBoundTime: The greates time for which the new logicRules template should be defined.
	 * @param [in] logicRulesTemplate: A dada type which includes all necessary information for modifying the logicRules.
	 */
	virtual void setLogicRulesTemplate(
			const scalar_t& lowerBoundTime,
			const scalar_t& upperBoundTime,
			const LogicRulesTemplate& logicRulesTemplate) {}

	/**
	 * Adjusts controller. This methos is called my the logicMachine whenever the logicRuls are updated.
	 * It allows the user to modify the controller to adapt to the changes of logics.
	 *
	 * @param controller: Control policy which should be adjusted.
	 */
	virtual void adjustController(controller_t& controller) const = 0;

	/**
	 * Gets the event times.
	 *
	 * @return A const reference to eventTimes_.
	 */
	const scalar_array_t& eventTimes() const {
		return eventTimes_;
	}

	/**
	 * Gets the event times.
	 *
	 * @return A const reference to eventTimes_.
	 */
	scalar_array_t& eventTimes() {
		return eventTimes_;
	}

protected:
	scalar_array_t eventTimes_;
};


/**
 * Null logic rules class
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class NullLogicRules : public LogicRulesBase<STATE_DIM, INPUT_DIM, NullLogicRulesTemplate>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef LogicRulesBase<STATE_DIM, INPUT_DIM, NullLogicRulesTemplate> BASE;

	typedef NullLogicRulesTemplate LogicRulesTemplate;

	typedef typename BASE::size_array_t size_array_t;
	typedef typename BASE::scalar_t scalar_t;
	typedef typename BASE::scalar_array_t scalar_array_t;
	typedef typename BASE::controller_t controller_t;
	typedef typename BASE::controller_array_t controller_array_t;

	/**
	 * Constructor
	 */
	NullLogicRules()
	: BASE()
	{}

	/**
	 * Copy constructor
	 */
	NullLogicRules(const NullLogicRules& rhs) = default;

	/**
	 * Destructor
	 */
	~NullLogicRules() = default;

	/**
	 * Move assignment
	 */
	NullLogicRules& operator=(NullLogicRules&& other) = default;

	/**
	 * Assignment
	 */
	NullLogicRules& operator=(const NullLogicRules& other) = default;

	/**
	 * Adjusts controller. This methos is called my the logicMachine whenever the logicRuls are updated.
	 * Here no adjustments take place.
	 *
	 * @param controller: Control policy which should be adjusted.
	 */
	void adjustController(controller_t& controller) const override
	{}

private:

};

} // namespace ocs2

#endif /* OGICRULESBASE_OCS2_H_ */
