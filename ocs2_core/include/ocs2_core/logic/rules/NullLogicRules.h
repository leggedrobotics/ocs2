/*
 * NullLogicRules.h
 *
 *  Created on: Mar 29, 2018
 *      Author: farbod
 */

#ifndef NULLLOGICRULES_OCS2_H_
#define NULLLOGICRULES_OCS2_H_

#include <memory>
#include <Eigen/StdVector>
#include <Eigen/Dense>
#include <vector>

#include "ocs2_core/Dimensions.h"
#include "ocs2_core/logic/rules/LogicRulesBase.h"

namespace ocs2{

/**
 * Null logic rules class
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class NullLogicRules : public LogicRulesBase<STATE_DIM, INPUT_DIM>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef LogicRulesBase<STATE_DIM, INPUT_DIM> BASE;

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
	 * Rewinds the class. This methid is only called in the MPC class.
	 *
	 * @param [in] lowerBoundTime: The smallest time for which the logicRules should be defined.
	 * @param [in] upperBoundTime: The greates time for which the logicRules should be defined.
	 */
	virtual void rewind(
			const scalar_t& lowerBoundTime,
			const scalar_t& upperBoundTime) override
	{}

	/**
	 * Adjusts controller. This methos is called my the logicMachine whenever the logicRuls are updated.
	 * Here no adjustments take place.
	 *
	 * @param controller: Control policy which should be adjusted.
	 */
	virtual void adjustController(controller_t& controller) const override
	{}

	/**
	 * This method can be used to update the internal variables. This method will be called by any
	 * program that trys to update the logic rules variables.
	 */
	virtual void update() override
	{}

private:

};

} // namespace ocs2

#endif /* NULLLOGICRULES_OCS2_H_ */
