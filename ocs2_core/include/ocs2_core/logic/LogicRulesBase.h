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
 * Logic rules base class
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class LogicRulesBase
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef typename Dimensions<STATE_DIM, INPUT_DIM>::size_array_t size_array_t;
	typedef typename Dimensions<STATE_DIM, INPUT_DIM>::scalar_t scalar_t;
	typedef typename Dimensions<STATE_DIM, INPUT_DIM>::scalar_array_t scalar_array_t;
	typedef typename Dimensions<STATE_DIM, INPUT_DIM>::controller_t controller_t;
	typedef typename Dimensions<STATE_DIM, INPUT_DIM>::controller_array_t controller_array_t;

	LogicRulesBase() {}

	virtual ~LogicRulesBase() {}


	virtual void adjustController(controller_t& controller) const = 0;


	const scalar_array_t& logicRulesSwitchingTimes() const {
		return logicRulesSwitchingTimes_;
	}

protected:
	scalar_array_t logicRulesSwitchingTimes_;
};


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

	NullLogicRules()
	: BASE()
	{}

	~NullLogicRules() {}

	void adjustController(controller_t& controller) const override {}

private:

};

} // namespace ocs2

#endif /* OGICRULESBASE_OCS2_H_ */
