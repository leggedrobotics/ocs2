/*
 * AnymalLogicRules.h
 *
 *  Created on: Nov 28, 2017
 *      Author: farbod
 */

#ifndef ANYMAL_LOGICRULES
#define ANYMAL_LOGICRULES

#include <c_switched_model_interface/logic/SwitchedModelLogicRulesBase.h>

namespace anymal {

class AnymalLogicRules : public switched_model::SwitchedModelLogicRulesBase<12>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef switched_model::SwitchedModelLogicRulesBase<12> Base;

	AnymalLogicRules(const scalar_t& swingLegLiftOff = 0.15,
			const scalar_t& swingTimeScale = 1.0);

	AnymalLogicRules(const AnymalLogicRules& rhs);

	~AnymalLogicRules()
	{}

};

} // namespace anymal

#endif /* ANYMAL_LOGICRULES */
