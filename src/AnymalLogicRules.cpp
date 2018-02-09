/*
 * AnymalLogicRules.cpp
 *
 *  Created on: Feb 9, 2018
 *      Author: farbod
 */

#include "ocs2_anymal_switched_model/logic/AnymalLogicRules.h"

namespace anymal {

AnymalLogicRules::AnymalLogicRules(const scalar_t& swingLegLiftOff /*= 0.15*/,
		const scalar_t& swingTimeScale /*= 1.0*/)

: Base(swingLegLiftOff, swingTimeScale)
{}

AnymalLogicRules::AnymalLogicRules(const AnymalLogicRules& rhs)
: Base(rhs)
{}

} // end of namespace anymal


