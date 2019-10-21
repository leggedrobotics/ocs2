/*
 * AnymalComKinoOperatingPoints.cpp
 *
 *  Created on: Feb 9, 2018
 *      Author: farbod
 */

#include "ocs2_anymal_switched_model/initialization/AnymalComKinoOperatingPoints.h"

#include <ocs2_anymal_switched_model/core/AnymalCom.h>

namespace anymal {

AnymalComKinoOperatingPoints::AnymalComKinoOperatingPoints(
		std::shared_ptr<const logic_rules_t> logicRulesPtr)
: Base(AnymalCom(), std::move(logicRulesPtr))
{}

AnymalComKinoOperatingPoints::AnymalComKinoOperatingPoints(const AnymalComKinoOperatingPoints& rhs)
: Base(rhs)
{}

AnymalComKinoOperatingPoints* AnymalComKinoOperatingPoints::clone() const {

	return new AnymalComKinoOperatingPoints(*this);
}

} //end of namespace anymal


