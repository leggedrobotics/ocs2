/*
 * AnymalComKinoOperatingPoints.cpp
 *
 *  Created on: Feb 9, 2018
 *      Author: farbod
 */

#include "ocs2_anymal_switched_model/initialization/AnymalComKinoOperatingPoints.h"

namespace anymal {

AnymalComKinoOperatingPoints::AnymalComKinoOperatingPoints(
		std::shared_ptr<const logic_rules_t> logicRulesPtr,
		const switched_model::Model_Settings& options,
		const generalized_coordinate_t& defaultConfiguration)
: Base(AnymalKinematics(), AnymalCom(), std::move(logicRulesPtr), options, defaultConfiguration)
{}

AnymalComKinoOperatingPoints::AnymalComKinoOperatingPoints(const AnymalComKinoOperatingPoints& rhs)
: Base(rhs)
{}

AnymalComKinoOperatingPoints* AnymalComKinoOperatingPoints::clone() const {

	return new AnymalComKinoOperatingPoints(*this);
}

} //end of namespace anymal


