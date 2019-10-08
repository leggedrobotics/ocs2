/*
 * AnymalComKinoConstraint.cpp
 *
 *  Created on: Feb 9, 2018
 *      Author: farbod
 */

#include "ocs2_anymal_switched_model/constraint/AnymalComKinoConstraint.h"

#include "ocs2_anymal_switched_model/kinematics/AnymalKinematics.h"
#include "ocs2_anymal_switched_model/dynamics/AnymalCom.h"

namespace anymal {

AnymalComKinoConstraint::AnymalComKinoConstraint(std::shared_ptr<const logic_rules_t> logicRulesPtr, const switched_model::Model_Settings& options)

: Base(AnymalKinematics(), AnymalCom(),  std::move(logicRulesPtr), options)
{}

AnymalComKinoConstraint::AnymalComKinoConstraint(const AnymalComKinoConstraint& rhs)
: Base(rhs)
{}

AnymalComKinoConstraint* AnymalComKinoConstraint::clone() const {

	return new AnymalComKinoConstraint(*this);
}

} // end of namespace anymal


