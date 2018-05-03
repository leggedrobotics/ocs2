/*
 * AnymalComKinoConstraint.cpp
 *
 *  Created on: Feb 9, 2018
 *      Author: farbod
 */

#include "ocs2_anymal_switched_model/constraint/AnymalComKinoConstraint.h"

namespace anymal {

AnymalComKinoConstraint::AnymalComKinoConstraint(const switched_model::Model_Settings& options)

: Base(AnymalKinematics(), AnymalCom(), options)
{}

AnymalComKinoConstraint::AnymalComKinoConstraint(const AnymalComKinoConstraint& rhs)
: Base(rhs)
{}

AnymalComKinoConstraint* AnymalComKinoConstraint::clone() const {

	return new AnymalComKinoConstraint(*this);
}

} // end of namespace anymal


