/*
 * AnymalComKinoConstraint.cpp
 *
 *  Created on: Feb 9, 2018
 *      Author: farbod
 */

#include "ocs2_anymal_switched_model/constraint/AnymalComKinoConstraint.h"

namespace anymal {

AnymalComKinoConstraint::AnymalComKinoConstraint(
		const scalar_t& gravitationalAcceleration, const switched_model::Options& options)

: Base(new AnymalKinematics(), new AnymalCom(), gravitationalAcceleration, options)
{}

AnymalComKinoConstraint::AnymalComKinoConstraint(const AnymalComKinoConstraint& rhs)
: Base(rhs)
{}

AnymalComKinoConstraint* AnymalComKinoConstraint::clone() const {

	return new AnymalComKinoConstraint(*this);
}

} // end of namespace anymal


