/*
 * AnymalComKinoOperatingPoints.cpp
 *
 *  Created on: Feb 9, 2018
 *      Author: farbod
 */

#include "ocs2_anymal_switched_model/initialization/AnymalComKinoOperatingPoints.h"

namespace anymal {

AnymalComKinoOperatingPoints::AnymalComKinoOperatingPoints(
		const switched_model::Model_Settings& options,
		const generalized_coordinate_t& defaultConfiguration)
: Base(AnymalKinematics(), AnymalCom(), options, defaultConfiguration)
{}

AnymalComKinoOperatingPoints::AnymalComKinoOperatingPoints(const AnymalComKinoOperatingPoints& rhs)
: Base(rhs)
{}

AnymalComKinoOperatingPoints* AnymalComKinoOperatingPoints::clone() const {

	return new AnymalComKinoOperatingPoints(*this);
}

} //end of namespace anymal


