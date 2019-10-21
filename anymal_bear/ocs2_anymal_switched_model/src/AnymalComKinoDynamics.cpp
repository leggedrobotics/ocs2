/*
 * AnymalComKinoDynamics.cpp
 *
 *  Created on: Nov 23, 2017
 *      Author: farbod
 */

#include "ocs2_anymal_switched_model/dynamics/AnymalComKinoDynamics.h"

#include "ocs2_anymal_switched_model/dynamics/AnymalCom.h"
#include "ocs2_anymal_switched_model/kinematics/AnymalKinematics.h"

namespace anymal {

AnymalComKinoDynamics::AnymalComKinoDynamics(const switched_model::Model_Settings& options)
: Base(AnymalKinematics(), AnymalCom(), options)
{}

AnymalComKinoDynamics::AnymalComKinoDynamics(const AnymalComKinoDynamics& rhs)
: Base(rhs)
{}

AnymalComKinoDynamics* AnymalComKinoDynamics::clone() const {

	return new AnymalComKinoDynamics(*this);
}

} //end of namespace anymal
