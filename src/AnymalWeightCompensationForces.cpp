/*
 * AnymalWeightCompensationForces.cpp
 *
 *  Created on: Nov 24, 2017
 *      Author: farbod
 */

#include "ocs2_anymal_switched_model/misc/AnymalWeightCompensationForces.h"


namespace anymal {


AnymalWeightCompensationForces::AnymalWeightCompensationForces()
	: AnymalWeightCompensationForces::Base(new AnymalKinematics, new AnymalCom)
{}

AnymalWeightCompensationForces::AnymalWeightCompensationForces(const AnymalWeightCompensationForces& rhs)
: Base(rhs)
{}

} //end of namespace anymal
