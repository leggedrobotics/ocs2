/*
 * AnymalComKinoDynamicsDerivative.cpp
 *
 *  Created on: Nov 23, 2017
 *      Author: farbod
 */

#include "ocs2_anymal_switched_model/dynamics_derivative/AnymalComKinoDynamicsDerivative.h"

namespace anymal {

AnymalComKinoDynamicsDerivative::AnymalComKinoDynamicsDerivative(
		const scalar_t& gravitationalAcceleration, const switched_model::Options& options)

: Base(new AnymalKinematics(), new AnymalCom(), gravitationalAcceleration, options)
{}

AnymalComKinoDynamicsDerivative::AnymalComKinoDynamicsDerivative(const AnymalComKinoDynamicsDerivative& rhs)
: Base(rhs)
{}

AnymalComKinoDynamicsDerivative* AnymalComKinoDynamicsDerivative::clone() const {

	return new AnymalComKinoDynamicsDerivative(*this);
}

} //end of namespace anymal
