/*
 * AnymalComKinoDynamicsDerivative.cpp
 *
 *  Created on: Nov 23, 2017
 *      Author: farbod
 */

#include "ocs2_anymal_switched_model/dynamics_derivative/AnymalComKinoDynamicsDerivative.h"

namespace anymal {

AnymalComKinoDynamicsDerivative::AnymalComKinoDynamicsDerivative(const switched_model::Model_Settings& options)
: Base(AnymalKinematics(), AnymalCom(), options)
{}

AnymalComKinoDynamicsDerivative::AnymalComKinoDynamicsDerivative(const AnymalComKinoDynamicsDerivative& rhs)
: Base(rhs)
{}

AnymalComKinoDynamicsDerivative* AnymalComKinoDynamicsDerivative::clone() const {

	return new AnymalComKinoDynamicsDerivative(*this);
}

} //end of namespace anymal
