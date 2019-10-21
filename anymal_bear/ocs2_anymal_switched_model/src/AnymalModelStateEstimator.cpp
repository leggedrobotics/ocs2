/*
 * AnymalModelStateEstimator.cpp
 *
 *  Created on: Nov 21, 2017
 *      Author: farbod
 */

#include "ocs2_anymal_switched_model/core/AnymalModelStateEstimator.h"

#include "ocs2_anymal_switched_model/core/AnymalCom.h"

namespace anymal {


AnymalModelStateEstimator::AnymalModelStateEstimator()
: Base(AnymalCom())
{}

AnymalModelStateEstimator::AnymalModelStateEstimator(const AnymalModelStateEstimator& rhs)
: Base(rhs)
{}


} //end of namespace anymal
