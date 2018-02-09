/*
 * AnymalModelStateEstimator.cpp
 *
 *  Created on: Nov 21, 2017
 *      Author: farbod
 */

#include "ocs2_anymal_switched_model/core/AnymalModelStateEstimator.h"

namespace anymal {


AnymalModelStateEstimator::AnymalModelStateEstimator()
: Base(new AnymalCom())
{}

AnymalModelStateEstimator::AnymalModelStateEstimator(const AnymalModelStateEstimator& rhs)
: Base(rhs)
{}


} //end of namespace anymal
