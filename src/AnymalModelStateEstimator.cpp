/*
 * AnymalModelStateEstimator.cpp
 *
 *  Created on: Nov 21, 2017
 *      Author: farbod
 */

#include "ocs2_anymal_switched_model/AnymalModelStateEstimator.h"

namespace anymal {


AnymalModelStateEstimator::AnymalModelStateEstimator()
	: AnymalModelStateEstimator::Base(
			std::allocate_shared< AnymalCom, Eigen::aligned_allocator<AnymalCom> > (Eigen::aligned_allocator<AnymalCom>() )
			)
{}


} //end of namespace anymal
