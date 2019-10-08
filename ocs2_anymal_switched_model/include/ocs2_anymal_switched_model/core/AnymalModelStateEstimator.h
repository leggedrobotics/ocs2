/*
 * AnymalModelStateEstimator.h
 *
 *  Created on: Nov 20, 2017
 *      Author: farbod
 */

#pragma once

#include <ocs2_switched_model_interface/core/SwitchedModelStateEstimator.h>

namespace anymal {

class AnymalModelStateEstimator final : public switched_model::SwitchedModelStateEstimator<12>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef switched_model::SwitchedModelStateEstimator<12> Base;

	AnymalModelStateEstimator();

	AnymalModelStateEstimator(const AnymalModelStateEstimator& rhs);

	~AnymalModelStateEstimator() {}
};

} //end of namespace anymal

