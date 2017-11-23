/*
 * AnymalModelStateEstimator.h
 *
 *  Created on: Nov 20, 2017
 *      Author: farbod
 */

#ifndef ANYMALMODELSTATEESTIMATOR_H_
#define ANYMALMODELSTATEESTIMATOR_H_

#include <c_switched_model_interface/core/SwitchedModelStateEstimator.h>
#include <ocs2_anymal_switched_model/dynamics/AnymalCom.h>

namespace anymal {

class AnymalModelStateEstimator : public switched_model::SwitchedModelStateEstimator<12>
{
public:
	typedef switched_model::SwitchedModelStateEstimator<12> Base;

	AnymalModelStateEstimator();

	~AnymalModelStateEstimator() {}

private:

};

} //end of namespace anymal

#endif /* ANYMALMODELSTATEESTIMATOR_H_ */
