/*
 * AnymalWeightCompensationForces.h
 *
 *  Created on: Nov 24, 2017
 *      Author: farbod
 */

#ifndef ANYMAL_ANYMALWEIGHTCOMPENSATIONFORCES_H_
#define ANYMAL_ANYMALWEIGHTCOMPENSATIONFORCES_H_

#include <memory>
#include <Eigen/Dense>

#include <ocs2_anymal_switched_model/dynamics/AnymalCom.h>
#include <ocs2_anymal_switched_model/kinematics/AnymalKinematics.h>
#include <ocs2_switched_model_interface/misc/WeightCompensationForces.h>

namespace anymal {

class AnymalWeightCompensationForces : public switched_model::WeightCompensationForces<12>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef switched_model::WeightCompensationForces<12> Base;

	AnymalWeightCompensationForces();

	AnymalWeightCompensationForces(const AnymalWeightCompensationForces& rhs);

	~AnymalWeightCompensationForces() {}

private:

};

}  // end of anymal namespace

#endif /* ANYMAL_ANYMALWEIGHTCOMPENSATIONFORCES_H_ */
