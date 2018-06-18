/*
 * AnymalCost.h
 *
 *  Created on: Nov 23, 2017
 *      Author: farbod
 */

#ifndef ANYMAL_ANYMALCOST_H_
#define ANYMAL_ANYMALCOST_H_

#include <c_switched_model_interface/cost/SwitchedModelCostBase.h>
#include <ocs2_anymal_switched_model/dynamics/AnymalCom.h>
#include <ocs2_anymal_switched_model/kinematics/AnymalKinematics.h>

namespace anymal {

class AnymalCost : public switched_model::SwitchedModelCostBase<12>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef SwitchedModelCostBase<12> Base;

	AnymalCost(const state_matrix_t& Q,
			const input_matrix_t& R,
			const state_matrix_t& QFinal,
			const state_vector_t& xFinal,
			const scalar_t& copWeightMax = 0.0,
			const state_matrix_t& QIntermediate = state_matrix_t::Zero(),
			const state_vector_t& xNominalIntermediate = state_vector_t::Zero(),
			const scalar_t& sigma = 1,
			const scalar_t& tp = 0);

	AnymalCost(const AnymalCost& rhs);

	~AnymalCost() {}

private:

};

} //end of namespace anymal


#endif /* ANYMAL_ANYMALCOST_H_ */
