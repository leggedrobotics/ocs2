/*
 * AnymalCost.cpp
 *
 *  Created on: Nov 23, 2017
 *      Author: farbod
 */

#include "ocs2_anymal_switched_model/cost/AnymalCost.h"

namespace anymal {

AnymalCost::AnymalCost(const state_matrix_t& Q,
		const input_matrix_t& R,
		const state_matrix_t& QFinal,
		const state_vector_t& xFinal,
		const scalar_t& copWeightMax /*=0.0*/,
		const state_matrix_t& QIntermediate /*=state_matrix_t::Zero()*/,
		const state_vector_t& xNominalIntermediate /*=state_vector_t::Zero()*/,
		const scalar_t& sigma /*=1*/,
		const scalar_t& tp /*=0*/)

	: Base(AnymalKinematics(), AnymalCom(),
		Q, R, QFinal, xFinal, copWeightMax, QIntermediate, xNominalIntermediate, sigma, tp)
{}


AnymalCost::AnymalCost(const AnymalCost& rhs)
	: Base(rhs)
{}

} //end of namespace anymal



