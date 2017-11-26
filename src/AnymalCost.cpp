/*
 * AnymalCost.cpp
 *
 *  Created on: Nov 23, 2017
 *      Author: farbod
 */

#include "ocs2_anymal_switched_model/cost/AnymalCost.h"

namespace anymal {

AnymalCost::AnymalCost(const std::array<bool,4>& stanceLegs,
		const state_matrix_t& Q,
		const control_matrix_t& R,
		const scalar_array_t&         tNominalTrajectory,
		const state_vector_array_t&   xNominalTrajectory,
		const control_vector_array_t& uNominalTrajectory,
		const state_matrix_t& QFinal,
		const state_vector_t& xFinal,
		const double& copWeightMax /*=0.0*/,
		const state_matrix_t& QIntermediate /*=state_matrix_t::Zero()*/,
		const state_vector_t& xNominalIntermediate /*=state_vector_t::Zero()*/,
		const double& sigma /*=1*/,
		const double& tp /*=0*/)

	: Base(new AnymalKinematics, new AnymalCom,
		stanceLegs, Q, R, tNominalTrajectory, xNominalTrajectory, uNominalTrajectory,
		QFinal, xFinal, copWeightMax, QIntermediate, xNominalIntermediate, sigma, tp)
{}


AnymalCost::AnymalCost(const AnymalCost& rhs)
	: Base(rhs)
{}

} //end of namespace anymal



