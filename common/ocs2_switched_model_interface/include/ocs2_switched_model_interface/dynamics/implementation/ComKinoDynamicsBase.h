/*
 * ComKinoDynamicsBase.h
 *
 *  Created on: Nov 12, 2017
 *      Author: farbod
 */
namespace switched_model {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
ComKinoDynamicsBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>*
	ComKinoDynamicsBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::clone() const {

	return new ComKinoDynamicsBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
void ComKinoDynamicsBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::computeFlowMap(
		const scalar_t& t,
		const state_vector_t& x,
		const input_vector_t& u,
		state_vector_t& dxdt)   {

	// set data for CoM class
	comDynamics_.setData(x.template tail<12>(), u.template tail<12>());

	// CoM state time derivatives
	typename ComDynamicsBase<JOINT_COORD_SIZE>::state_vector_t stateDerivativeCoM;
	comDynamics_.computeFlowMap(t, x.template head<12>(), u.template head<12>(), stateDerivativeCoM);

	// extended state time derivatives
	dxdt << stateDerivativeCoM, u.template tail<12>();
}

} // end of namespace switched_model


