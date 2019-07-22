/*
 * ComKinoDynamicsDerivativeBase.h
 *
 *  Created on: Nov 12, 2017
 *      Author: farbod
 */


namespace switched_model {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
ComKinoDynamicsDerivativeBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>*
	ComKinoDynamicsDerivativeBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::clone() const {

	return new ComKinoDynamicsDerivativeBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
void ComKinoDynamicsDerivativeBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::setCurrentStateAndControl(
		const scalar_t& t,
		const state_vector_t& x,
		const input_vector_t& u)  {

	Base::setCurrentStateAndControl(t, x, u);

	// set CoM data
	comDynamicsDerivative_.setData(x.template tail<12>(), u.template tail<12>());
	comDynamicsDerivative_.setCurrentStateAndControl(t, x.template head<12>(), u.template head<12>());
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
void ComKinoDynamicsDerivativeBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::getFlowMapDerivativeState(
		state_matrix_t& A)  {

	// A matrix
	/*			com		qJoints
	 * A = 	|	A00			A01	|
	 *		|	0			0	|
	 */

	// CoM derivative with respect to CoM
	Eigen::Matrix<double,12,12> comAcom;
	comDynamicsDerivative_.getFlowMapDerivativeState(comAcom);

	// CoM derivative with respect to qJoints
	Eigen::Matrix<double,12,JOINT_COORD_SIZE> comAjoints;
	comDynamicsDerivative_.getApproximateDerivativesJoint(comAjoints);

	A << comAcom,    							comAjoints,
		 Eigen::Matrix<double,12,12>::Zero(), 	Eigen::Matrix<double,12,12>::Zero();
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
void ComKinoDynamicsDerivativeBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::getFlowMapDerivativeInput(
		state_input_matrix_t& B)  {

	// B matrix
	/*			lambda		jointsVel=omega
	 * B = 	|	B00			B01	|
	 *		|	0			I	|
	 */
	// CoM derivative with respect to lambdas
	Eigen::Matrix<double,12,12> comBlambda;
	comDynamicsDerivative_.getFlowMapDerivativeInput(comBlambda);

	// CoM derivative with respect to joints' velocities
	Eigen::Matrix<double,12,JOINT_COORD_SIZE> comBomega;
	comDynamicsDerivative_.getApproximateDerivativesJointVelocity(comBomega);

	B << comBlambda,    						comBomega,
		 Eigen::Matrix<double,12,12>::Zero(), 	Eigen::Matrix<double,12,12>::Identity();

}

} //end of namespace switched_model
