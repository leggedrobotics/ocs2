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
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
ComKinoDynamicsDerivativeBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>*
	ComKinoDynamicsDerivativeBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::clone() const {

	return new ComKinoDynamicsDerivativeBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ComKinoDynamicsDerivativeBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::initializeModel(
		logic_rules_machine_t& logicRulesMachine,
		const size_t& partitionIndex,
		const char* algorithmName/*=NULL*/) {

	Base::initializeModel(logicRulesMachine, partitionIndex, algorithmName);

	findActiveSubsystemFnc_ = std::move( logicRulesMachine.getHandleToFindActiveEventCounter(partitionIndex) );

	logicRulesPtr_ = logicRulesMachine.getLogicRulesPtr();

	if (algorithmName!=NULL)
		algorithmName_.assign(algorithmName);
	else
		algorithmName_.clear();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ComKinoDynamicsDerivativeBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::setCurrentStateAndControl(
		const scalar_t& t,
		const state_vector_t& x,
		const input_vector_t& u)  {

	Base::setCurrentStateAndControl(t, x, u);

//	size_t index = findActiveSubsystemFnc_(t);
//	logicRulesPtr_->getContactFlags(index, stanceLegs_);

	// HyQ's joints
	qJoints_  = x.template tail<12>();
	// HyQ's joints' velocities
	dqJoints_ = u.template tail<12>();

	// set CoM data
	comDynamicsDerivative_.setData(stanceLegs_, qJoints_, dqJoints_);
	comDynamicsDerivative_.setCurrentStateAndControl(t, x.template head<12>(), u.template head<12>());
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ComKinoDynamicsDerivativeBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getFlowMapDerivativeState(
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
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ComKinoDynamicsDerivativeBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getFlowMapDerivativeInput(
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

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ComKinoDynamicsDerivativeBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::setStanceLegs(const contact_flag_t& stanceLegs)  {

	stanceLegs_ = stanceLegs;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ComKinoDynamicsDerivativeBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getStanceLegs(contact_flag_t& stanceLegs) const {

	stanceLegs = stanceLegs_;
}

} //end of namespace switched_model
