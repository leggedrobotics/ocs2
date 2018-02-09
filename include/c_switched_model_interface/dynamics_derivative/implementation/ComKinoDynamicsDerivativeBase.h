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
template <size_t JOINT_COORD_SIZE>
ComKinoDynamicsDerivativeBase<JOINT_COORD_SIZE>* ComKinoDynamicsDerivativeBase<JOINT_COORD_SIZE>::clone() const {

	return new ComKinoDynamicsDerivativeBase<JOINT_COORD_SIZE>(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void ComKinoDynamicsDerivativeBase<JOINT_COORD_SIZE>::initializeModel(const logic_rules_machine_t& logicRulesMachine,
		const size_t& partitionIndex, const char* algorithmName/*=NULL*/) {

	Base::initializeModel(logicRulesMachine, partitionIndex, algorithmName);

	findActiveSubsystemFnc_ = std::move( logicRulesMachine.getHandleToFindActiveSubsystemID(partitionIndex) );

	logicRulesPtr_ = logicRulesMachine.getLogicRulesPtr();

	if (algorithmName!=NULL)
		algorithmName_.assign(algorithmName);
	else
		algorithmName_.clear();
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void ComKinoDynamicsDerivativeBase<JOINT_COORD_SIZE>::setCurrentStateAndControl(const scalar_t& t, const state_vector_t& x,
		const input_vector_t& u)  {

	Base::setCurrentStateAndControl(t, x, u);

	size_t index = findActiveSubsystemFnc_(t);
	logicRulesPtr_->getContactFlags(index, stanceLegs_);

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
template <size_t JOINT_COORD_SIZE>
void ComKinoDynamicsDerivativeBase<JOINT_COORD_SIZE>::getDerivativeState(state_matrix_t& A)  {

	// A matrix
	/*			com		qJoints
	 * A = 	|	A00			A01	|
	 *		|	0			0	|
	 */

	// CoM derivative with respect to CoM
	Eigen::Matrix<double,12,12> comAcom;
	comDynamicsDerivative_.getDerivativeState(comAcom);

	// CoM derivative with respect to qJoints
	Eigen::Matrix<double,12,JOINT_COORD_SIZE> comAjoints;
	comDynamicsDerivative_.getApproximateDerivativesJoint(comAjoints);

	A << comAcom,    							comAjoints,
		 Eigen::Matrix<double,12,12>::Zero(), 	Eigen::Matrix<double,12,12>::Zero();
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void ComKinoDynamicsDerivativeBase<JOINT_COORD_SIZE>::getDerivativesControl(control_gain_matrix_t& B)  {

	// B matrix
	/*			lambda		jointsVel=omega
	 * B = 	|	B00			B01	|
	 *		|	0			I	|
	 */
	// CoM derivative with respect to lambdas
	Eigen::Matrix<double,12,12> comBlambda;
	comDynamicsDerivative_.getDerivativesControl(comBlambda);

	// CoM derivative with respect to joints' velocities
	Eigen::Matrix<double,12,JOINT_COORD_SIZE> comBomega;
	comDynamicsDerivative_.getApproximateDerivativesJointVelocity(comBomega);

	B << comBlambda,    						comBomega,
		 Eigen::Matrix<double,12,12>::Zero(), 	Eigen::Matrix<double,12,12>::Identity();

}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void ComKinoDynamicsDerivativeBase<JOINT_COORD_SIZE>::setStanceLegs (const std::array<bool,4>& stanceLegs)  {
	stanceLegs_ = stanceLegs;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void ComKinoDynamicsDerivativeBase<JOINT_COORD_SIZE>::getStanceLegs (std::array<bool,4>& stanceLegs) const {
	stanceLegs = stanceLegs_;
}

} //end of namespace switched_model
