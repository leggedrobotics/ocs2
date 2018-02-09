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
template <size_t JOINT_COORD_SIZE>
ComKinoDynamicsBase<JOINT_COORD_SIZE>* ComKinoDynamicsBase<JOINT_COORD_SIZE>::clone() const {

	return new ComKinoDynamicsBase<JOINT_COORD_SIZE>(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void ComKinoDynamicsBase<JOINT_COORD_SIZE>::initializeModel(const logic_rules_machine_t& logicRulesMachine,
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
void ComKinoDynamicsBase<JOINT_COORD_SIZE>::computeDerivative(const scalar_t& t,
		const state_vector_t& x,
		const input_vector_t& u,
		state_vector_t& dxdt)   {

	size_t index = findActiveSubsystemFnc_(t);
	logicRulesPtr_->getContactFlags(index, stanceLegs_);

	// set data for CoM class
	comDynamics_.setData(stanceLegs_, x.template tail<12>(), u.template tail<12>());

	// CoM state time derivatives
	typename ComDynamicsBase<JOINT_COORD_SIZE>::state_vector_t stateDerivativeCoM;
	comDynamics_.computeDerivative(t, x.template head<12>(), u.template head<12>(), stateDerivativeCoM);

	// extended state time derivatives
	dxdt << stateDerivativeCoM, u.template tail<12>();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void ComKinoDynamicsBase<JOINT_COORD_SIZE>::setStanceLegs (const std::array<bool,4>& stanceLegs)  {
	stanceLegs_ = stanceLegs;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void ComKinoDynamicsBase<JOINT_COORD_SIZE>::getStanceLegs (std::array<bool,4>& stanceLegs)  const {
	stanceLegs = stanceLegs_;
}


} // end of namespace switched_model


