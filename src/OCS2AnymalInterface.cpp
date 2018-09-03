/*
 * OCS2AnymalInterface.cpp
 *
 *  Created on: May 11, 2017
 *      Author: farbod
 */

#include "ocs2_anymal_interface/OCS2AnymalInterface.h"

namespace anymal {
/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
OCS2AnymalInterface::OCS2AnymalInterface(const std::string& pathToConfigFolder)

: BASE(AnymalKinematics(), AnymalCom(), pathToConfigFolder)
{
	// set up optimizers
	setupOptimizer(logicRulesPtr_, &modeSequenceTemplate_, slqPtr_, mpcPtr_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void OCS2AnymalInterface::designWeightCompensatingInput(
		const state_vector_t& switchedState,
		input_vector_t& uForWeightCompensation)  {

	static AnymalWeightCompensationForces::vector_3d_t gravity(0.0, 0.0, -modelSettings_.gravitationalAcceleration_);

	AnymalWeightCompensationForces::vector_3d_array_t weightCompensatingForces;
	weightCompensationForces_.computeCartesianForces(gravity, contact_flag_t{1,1,1,1}/*stanceLegSequene_*/, switchedState.tail<12>(),
			weightCompensatingForces);

	for (size_t j=0; j<4; j++)
		uForWeightCompensation.segment<3>(3*j) = weightCompensatingForces[j];
	uForWeightCompensation.tail<12>().setZero();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void OCS2AnymalInterface::setupOptimizer(
		const logic_rules_ptr_t& logicRulesPtr,
		const mode_sequence_template_t* modeSequenceTemplatePtr,
		slq_base_ptr_t& slqPtr,
		mpc_ptr_t& mpcPtr) {

	dynamicsPtr_            = std::unique_ptr<system_dynamics_t>( new system_dynamics_t(modelSettings_) );
	dynamicsDerivativesPtr_ = std::unique_ptr<system_dynamics_derivative_t>( new system_dynamics_derivative_t(modelSettings_) );
	constraintsPtr_  = std::unique_ptr<constraint_t>( new constraint_t(modelSettings_) );
	costFunctionPtr_ = std::unique_ptr<cost_funtion_t>( new cost_funtion_t(Q_, R_, QFinal_, xFinal_, modelSettings_.copWeight_) );

	generalized_coordinate_t defaultCoordinate = initRbdState_.template head<18>();
	operatingPointsPtr_ = std::unique_ptr<operating_point_t>( new operating_point_t(modelSettings_, defaultCoordinate) );

	// SLQ
	if (slqSettings_.useMultiThreading_==true) {
		slqPtr = slq_base_ptr_t(new slq_mp_t(dynamicsPtr_.get(), dynamicsDerivativesPtr_.get(), constraintsPtr_.get(),
				costFunctionPtr_.get(), operatingPointsPtr_.get(), slqSettings_, logicRulesPtr.get()) );
	} else {
		slqPtr = slq_base_ptr_t(new slq_t(dynamicsPtr_.get(), dynamicsDerivativesPtr_.get(), constraintsPtr_.get(),
				costFunctionPtr_.get(), operatingPointsPtr_.get(), slqSettings_, logicRulesPtr.get()) );
	}

	// MPC
	if (modelSettings_.gaitOptimization_ == false) {
		mpcPtr = mpc_ptr_t( new mpc_t(dynamicsPtr_.get(), dynamicsDerivativesPtr_.get(), constraintsPtr_.get(),
				costFunctionPtr_.get(), operatingPointsPtr_.get(),
				partitioningTimes_,
				slqSettings_, mpcSettings_, logicRulesPtr.get(), modeSequenceTemplatePtr));

	} else {
		typedef ocs2::MPC_OCS2<BASE::state_dim_, BASE::input_dim_, BASE::logic_rules_t>	mpc_ocs2_t;

		mpcPtr = mpc_ptr_t( new mpc_ocs2_t(dynamicsPtr_.get(), dynamicsDerivativesPtr_.get(), constraintsPtr_.get(),
				costFunctionPtr_.get(), operatingPointsPtr_.get(),
				partitioningTimes_,
				slqSettings_, mpcSettings_, logicRulesPtr.get(), modeSequenceTemplatePtr));
	}

}

} // end of namespace anymal
