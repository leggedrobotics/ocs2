/*
 * ComKinoOperatingPointsBase.h
 *
 *  Created on: Feb 9, 2018
 *      Author: farbod
 */

namespace switched_model {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
ComKinoOperatingPointsBase<JOINT_COORD_SIZE>* ComKinoOperatingPointsBase<JOINT_COORD_SIZE>::clone() const {

	return new ComKinoOperatingPointsBase<JOINT_COORD_SIZE>(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void ComKinoOperatingPointsBase<JOINT_COORD_SIZE>::computeOperatingPoints(
		const generalized_coordinate_t& defaultConfiguration,
		std::vector<contact_flag_t>& possibleStanceLegs,
		state_vector_array_t& stateOperatingPoints,
		input_vector_array_t& inputOperatingPoints) {

	const size_t NUM_CONTACT_POINTS = SwitchedModel<JOINT_COORD_SIZE>::NUM_CONTACT_POINTS;
	const size_t NUM_PHASES = std::pow(2, NUM_CONTACT_POINTS);

	possibleStanceLegs.resize(NUM_PHASES);
	for (size_t i=0; i<NUM_PHASES; i++) {

		if (options_.constrainedIntegration_==true)  {
			possibleStanceLegs[i] = modeNumber2StanceLeg(i);
//			possibleStanceLegs[i] = contact_flag_t{1,1,1,1};

		} else {
			possibleStanceLegs[i] = contact_flag_t{1,1,1,1};
		}
	}  // end of i loop

	stateOperatingPoints.resize(NUM_PHASES);
	inputOperatingPoints.resize(NUM_PHASES);
	for (size_t i=0; i<NUM_PHASES; i++) {

		// state operating point
		stateOperatingPoints[i].template head<6>() = defaultConfiguration.template head<6>();
		stateOperatingPoints[i].template segment<6>(6).setZero();
		stateOperatingPoints[i].template tail<JOINT_COORD_SIZE>() = defaultConfiguration.template tail<JOINT_COORD_SIZE>();

		// Input operating point
		computeInputOperatingPoints(stateOperatingPoints[i], possibleStanceLegs[i], inputOperatingPoints[i]);

	}  // end of i loop

}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void ComKinoOperatingPointsBase<JOINT_COORD_SIZE>::computeInputOperatingPoints(
		const state_vector_t& switchedState,
		const contact_flag_t& stanceLegs,
		input_vector_t& inputOperatingPoint) {

	const size_t NUM_CONTACT_POINTS = SwitchedModel<JOINT_COORD_SIZE>::NUM_CONTACT_POINTS;

	matrix3d_t b_R_o = RotationMatrixOrigintoBase(switchedState.template head<3>());

	// Input operating point
	std::array<vector3d_t,NUM_CONTACT_POINTS> sphericalWeightCompensationForces;
	weightCompensationForces_.computeCartesianForces(b_R_o*o_gravityVector_, stanceLegs,
			switchedState.template tail<JOINT_COORD_SIZE>(), sphericalWeightCompensationForces);

	inputOperatingPoint.template tail<JOINT_COORD_SIZE>().setZero();
	for (size_t j=0; j<NUM_CONTACT_POINTS; j++)
		inputOperatingPoint.template segment<3>(3*j) = sphericalWeightCompensationForces[j];

}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void ComKinoOperatingPointsBase<JOINT_COORD_SIZE>::initializeModel(logic_rules_machine_t& logicRulesMachine,
		const size_t& partitionIndex, const char* algorithmName /*=NULL*/) {

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
template <size_t JOINT_COORD_SIZE>
void ComKinoOperatingPointsBase<JOINT_COORD_SIZE>::getSystemOperatingTrajectories(
		const state_vector_t& initialState,
		const scalar_t& startTime,
		const scalar_t& finalTime,
		scalar_array_t& timeTrajectory,
		state_vector_array_t& stateTrajectory,
		input_vector_array_t& inputTrajectory,
		bool concatOutput /*= false*/) {

	size_t index = findActiveSubsystemFnc_(0.5*(startTime+finalTime));
	logicRulesPtr_->getContactFlags(index, stanceLegs_);

	// approach 1: using the default values
	size_t mode = stanceLeg2ModeNumber(stanceLegs_);
	Base::stateOperatingPoint_ = allStateOperatingPoints_[mode];
	Base::stateOperatingPoint_.template head<6>() = initialState.template head<6>();
	Base::inputOperatingPoint_ = allInputOperatingPoints_[mode];

	// approach 2: recomputing the inpu
//	Base::stateOperatingPoint_ = initialState;
//	Base::stateOperatingPoint_.template segment<6>(6).setZero();
//	computeInputOperatingPoints(Base::stateOperatingPoint_, stanceLegs_, Base::inputOperatingPoint_);


	if (concatOutput==false) {
		timeTrajectory.clear();
		stateTrajectory.clear();
		inputTrajectory.clear();
	}

	timeTrajectory.push_back(startTime);
	timeTrajectory.push_back(finalTime);

	stateTrajectory.push_back(Base::stateOperatingPoint_);
	stateTrajectory.push_back(Base::stateOperatingPoint_);

	inputTrajectory.push_back(Base::inputOperatingPoint_);
	inputTrajectory.push_back(Base::inputOperatingPoint_);
}

} // end of namespace switched_model


