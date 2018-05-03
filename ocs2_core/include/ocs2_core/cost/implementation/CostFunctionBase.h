/*
 * CostFunctionBase.h
 *
 *  Created on: May 3, 2018
 *      Author: farbod
 */

namespace ocs2{

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class logic_rules_template_t>
void CostFunctionBase<STATE_DIM, INPUT_DIM, logic_rules_template_t>::setCostNominalTrajectories(
		const scalar_array_t& timeTrajectory,
		const state_vector_array_t& stateTrajectory,
		const input_vector_array_t& inputTrajectory /*= input_vector_array_t()*/) {

	tNominalTrajectoryPtr_ = &timeTrajectory;

	if (stateTrajectory.empty()==false) {
		xNominalTrajectoryPtr_ = &stateTrajectory;
		xNominalFunc_.reset();
		xNominalFunc_.setTimeStamp(tNominalTrajectoryPtr_);
		xNominalFunc_.setData(xNominalTrajectoryPtr_);
	} else {
		xNominalTrajectoryPtr_ = nullptr;
		xNominalFunc_.setZero();
	}

	if (inputTrajectory.empty()==false) {
		uNominalTrajectoryPtr_ = &inputTrajectory;
		uNominalFunc_.reset();
		uNominalFunc_.setTimeStamp(tNominalTrajectoryPtr_);
		uNominalFunc_.setData(uNominalTrajectoryPtr_);
	} else {
		uNominalTrajectoryPtr_ = nullptr;
		uNominalFunc_.setZero();
	}
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class logic_rules_template_t>
void CostFunctionBase<STATE_DIM, INPUT_DIM, logic_rules_template_t>::getCostNominalTrajectories(
		scalar_array_t& timeTrajectory,
		state_vector_array_t& stateTrajectory,
		input_vector_array_t& inputTrajectory) const {

	getCostNominalState(timeTrajectory, stateTrajectory);

	if (uNominalTrajectoryPtr_) {
		inputTrajectory = *uNominalTrajectoryPtr_;
	} else {
		inputTrajectory.empty();
	}
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class logic_rules_template_t>
void CostFunctionBase<STATE_DIM, INPUT_DIM, logic_rules_template_t>::getCostNominalState(
		scalar_array_t& timeTrajectory,
		state_vector_array_t& stateTrajectory) const {

	if (xNominalTrajectoryPtr_) {
		timeTrajectory  = *tNominalTrajectoryPtr_;
		stateTrajectory = *xNominalTrajectoryPtr_;
	} else {
		timeTrajectory.empty();
		stateTrajectory.empty();
	}
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class logic_rules_template_t>
void CostFunctionBase<STATE_DIM, INPUT_DIM, logic_rules_template_t>::getCostNominalInput(
		scalar_array_t& timeTrajectory,
		input_vector_array_t& inputTrajectory) const {

	if (uNominalTrajectoryPtr_) {
		timeTrajectory  = *tNominalTrajectoryPtr_;
		inputTrajectory = *uNominalTrajectoryPtr_;
	} else {
		timeTrajectory.empty();
		inputTrajectory.empty();
	}
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class logic_rules_template_t>
void CostFunctionBase<STATE_DIM, INPUT_DIM, logic_rules_template_t>::setTimePeriod(
		const scalar_t& timeStart,
		const scalar_t& timeFinal) {

	timeStart_ = timeStart;
	timeFinal_ = timeFinal;

	timeSD_ = (timeFinal-timeStart) / 6.0;
	timeMean_ = (timeFinal+timeStart) / 2.0;
}


} // namespace ocs2
