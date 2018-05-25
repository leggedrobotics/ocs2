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
void CostFunctionBase<STATE_DIM, INPUT_DIM, logic_rules_template_t>::setCostDesiredTrajectories(
		const cost_desired_trajectories_t& costDesiredTrajectories) {

	costDesiredTrajectoriesPtr_ = &costDesiredTrajectories;

	costDesiredTrajectories.getDesiredStateFunc(xNominalFunc_);
	costDesiredTrajectories.getDesiredInputFunc(uNominalFunc_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class logic_rules_template_t>
void CostFunctionBase<STATE_DIM, INPUT_DIM, logic_rules_template_t>::getCostDesiredTrajectories(
		cost_desired_trajectories_t& costDesiredTrajectories) const {

	costDesiredTrajectories = *costDesiredTrajectoriesPtr_;
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
