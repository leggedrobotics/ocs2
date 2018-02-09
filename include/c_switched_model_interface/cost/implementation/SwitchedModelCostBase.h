/*
 * SwitchedModelCostBase.h
 *
 *  Created on: Nov 23, 2017
 *      Author: farbod
 */

namespace switched_model {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
SwitchedModelCostBase<JOINT_COORD_SIZE>* SwitchedModelCostBase<JOINT_COORD_SIZE>::clone() const {

	return new SwitchedModelCostBase<JOINT_COORD_SIZE>(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void SwitchedModelCostBase<JOINT_COORD_SIZE>::initializeModel(const logic_rules_machine_t& logicRulesMachine,
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
void SwitchedModelCostBase<JOINT_COORD_SIZE>::setCurrentStateAndControl(const scalar_t& t, const state_vector_t& x, const input_vector_t& u) {

	Base::setCurrentStateAndControl(t, x, u);

	size_t index = findActiveSubsystemFnc_(t);
	logicRulesPtr_->getContactFlags(index, stanceLegs_);

	// TODO: fix me. make it consistant
	const scalar_t tSpan = Base::timeFinal_-Base::timeStart_;
	const scalar_t tElapsedRatio = (t - Base::timeStart_)/tSpan;
	QFinal_.setZero();
	Q_ = (1.0-tElapsedRatio)*Q_desired_ + tElapsedRatio*QFinal_desired_;

	state_vector_t xNominal;
	xNominalFunc_.interpolate(t, xNominal);
	xDeviation_ = x - xNominal;
	input_vector_t uNominal;
	uNominalFunc_.interpolate(t, uNominal);
	uDeviation_ = u - uNominal;

	xDeviationIntermediate_ = x - xNominalIntermediate_;

	dtSquared_ = (t-tp_) * (t-tp_);

	/* CoP constraint */
	if (copWeightMax_ > std::numeric_limits<scalar_t>::epsilon())  {

		copWeight_ = copWeightMax_ * std::exp( -0.5 * std::pow((t-Base::timeMean_)/Base::timeSD_, 2) );

		copErrorCostFunc(x.template tail<12>(), u.template head<12>(),
				copCost_, devJoints_copCost_, devLambda_copCost_,
				hessJoints_copCost_, hessLambda_copCost_, devLambdaJoints_copCost_);

	} else {
		copCost_ = 0.0;
		devJoints_copCost_.setZero();
		devLambda_copCost_.setZero();
		hessJoints_copCost_.setZero();
		hessLambda_copCost_.setZero();
		devLambdaJoints_copCost_.setZero();
	}

}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void SwitchedModelCostBase<JOINT_COORD_SIZE>::setCostNominalTrajectories(
		const scalar_array_t& timeTrajectory,
		const state_vector_array_t& stateTrajectory,
		const input_vector_array_t& inputTrajectory /*= input_vector_array_t()*/) {

	tNominalTrajectory_ = timeTrajectory;
	xNominalTrajectory_ = stateTrajectory;
	uNominalTrajectory_ = inputTrajectory;

	if (xNominalTrajectory_.empty()==true) {
		xNominalFunc_.setZero();
	} else {
		xNominalFunc_.reset();
		xNominalFunc_.setTimeStamp(&tNominalTrajectory_);
		xNominalFunc_.setData(&xNominalTrajectory_);
	}

	if (uNominalTrajectory_.empty()) {
		uNominalFunc_.setZero();
	} else {
		uNominalFunc_.reset();
		uNominalFunc_.setTimeStamp(&tNominalTrajectory_);
		uNominalFunc_.setData(&uNominalTrajectory_);
	}
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void SwitchedModelCostBase<JOINT_COORD_SIZE>::getCostNominalTrajectories(
		scalar_array_t& timeTrajectory,
		state_vector_array_t& stateTrajectory,
		input_vector_array_t& inputTrajectory) const {

	timeTrajectory  = tNominalTrajectory_;
	stateTrajectory = xNominalTrajectory_;
	inputTrajectory = uNominalTrajectory_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void SwitchedModelCostBase<JOINT_COORD_SIZE>::getCostNominalState(scalar_array_t& timeTrajectory,
		state_vector_array_t& stateTrajectory) const {

	timeTrajectory  = tNominalTrajectory_;
	stateTrajectory = xNominalTrajectory_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void SwitchedModelCostBase<JOINT_COORD_SIZE>::getCostNominalInput(
		scalar_array_t& timeTrajectory,
		input_vector_array_t& inputTrajectory) const {

	timeTrajectory  = tNominalTrajectory_;
	inputTrajectory = uNominalTrajectory_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void SwitchedModelCostBase<JOINT_COORD_SIZE>::evaluate(scalar_t& L)  {

	scalar_t costQ = 0.5 * xDeviation_.transpose() * Q_ * xDeviation_;

	scalar_t costQintermediate = 0.5 * normalization_ * std::exp(-0.5 * dtSquared_ / sigmaSquared_) *
			xDeviationIntermediate_.transpose() * QIntermediate_ * xDeviationIntermediate_;
	scalar_t costR = 0.5 * uDeviation_.transpose() * R_ * uDeviation_;

	L = costQ + costQintermediate + costR + copWeight_*copCost_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void SwitchedModelCostBase<JOINT_COORD_SIZE>::stateDerivative(state_vector_t& dLdx)  {

	state_vector_t costQ = Q_ * xDeviation_;
	state_vector_t costQintermediate = QIntermediate_ * xDeviationIntermediate_ * normalization_ * std::exp(-0.5 * dtSquared_ / sigmaSquared_);
	dLdx = costQ + costQintermediate;

	dLdx.template tail<12>() += copWeight_*devJoints_copCost_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void SwitchedModelCostBase<JOINT_COORD_SIZE>::stateSecondDerivative(state_matrix_t& dLdxx)  {
	dLdxx = Q_ + QIntermediate_ * normalization_ * std::exp(-0.5 * dtSquared_ / sigmaSquared_);
	dLdxx.template bottomRightCorner<12,12>() += copWeight_*hessJoints_copCost_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void SwitchedModelCostBase<JOINT_COORD_SIZE>::controlDerivative(input_vector_t& dLdu)  {
	dLdu = R_ * uDeviation_;
	dLdu.template head<12>() += copWeight_*devLambda_copCost_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void SwitchedModelCostBase<JOINT_COORD_SIZE>::controlSecondDerivative(control_matrix_t& dLduu)  {
	dLduu = R_;
	dLduu.template topLeftCorner<12,12>() += copWeight_*hessLambda_copCost_;

}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void SwitchedModelCostBase<JOINT_COORD_SIZE>::stateControlDerivative(control_feedback_t& dLdxu)  {
	dLdxu.setZero();
	dLdxu.template topRightCorner<12,12>() += copWeight_*devLambdaJoints_copCost_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void SwitchedModelCostBase<JOINT_COORD_SIZE>::terminalCost(scalar_t& Phi) {

	state_vector_t x_deviation_final = Base::x_ - xFinal_;
	Phi = 0.5 * x_deviation_final.transpose() * QFinal_ * x_deviation_final;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void SwitchedModelCostBase<JOINT_COORD_SIZE>::terminalCostStateDerivative(state_vector_t& dPhidx)
{
	state_vector_t x_deviation_final = Base::x_ - xFinal_;
	dPhidx =  QFinal_ * x_deviation_final;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void SwitchedModelCostBase<JOINT_COORD_SIZE>::terminalCostStateSecondDerivative(state_matrix_t& dPhidxx)  {
	dPhidxx = QFinal_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void SwitchedModelCostBase<JOINT_COORD_SIZE>::copErrorCostFunc(
		const joint_coordinate_t& qJoints, const joint_coordinate_t& lambda,
		double& copCost,
		Eigen::Matrix<double,JOINT_COORD_SIZE,1>& devJoints_copCost,
		Eigen::Matrix<double,JOINT_COORD_SIZE,1>& devLambda_copCost,
		Eigen::Matrix<double,JOINT_COORD_SIZE,JOINT_COORD_SIZE>& hessJoints_copCost,
		Eigen::Matrix<double,JOINT_COORD_SIZE,JOINT_COORD_SIZE>& hessLambda_copCost,
		Eigen::Matrix<double,JOINT_COORD_SIZE,JOINT_COORD_SIZE>& devLambdaJoints_copCost) {


	// copError = Momentum_total - cop_des * lambda_total
	Eigen::Matrix<scalar_t,2,1> copError;
	Eigen::Matrix<scalar_t,2,12> devJoints_copError;
	Eigen::Matrix<scalar_t,2,12> devLambda_copError;
	copEstimatorPtr_->copErrorEstimator(stanceLegs_, qJoints, lambda, copError, devJoints_copError, devLambda_copError);

	copCost = 0.5*copError.squaredNorm();
	devJoints_copCost = devJoints_copError.transpose() * copError;
	devLambda_copCost = devLambda_copError.transpose() * copError;

	hessJoints_copCost = devJoints_copError.transpose() * devJoints_copError;
	hessLambda_copCost = devLambda_copError.transpose() * devLambda_copError;
	devLambdaJoints_copCost = devLambda_copError.transpose() * devJoints_copError;
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
double SwitchedModelCostBase<JOINT_COORD_SIZE>::GaussianFunc (const scalar_t& mu, const scalar_t& sigma, const scalar_t& x) {
	return exp( -0.5 * pow( (x-mu)/sigma ,2) );
}

} // namespace switched_model

