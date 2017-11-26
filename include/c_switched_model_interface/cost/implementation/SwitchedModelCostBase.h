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
std::shared_ptr<typename SwitchedModelCostBase<JOINT_COORD_SIZE>::Base> SwitchedModelCostBase<JOINT_COORD_SIZE>::clone() const {
	return std::allocate_shared< SwitchedModelCostBase<JOINT_COORD_SIZE>, Eigen::aligned_allocator<SwitchedModelCostBase<JOINT_COORD_SIZE>> >(
			Eigen::aligned_allocator<SwitchedModelCostBase<JOINT_COORD_SIZE>>(), *this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void SwitchedModelCostBase<JOINT_COORD_SIZE>::setCurrentStateAndControl(const scalar_t& t, const state_vector_t& x, const control_vector_t& u) {

	Base::setCurrentStateAndControl(t, x, u);

	// TODO: fix me. make it consistant
	const double tSpan = Base::timeFinal_-Base::timeStart_;
	const double tElapsedRatio = (t - Base::timeStart_)/tSpan;
	QFinal_.setZero();
	Q_ = (1.0-tElapsedRatio)*Q_desired_ + tElapsedRatio*QFinal_desired_;

	state_vector_t xNominal;
	xNominalFunc_.interpolate(t, xNominal);
	xDeviation_ = x - xNominal;
	control_vector_t uNominal;
	uNominalFunc_.interpolate(t, uNominal);
	uDeviation_ = u - uNominal;

	xDeviationIntermediate_ = x - xNominalIntermediate_;

	dtSquared_ = (t-tp_) * (t-tp_);

	/* CoP constraint */
	if (copWeightMax_ > std::numeric_limits<double>::epsilon())  {

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
void SwitchedModelCostBase<JOINT_COORD_SIZE>::setCostNominalState(const scalar_array_t& timeTrajectory,
		const state_vector_array_t& stateTrajectory) {

	tNominalTrajectory_ = timeTrajectory;
	xNominalTrajectory_ = stateTrajectory;
	xNominalFunc_.setTimeStamp(&tNominalTrajectory_);
	xNominalFunc_.setData(&xNominalTrajectory_);
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
void SwitchedModelCostBase<JOINT_COORD_SIZE>::controlDerivative(control_vector_t& dLdu)  {
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
	Eigen::Matrix<double,2,1> copError;
	Eigen::Matrix<double,2,12> devJoints_copError;
	Eigen::Matrix<double,2,12> devLambda_copError;
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
double SwitchedModelCostBase<JOINT_COORD_SIZE>::GaussianFunc (const double& mu, const double& sigma, const double& x) {
	return exp( -0.5 * pow( (x-mu)/sigma ,2) );
}

} // namespace switched_model

