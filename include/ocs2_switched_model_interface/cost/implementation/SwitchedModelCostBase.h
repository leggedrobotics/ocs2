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
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
SwitchedModelCostBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::SwitchedModelCostBase(
		const kinematic_model_t& kinematicModel,
		const com_model_t& comModel,
		const state_matrix_t& Q,
		const input_matrix_t& R,
		const state_matrix_t& QFinal,
		const state_vector_t& xFinal,
		const scalar_t& copWeightMax /*= 0.0*/,
		const state_matrix_t& QIntermediate /*= state_matrix_t::Zero()*/,
		const state_vector_t& xNominalIntermediate /*= state_vector_t::Zero()*/,
		const scalar_t& sigma /*= 1.0*/,
		const scalar_t& tp /*= 0.0*/)

	: Base()
	, kinematicModelPtr_(kinematicModel.clone())
	, comModelPtr_(comModel.clone())
	, Q_desired_(Q)
	, R_desired_(R)
	, QFinal_desired_(QFinal)
	, xFinal_(xFinal)
	, copWeightMax_(copWeightMax)
	, QIntermediate_(QIntermediate)
	, xNominalIntermediate_(xNominalIntermediate)
	, sigma_(sigma)
	, sigmaSquared_(sigma*sigma)
	, normalization_(1.0 / (sigma_ * std::sqrt(2.0*M_PI)) )
	, tp_(tp)
	, dtSquared_(0.0)
	, copEstimatorPtr_(new cop_estimator_t(kinematicModel, comModel))
	, timeStart_(0.0)
	, timeFinal_(1.0)
	, timeSD_(0.17)
	, timeMean_(0.5)
{
	const size_t numMotionPhases = std::pow(2, (int)NUM_CONTACT_POINTS_);

	for (size_t i=0; i<numMotionPhases; i++) {

		contact_flag_t stanceLeg = modeNumber2StanceLeg(i);
		R_Bank_[stanceLeg] = correctedInputCost(stanceLeg, R_desired_);
	} // end of i loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
SwitchedModelCostBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::SwitchedModelCostBase(
		const SwitchedModelCostBase& rhs)

	: Base(rhs)
	, kinematicModelPtr_(rhs.kinematicModelPtr_->clone())
	, comModelPtr_(rhs.comModelPtr_->clone())
	, Q_desired_(rhs.Q_desired_)
	, R_desired_(rhs.R_desired_)
	, R_Bank_(rhs.R_Bank_)
	, QFinal_desired_(rhs.QFinal_desired_)
	, xFinal_(rhs.xFinal_)
	, copWeightMax_(rhs.copWeightMax_)
	, QIntermediate_(rhs.QIntermediate_)
	, xNominalIntermediate_(rhs.xNominalIntermediate_)
	, sigma_(rhs.sigma_)
	, sigmaSquared_(rhs.sigmaSquared_)
	, normalization_(rhs.normalization_)
	, tp_(rhs.tp_)
	, dtSquared_(rhs.dtSquared_)
	, copEstimatorPtr_(new cop_estimator_t(*rhs.kinematicModelPtr_, *rhs.comModelPtr_))
	, timeStart_(rhs.timeStart_)
	, timeFinal_(rhs.timeFinal_)
	, timeSD_(rhs.timeSD_)
	, timeMean_(rhs.timeMean_)
{}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
SwitchedModelCostBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>*
	SwitchedModelCostBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::clone() const {

	return new SwitchedModelCostBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SwitchedModelCostBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::initializeModel(
		logic_rules_machine_t& logicRulesMachine,
		const size_t& partitionIndex,
		const char* algorithmName/*=NULL*/) {

	Base::initializeModel(logicRulesMachine, partitionIndex, algorithmName);

	findActiveSubsystemFnc_ = std::move( logicRulesMachine.getHandleToFindActiveEventCounter(partitionIndex) );

	logicRulesPtr_ = logicRulesMachine.getLogicRulesPtr();

	// set the start and final time for cost function
	const scalar_array_t& partitioningTimes= logicRulesMachine.getPartitioningTimes();
	setTimePeriod(partitioningTimes[partitionIndex], partitioningTimes[partitionIndex+1]);

	if (algorithmName!=NULL)
		algorithmName_.assign(algorithmName);
	else
		algorithmName_.clear();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SwitchedModelCostBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::setTimePeriod(
		const scalar_t& timeStart,
		const scalar_t& timeFinal) {

	timeStart_ = timeStart;
	timeFinal_ = timeFinal;

	timeSD_ = (timeFinal-timeStart) / 6.0;
	timeMean_ = (timeFinal+timeStart) / 2.0;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SwitchedModelCostBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::setCurrentStateAndControl(
		const scalar_t& t,
		const state_vector_t& x,
		const input_vector_t& u) {

	Base::setCurrentStateAndControl(t, x, u);

	size_t index = findActiveSubsystemFnc_(t);
	logicRulesPtr_->getContactFlags(index, stanceLegs_);

	// TODO: fix me. make it consistent
	QFinal_.setZero();
//	const scalar_t tSpan = timeFinal_-timeStart_;
//	const scalar_t tElapsedRatio = (t - timeStart_)/tSpan;
//	Q_ = (1.0-tElapsedRatio)*Q_desired_ + tElapsedRatio*QFinal_desired_;
	Q_ = Q_desired_ + QFinal_desired_ / 1.0;

	// R matrix
	R_ = R_Bank_[stanceLegs_];

	dynamic_vector_t xNominal;
	Base::xNominalFunc_.interpolate(t, xNominal);

	xDeviation_ = x - xNominal;
	dynamic_vector_t uNominal;
	Base::uNominalFunc_.interpolate(t, uNominal);
	uDeviation_ = u - uNominal;

	xDeviationIntermediate_ = x - xNominalIntermediate_;

	dtSquared_ = (t-tp_) * (t-tp_);

	/* CoP constraint */
	if (copWeightMax_ > std::numeric_limits<scalar_t>::epsilon())  {

		copWeight_ = copWeightMax_ * std::exp( -0.5 * std::pow((t-timeMean_)/timeSD_, 2) );

		copErrorCostFunc(x.template segment<12>(12), u.template head<12>(),
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
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SwitchedModelCostBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getIntermediateCost(
		scalar_t& L)  {

	scalar_t costQ = 0.5 * xDeviation_.transpose() * Q_ * xDeviation_;

	scalar_t costQintermediate = 0.5 * normalization_ * std::exp(-0.5 * dtSquared_ / sigmaSquared_) *
			xDeviationIntermediate_.transpose() * QIntermediate_ * xDeviationIntermediate_;
	scalar_t costR = 0.5 * uDeviation_.transpose() * R_ * uDeviation_;

	L = costQ + costQintermediate + costR + copWeight_*copCost_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SwitchedModelCostBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getIntermediateCostDerivativeState(
		state_vector_t& dLdx)  {

	state_vector_t costQ = Q_ * xDeviation_;
	state_vector_t costQintermediate = QIntermediate_ * xDeviationIntermediate_ * normalization_ * std::exp(-0.5 * dtSquared_ / sigmaSquared_);
	dLdx = costQ + costQintermediate;

	dLdx.template segment<12>(12) += copWeight_*devJoints_copCost_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SwitchedModelCostBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getIntermediateCostSecondDerivativeState(
		state_matrix_t& dLdxx)  {

	dLdxx = Q_ + QIntermediate_ * normalization_ * std::exp(-0.5 * dtSquared_ / sigmaSquared_);
	dLdxx.template block<12,12>(12,12) += copWeight_*hessJoints_copCost_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SwitchedModelCostBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getIntermediateCostDerivativeInput(
		input_vector_t& dLdu)  {

	dLdu = R_ * uDeviation_;
	dLdu.template head<12>() += copWeight_*devLambda_copCost_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SwitchedModelCostBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getIntermediateCostSecondDerivativeInput(
		input_matrix_t& dLduu)  {

	dLduu = R_;
	dLduu.template topLeftCorner<12,12>() += copWeight_*hessLambda_copCost_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SwitchedModelCostBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getIntermediateCostDerivativeInputState(
		input_state_matrix_t& dLdux)  {

	dLdux.setZero();
	dLdux.template block<12,12>(0,12) += copWeight_*devLambdaJoints_copCost_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SwitchedModelCostBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getTerminalCost(scalar_t& Phi) {

	state_vector_t x_deviation_final = Base::x_ - xFinal_;
	Phi = 0.5 * x_deviation_final.transpose() * QFinal_ * x_deviation_final;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SwitchedModelCostBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getTerminalCostDerivativeState(
		state_vector_t& dPhidx) {

	state_vector_t x_deviation_final = Base::x_ - xFinal_;
	dPhidx =  QFinal_ * x_deviation_final;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SwitchedModelCostBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getTerminalCostSecondDerivativeState(
		state_matrix_t& dPhidxx)  {

	dPhidxx = QFinal_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
typename SwitchedModelCostBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::input_matrix_t
	SwitchedModelCostBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::correctedInputCost(
			const contact_flag_t& stanceLeg,
			const input_matrix_t& R) {

	input_matrix_t nondiagonalR = R;
	scalar_t meanRz = 0;
	for (size_t i=2; i<12; i+=3)
		meanRz += R(i,i) / static_cast<double>(NUM_CONTACT_POINTS_);
	for (size_t j=0; j<NUM_CONTACT_POINTS_; j++)
		for (size_t k=0; k<=j; k++)
			if (k==j) {
				nondiagonalR(3*j+0,3*k+0) = 1.05*R(3*j+0,3*k+0);
				nondiagonalR(3*j+1,3*k+1) = 1.05*R(3*j+1,3*k+1);
				nondiagonalR(3*j+2,3*k+2) = 1.05*meanRz;
			}
			else {
				if (stanceLeg[j] && stanceLeg[k])
					nondiagonalR(3*j+2,3*k+2) = 2*meanRz;
				else
					nondiagonalR(3*j+2,3*k+2) = 0.0;
			}
	nondiagonalR = 0.5*(nondiagonalR + nondiagonalR.transpose()).eval();

	return nondiagonalR;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SwitchedModelCostBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::copErrorCostFunc(
		const joint_coordinate_t& qJoints,
		const joint_coordinate_t& lambda,
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
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
double SwitchedModelCostBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::GaussianFunc (
		const scalar_t& mu,
		const scalar_t& sigma,
		const scalar_t& x) {

	return exp( -0.5 * pow( (x-mu)/sigma ,2) );
}

} // namespace switched_model

