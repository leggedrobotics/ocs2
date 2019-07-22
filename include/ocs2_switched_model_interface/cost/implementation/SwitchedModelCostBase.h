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
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
SwitchedModelCostBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::SwitchedModelCostBase(
		const kinematic_model_t& kinematicModel,
		const com_model_t& comModel,
		std::shared_ptr<const logic_rules_t> logicRulesPtr,
		const state_matrix_t& Q,
		const input_matrix_t& R,
		const state_matrix_t& QFinal,
		const state_vector_t& xNominalFinal,
		const scalar_t& copWeightMax /*= 0.0*/,
		const state_matrix_t& QIntermediateGoal /*= state_matrix_t::Zero()*/,
		const state_vector_t& xIntermediateGoal /*= state_vector_t::Zero()*/,
		const scalar_t& sigma /*= 1.0*/,
		const scalar_t& tp /*= 0.0*/)

	: BASE(Q, R, state_vector_t::Zero(), input_vector_t::Zero(), QFinal, xNominalFinal)
	, kinematicModelPtr_(kinematicModel.clone())
	, comModelPtr_(comModel.clone())
	, logicRulesPtr_(std::move(logicRulesPtr))
	, copWeightMax_(copWeightMax)
	, QIntermediateGoal_(QIntermediateGoal)
	, xIntermediateGoal_(xIntermediateGoal)
	, sigma_(sigma)
	, sigmaSquared_(sigma*sigma)
	, normalization_(1.0 / (sigma_ * std::sqrt(2.0*M_PI)) )
	, tp_(tp)
	, dtSquared_(0.0)
	, copEstimatorPtr_(new cop_estimator_t(kinematicModel, comModel))
{
	if (!logicRulesPtr_) {
		throw std::runtime_error("[SwitchedModelCostBase] logicRules cannot be a nullptr");
	}

	const size_t numMotionPhases = std::pow(2, (int)NUM_CONTACT_POINTS_);

	for (size_t i=0; i<numMotionPhases; i++) {

		contact_flag_t stanceLeg = modeNumber2StanceLeg(i);
		R_Bank_[stanceLeg] = correctedInputCost(stanceLeg, R);
	} // end of i loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
SwitchedModelCostBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::SwitchedModelCostBase(
		const SwitchedModelCostBase& rhs)

	: BASE(rhs)
	, kinematicModelPtr_(rhs.kinematicModelPtr_->clone())
	, comModelPtr_(rhs.comModelPtr_->clone())
	, logicRulesPtr_(rhs.logicRulesPtr_)
	, R_Bank_(rhs.R_Bank_)
	, copWeightMax_(rhs.copWeightMax_)
	, QIntermediateGoal_(rhs.QIntermediateGoal_)
	, xIntermediateGoal_(rhs.xIntermediateGoal_)
	, sigma_(rhs.sigma_)
	, sigmaSquared_(rhs.sigmaSquared_)
	, normalization_(rhs.normalization_)
	, tp_(rhs.tp_)
	, dtSquared_(rhs.dtSquared_)
	, copEstimatorPtr_(new cop_estimator_t(*rhs.kinematicModelPtr_, *rhs.comModelPtr_))
{}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
SwitchedModelCostBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>*
	SwitchedModelCostBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::clone() const {

	return new SwitchedModelCostBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
void SwitchedModelCostBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::setCurrentStateAndControl(
		const scalar_t& t,
		const state_vector_t& x,
		const input_vector_t& u) {

	// active subsystem
	size_t index = logicRulesPtr_->getEventTimeCount(t);
	logicRulesPtr_->getContactFlags(index, stanceLegs_);

	// R matrix
	BASE::R_ = R_Bank_[stanceLegs_];

	dynamic_vector_t xNominal;
	BASE::xNominalFunc_.interpolate(t, xNominal);
	dynamic_vector_t uNominal;
	BASE::uNominalFunc_.interpolate(t, uNominal);

	// set base class
	BASE::setCurrentStateAndControl(t, x, u, xNominal, uNominal, xNominal);

	// intermediate goal
	xIntermediateDeviationGoal_ = x - xIntermediateGoal_;
	dtSquared_ = (t-tp_) * (t-tp_);

	/* CoP constraint */
	if (copWeightMax_ > std::numeric_limits<scalar_t>::epsilon())  {

		copWeight_ = copWeightMax_;

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
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
void SwitchedModelCostBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::getIntermediateCost(
		scalar_t& L)  {

	BASE::getIntermediateCost(L);

	scalar_t costQintermediate = 0.5 * normalization_ * std::exp(-0.5 * dtSquared_ / sigmaSquared_) *
			xIntermediateDeviationGoal_.dot(QIntermediateGoal_*xIntermediateDeviationGoal_);

	L += costQintermediate+ copWeight_*copCost_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
void SwitchedModelCostBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::getIntermediateCostDerivativeState(
		state_vector_t& dLdx)  {

	BASE::getIntermediateCostDerivativeState(dLdx);

	state_vector_t costQintermediate = normalization_ * std::exp(-0.5 * dtSquared_ / sigmaSquared_) *
			QIntermediateGoal_ * xIntermediateDeviationGoal_;
	dLdx += costQintermediate;

	dLdx.template segment<12>(12) += copWeight_*devJoints_copCost_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
void SwitchedModelCostBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::getIntermediateCostSecondDerivativeState(
		state_matrix_t& dLdxx)  {

	BASE::getIntermediateCostSecondDerivativeState(dLdxx);

	dLdxx += normalization_ * std::exp(-0.5 * dtSquared_ / sigmaSquared_) * QIntermediateGoal_;

	dLdxx.template block<12,12>(12,12) += copWeight_*hessJoints_copCost_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
void SwitchedModelCostBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::getIntermediateCostDerivativeInput(
		input_vector_t& dLdu)  {

	BASE::getIntermediateCostDerivativeInput(dLdu);

	dLdu.template head<12>() += copWeight_*devLambda_copCost_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
void SwitchedModelCostBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::getIntermediateCostSecondDerivativeInput(
		input_matrix_t& dLduu)  {

	BASE::getIntermediateCostSecondDerivativeInput(dLduu);

	dLduu.template topLeftCorner<12,12>() += copWeight_*hessLambda_copCost_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
void SwitchedModelCostBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::getIntermediateCostDerivativeInputState(
		input_state_matrix_t& dLdux)  {

	BASE::getIntermediateCostDerivativeInputState(dLdux);

	dLdux.template block<12,12>(0,12) += copWeight_*devLambdaJoints_copCost_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
void SwitchedModelCostBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::getTerminalCost(
		scalar_t& Phi) {

	BASE::getTerminalCost(Phi);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
void SwitchedModelCostBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::getTerminalCostDerivativeState(
		state_vector_t& dPhidx) {

	BASE::getTerminalCostDerivativeState(dPhidx);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
void SwitchedModelCostBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::getTerminalCostSecondDerivativeState(
		state_matrix_t& dPhidxx)  {

	BASE::getTerminalCostSecondDerivativeState(dPhidxx);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
typename SwitchedModelCostBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::input_matrix_t
	SwitchedModelCostBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::correctedInputCost(
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
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
void SwitchedModelCostBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::copErrorCostFunc(
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
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
double SwitchedModelCostBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::GaussianFunc (
		const scalar_t& mu,
		const scalar_t& sigma,
		const scalar_t& x) {

	return exp( -0.5 * pow( (x-mu)/sigma ,2) );
}

} // namespace switched_model

