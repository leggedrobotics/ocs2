/*
 * SwitchedModelCost.cpp
 *
 *  Created on: May 11, 2017
 *      Author: farbod
 */

#include "ocs2_anymal_cost/SwitchedModelCost.h"

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SwitchedModelCost::setCurrentStateAndControl(const scalar_t& t, const state_vector_t& x, const control_vector_t& u) {

	BASE::setCurrentStateAndControl(t, x, u);

	state_vector_t xNominal;
	control_vector_t uNominal;

	// TODO: fix me. make it consistant
	QFinal_.setZero();
	Q_ = Q_desired_ + QFinal_desired_ / (timeFinal_-timeStart_);
//	if (timeFinal_-timeStart_ > std::numeric_limits<double>::epsilon()) {
//
//		QFinal_.setZero();
//		Q_ = Q_desired_ + QFinal_desired_ / (timeFinal_-timeStart_);
//
//	} else {
//
//		QFinal_ = QFinal_desired_;
//		Q_ = Q_desired_;
//	}

	xNominalFunc_.interpolate(t, xNominal);
	xDeviation_ = x - xNominal;
	uNominalFunc_.interpolate(t, uNominal);
	uDeviation_ = u - uNominal;

	xDeviationIntermediate_ = x - xNominalIntermediate_;

	dtSquared_ = (t-tp_) * (t-tp_);

	/* ZMP constraint */
	if (copWeightMax_ > std::numeric_limits<double>::epsilon())  {

		copWeight_ = copWeightMax_ * std::exp( -0.5 * std::pow((t-timeMean_)/timeSD_, 2) );

		copErrorCostFunc(x.tail<12>(), u.head<12>(), copCost_, devJoints_copCost_, devLambda_copCost_,
				hessJoints_copCost_, hessLambda_copCost_, devLambdaJoints_copCost_);
//		Eigen::Matrix<double,12,1> numDevJoints_copCost;
//		Eigen::Matrix<double,12,1> numDevLambda_copCost;
//		numCopDev(x.tail<12>(), u.head<12>(), copCost_, numDevJoints_copCost, numDevLambda_copCost);
//
//		if (numDevJoints_copCost.isApprox(devJoints_copCost_,1e-3) != true) {
//			std::cout << "devJoints_copCost_:   " << devJoints_copCost_.transpose() << std::endl;
//			std::cout << "numDevJoints_copCost: " << numDevJoints_copCost.transpose() << std::endl;
//			std::cout << "Deff: " << (numDevJoints_copCost - devJoints_copCost_).transpose() << std::endl;
//		}
//
//		if (numDevLambda_copCost.isApprox(devLambda_copCost_,1e-3) != true) {
//			std::cout << "devLambda_copCost_:   " << devLambda_copCost_.transpose() << std::endl;
//			std::cout << "numDevLambda_copCost: " << numDevLambda_copCost.transpose() << std::endl;
//			std::cout << "Deff: " << (numDevLambda_copCost - devLambda_copCost_).transpose() << std::endl;
//		}

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
void SwitchedModelCost::setCostNominalState(const scalar_array_t& timeTrajectory,
		const state_vector_array_t& stateTrajectory) {

	tNominalTrajectory_ = timeTrajectory;
	xNominalTrajectory_ = stateTrajectory;
	xNominalFunc_.setTimeStamp(&tNominalTrajectory_);
	xNominalFunc_.setData(&xNominalTrajectory_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SwitchedModelCost::getCostNominalState(scalar_array_t& timeTrajectory,
		state_vector_array_t& stateTrajectory) const {

	timeTrajectory  = tNominalTrajectory_;
	stateTrajectory = xNominalTrajectory_;
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SwitchedModelCost::numCopDev (const Eigen::Matrix<double,12,1>& qJoints, const Eigen::Matrix<double,12,1>& lambda,
		const double& copCost, Eigen::Matrix<double,12,1>& numDevJoints_copCost, Eigen::Matrix<double,12,1>& numDevLambda_copCost) {

	const double eps = 1e-8;

	double copCostPlus;
	Eigen::Matrix<double,12,1> devJoints_copCost;
	Eigen::Matrix<double,12,1> devLambda_copCost;
	Eigen::Matrix<double,12,12> hessJoints_copCost;
	Eigen::Matrix<double,12,12> hessLambda_copCost;
	Eigen::Matrix<double,12,12> devLambdaJoints_copCost;
	for (size_t i=0; i<12; i++) {
		Eigen::Matrix<double,12,1> qJointsPlus = qJoints;
		qJointsPlus(i) += eps;
		copErrorCostFunc(qJointsPlus, lambda, copCostPlus, devJoints_copCost, devLambda_copCost,
				hessJoints_copCost, hessLambda_copCost, devLambdaJoints_copCost);
		numDevJoints_copCost(i) = (copCostPlus-copCost) / eps;

		Eigen::Matrix<double,12,1> lambdaPlus = lambda;
		lambdaPlus(i) += eps;
		copErrorCostFunc(qJoints, lambdaPlus, copCostPlus, devJoints_copCost, devLambda_copCost,
				hessJoints_copCost, hessLambda_copCost, devLambdaJoints_copCost);
		numDevLambda_copCost(i) = (copCostPlus-copCost) / eps;
	}
}



/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SwitchedModelCost::evaluate(scalar_t& L)  {

	scalar_t costQ = 0.5 * xDeviation_.transpose() * Q_ * xDeviation_;

	scalar_t costQintermediate = 0.5 * normalization_ * std::exp(-0.5 * dtSquared_ / sigmaSquared_) *
			xDeviationIntermediate_.transpose() * QIntermediate_ * xDeviationIntermediate_;
	scalar_t costR = 0.5 * uDeviation_.transpose() * R_ * uDeviation_;

	L = costQ + costQintermediate + costR + copWeight_*copCost_;

//	scalar_t costSwingLeg  = 0.5 * BASE::u_.tail<12>().transpose() * R_.bottomRightCorner<12,12>() * BASE::u_.tail<12>();
//	scalar_t costTakeoff   = costSwingLeg * GaussianFunc(timeStart_, impulseSigmeFactor_*(BASE::timeFinal_-timeStart_), BASE::t_);
//	scalar_t costTouchdown = costSwingLeg * GaussianFunc(BASE::timeFinal_, impulseSigmeFactor_*(BASE::timeFinal_-timeStart_), BASE::t_);
//	L += impulseWeight_*(costTakeoff + costTouchdown);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SwitchedModelCost::stateDerivative(state_vector_t& dLdx)  {

	state_vector_t costQ = Q_ * xDeviation_;
	state_vector_t costQintermediate = QIntermediate_ * xDeviationIntermediate_ * normalization_ * std::exp(-0.5 * dtSquared_ / sigmaSquared_);
	dLdx = costQ + costQintermediate;

	dLdx.tail<12>() += copWeight_*devJoints_copCost_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SwitchedModelCost::stateSecondDerivative(state_matrix_t& dLdxx)  {
	dLdxx = Q_ + QIntermediate_ * normalization_ * std::exp(-0.5 * dtSquared_ / sigmaSquared_);
	dLdxx.bottomRightCorner<12,12>() += copWeight_*hessJoints_copCost_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SwitchedModelCost::controlDerivative(control_vector_t& dLdu)  {
	dLdu = R_ * uDeviation_;
	dLdu.head<12>() += copWeight_*devLambda_copCost_;

//		Eigen::Matrix<double,12,1> costSwingLegDerivative  = R_.bottomRightCorner<12,12>() * BASE::u_.tail<12>();
//		Eigen::Matrix<double,12,1> costTakeoffDerivative   = costSwingLegDerivative
//				* GaussianFunc(timeStart_, impulseSigmeFactor_*(BASE::timeFinal_-timeStart_), BASE::t_);
//		Eigen::Matrix<double,12,1> costTouchdownDerivative = costSwingLegDerivative
//				* GaussianFunc(BASE::timeFinal_, impulseSigmeFactor_*(BASE::timeFinal_-timeStart_), BASE::t_);
//		dLdu.tail<12>() += impulseWeight_*(costTakeoffDerivative + costTouchdownDerivative);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SwitchedModelCost::controlSecondDerivative(control_matrix_t& dLduu)  {
	dLduu = R_;
	dLduu.topLeftCorner<12,12>() += copWeight_*hessLambda_copCost_;

//		Eigen::Matrix<double,12,12> costTakeoffSecondDerivative   = R_.bottomRightCorner<12,12>()
//				* GaussianFunc(timeStart_, impulseSigmeFactor_*(BASE::timeFinal_-timeStart_), BASE::t_);
//		Eigen::Matrix<double,12,12> costTouchdownSecondDerivative = R_.bottomRightCorner<12,12>()
//				* GaussianFunc(BASE::timeFinal_, impulseSigmeFactor_*(BASE::timeFinal_-timeStart_), BASE::t_);
//		dLduu.bottomRightCorner<12,12>() += impulseWeight_*(costTakeoffSecondDerivative + costTouchdownSecondDerivative);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SwitchedModelCost::stateControlDerivative(control_feedback_t& dLdxu)  {
	dLdxu.setZero();
	dLdxu.topRightCorner<12,12>() += copWeight_*devLambdaJoints_copCost_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SwitchedModelCost::terminalCost(scalar_t& Phi) {

	state_vector_t x_deviation_final = x_ - xFinal_;
	Phi = 0.5 * x_deviation_final.transpose() * QFinal_ * x_deviation_final;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SwitchedModelCost::terminalCostStateDerivative(state_vector_t& dPhidx)
{
	state_vector_t x_deviation_final = x_ - xFinal_;
	dPhidx =  QFinal_ * x_deviation_final;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SwitchedModelCost::terminalCostStateSecondDerivative(state_matrix_t& dPhidxx)  {
	dPhidxx = QFinal_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SwitchedModelCost::copErrorCostFunc(const Eigen::Matrix<double,12,1>& qJoints, const Eigen::Matrix<double,12,1>& lambda,
		double& copCost, Eigen::Matrix<double,12,1>& devJoints_copCost, Eigen::Matrix<double,12,1>& devLambda_copCost,
		Eigen::Matrix<double,12,12>& hessJoints_copCost, Eigen::Matrix<double,12,12>& hessLambda_copCost,
		Eigen::Matrix<double,12,12>& devLambdaJoints_copCost) {

	std::array<Eigen::Vector2d,4> b_feetPosition;
	double numStanceLegs = 0.0;
	Eigen::Matrix<double,2,1>  b_totalPressureMoment = Eigen::Matrix<double,2,1>::Zero();
	Eigen::Matrix<double,2,12> devJoints_b_totalPressureMoment = Eigen::Matrix<double,2,12>::Zero();
	double totalPressure = 0.0;
	Eigen::Matrix<double,2,1>  b_base2CopDesired = Eigen::Matrix<double,2,1>::Zero();
	Eigen::Matrix<double,2,12> devJoints_b_base2CopDesired = Eigen::Matrix<double,2,12>::Zero();
	for (size_t j=0; j<4; j++)
		if (stanceLegs_[j]==true) {

			Eigen::Vector3d b_base2StanceFoot;
			hyq::SwitchedModelKinematics::FootPositionBaseFrame(qJoints, j, b_base2StanceFoot);
			Eigen::Matrix<double,6,12> b_jacobianFoot;
			hyq::SwitchedModelKinematics::FootJacobainBaseFrame(qJoints, j, b_jacobianFoot);

			// stance feet [x,y]'
			b_feetPosition[j] = b_base2StanceFoot.head<2>();

			// += stanceLeg
			numStanceLegs += 1;
			// =+ [x,y]' lambda_z
			b_totalPressureMoment += b_feetPosition[j]*lambda(3*j+2);
			// =+ d/dq [x,y]' lambda_z
			devJoints_b_totalPressureMoment += b_jacobianFoot.block<2,12>(3,0)*lambda(3*j+2);

			// += lambda_z
			totalPressure += lambda(3*j+2);
			// =+ [x,y]'
			b_base2CopDesired += b_feetPosition[j];
			// =+ d/dq [x,y]'
			devJoints_b_base2CopDesired += b_jacobianFoot.block<2,12>(3,0);
		}

	if (numStanceLegs > 0) {
		b_base2CopDesired /= numStanceLegs;
		devJoints_b_base2CopDesired /= numStanceLegs;
	}

	// copError = M_total - cop_des * lambda_total
	Eigen::Matrix<double,2,1> copError = b_totalPressureMoment - b_base2CopDesired*totalPressure;


	Eigen::Matrix<double,2,12> devLambda_b_base2Cop = Eigen::Matrix<double,2,12>::Zero(); //
	Eigen::Matrix<double,2,12> devLambda_b_base2CopDesired = Eigen::Matrix<double,2,12>::Zero(); //
	for (size_t j=0; j<4; j++)
		if (stanceLegs_[j]==true) {

			Eigen::Matrix<double,1,12> devLambdaZ = Eigen::Matrix<double,1,12>::Zero();
			devLambdaZ(3*j+2) = 1.0;
			devLambda_b_base2Cop += b_feetPosition[j]*devLambdaZ;
		}

	Eigen::Matrix<double,2,12> devJoints_copError = devJoints_b_totalPressureMoment-devJoints_b_base2CopDesired*totalPressure;

	Eigen::Matrix<double,2,12> devLambda_copError = Eigen::Matrix<double,2,12>::Zero();
	for (size_t j=0; j<4; j++)
		if (stanceLegs_[j]==true)
			devLambda_copError.col(3*j+2) = b_feetPosition[j] - b_base2CopDesired;

	const double totalWeight = 73.172*9.81;

	copCost = 0.5*copError.squaredNorm() / pow(totalWeight,2);
	devJoints_copCost = devJoints_copError.transpose() * copError / pow(totalWeight,2);
	devLambda_copCost = devLambda_copError.transpose() * copError / pow(totalWeight,2);

	hessJoints_copCost = devJoints_copError.transpose() * devJoints_copError / pow(totalWeight,2);
	hessLambda_copCost = devLambda_copError.transpose() * devLambda_copError / pow(totalWeight,2);
	devLambdaJoints_copCost = devLambda_copError.transpose() * devJoints_copError / pow(totalWeight,2);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
double SwitchedModelCost::GaussianFunc (const double& mu, const double& sigma, const double& x) {
	return exp( -0.5 * pow( (x-mu)/sigma ,2) );
}

