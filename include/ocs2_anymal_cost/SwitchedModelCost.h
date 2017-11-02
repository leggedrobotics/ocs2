/*
 * SwitchedModelCost.h
 *
 *  Created on: Jul 6, 2016
 *      Author: farbod
 */

#ifndef SWITCHEDMODELCOST_H_
#define SWITCHEDMODELCOST_H_

#include <array>

#include <kinematics/SwitchedModelKinematics.h>

#include <ocs2_core/misc/LinearInterpolation.h>
#include <ocs2_core/cost/CostFunctionBaseOCS2.h>


class SwitchedModelCost : public ocs2::CostFunctionBaseOCS2<12+12, 12+12>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef ocs2::CostFunctionBaseOCS2<12+12, 12+12> BASE;

	/******************************************************************************************************/
	SwitchedModelCost(const std::array<bool,4>& stanceLegs,
			const state_matrix_t& Q,
			const control_matrix_t& R,
			const scalar_array_t&         tNominalTrajectory,
			const state_vector_array_t&   xNominalTrajectory,
			const control_vector_array_t& uNominalTrajectory,
			const state_matrix_t& QFinal,
			const state_vector_t& xFinal,
			const double& copWeightMax = 0.0,
			const double& impulseWeight = 0.0,
			const double& impulseSigmeFactor = 1.0/9.0,
			state_matrix_t QIntermediate = state_matrix_t::Zero(),
			const state_vector_t xNominalIntermediate = state_vector_t::Zero(),
			const double sigma = 1,
			const double tp = 0)

	: stanceLegs_(stanceLegs),
	  Q_desired_(Q),
	  R_(R),
	  tNominalTrajectory_(tNominalTrajectory),
	  xNominalTrajectory_(xNominalTrajectory),
	  uNominalTrajectory_(uNominalTrajectory),
	  xNominalFunc_(&tNominalTrajectory_, &xNominalTrajectory_),
	  uNominalFunc_(&tNominalTrajectory_, &uNominalTrajectory_),
	  QIntermediate_(QIntermediate),
	  xNominalIntermediate_(xNominalIntermediate),
	  sigma_(sigma),
	  sigmaSquared_(sigma*sigma),
	  normalization_(1.0 / (sigma_ * std::sqrt(2.0*3.14)) ),
	  tp_(tp),
	  dtSquared_(0.0),
	  xFinal_(xFinal),
	  QFinal_desired_(QFinal),
	  copWeightMax_(copWeightMax),
	  impulseWeight_(impulseWeight),
	  impulseSigmeFactor_(impulseSigmeFactor)
	{}

	/******************************************************************************************************/
	~SwitchedModelCost() {}

	/******************************************************************************************************/
	void setCurrentStateAndControl(const scalar_t& t, const state_vector_t& x, const control_vector_t& u) override;

	/******************************************************************************************************/
	void setCostNominalState(const scalar_array_t& timeTrajectory, const state_vector_array_t& stateTrajectory) override;

	/******************************************************************************************************/
	void getCostNominalState(scalar_array_t& timeTrajectory, state_vector_array_t& stateTrajectory) const override;

	/******************************************************************************************************/
	// for test
	void numCopDev (const Eigen::Matrix<double,12,1>& qJoints, const Eigen::Matrix<double,12,1>& lambda,
			const double& copCost, Eigen::Matrix<double,12,1>& numDevJoints_copCost, Eigen::Matrix<double,12,1>& numDevLambda_copCost);

	/******************************************************************************************************/
	void evaluate(scalar_t& L) override;

	/******************************************************************************************************/
	void stateDerivative(state_vector_t& dLdx) override;

	/******************************************************************************************************/
	void stateSecondDerivative(state_matrix_t& dLdxx) override;

	/******************************************************************************************************/
	void controlDerivative(control_vector_t& dLdu) override;

	/******************************************************************************************************/
	void controlSecondDerivative(control_matrix_t& dLduu) override;

	/******************************************************************************************************/
	void stateControlDerivative(control_feedback_t& dLdxu) override;

	/******************************************************************************************************/
	void terminalCost(scalar_t& Phi) override;

	/******************************************************************************************************/
	void terminalCostStateDerivative(state_vector_t& dPhidx) override;

	/******************************************************************************************************/
	void terminalCostStateSecondDerivative(state_matrix_t& dPhidxx) override;

	/******************************************************************************************************/
	std::shared_ptr<BASE> clone() const {
		return std::allocate_shared<SwitchedModelCost, Eigen::aligned_allocator<SwitchedModelCost>>( Eigen::aligned_allocator<SwitchedModelCost>(), *this);
	}

protected:
	/******************************************************************************************************/
	void copErrorCostFunc(const Eigen::Matrix<double,12,1>& qJoints, const Eigen::Matrix<double,12,1>& lambda,
			double& copCost, Eigen::Matrix<double,12,1>& devJoints_copCost, Eigen::Matrix<double,12,1>& devLambda_copCost,
			Eigen::Matrix<double,12,12>& hessJoints_copCost, Eigen::Matrix<double,12,12>& hessLambda_copCost,
			Eigen::Matrix<double,12,12>& devLambdaJoints_copCost);

	/******************************************************************************************************/
	static double GaussianFunc (const double& mu, const double& sigma, const double& x);


private:
	std::array<bool,4> stanceLegs_;

	state_vector_t xDeviation_;
	control_vector_t uDeviation_;

	state_matrix_t Q_;
	state_matrix_t Q_desired_;
	control_matrix_t R_;

	scalar_array_t tNominalTrajectory_;
	state_vector_array_t xNominalTrajectory_;
	control_vector_array_t uNominalTrajectory_;

	ocs2::LinearInterpolation<state_vector_t,Eigen::aligned_allocator<state_vector_t> > xNominalFunc_;
	ocs2::LinearInterpolation<control_vector_t,Eigen::aligned_allocator<control_vector_t> > uNominalFunc_;

	state_matrix_t QIntermediate_;
	state_vector_t xDeviationIntermediate_;
	state_vector_t xNominalIntermediate_;

	double sigma_;
	double sigmaSquared_;
	double normalization_;
	double tp_;
	double dtSquared_;

	state_vector_t xFinal_;
	state_matrix_t QFinal_;
	state_matrix_t QFinal_desired_;

	double copWeight_;
	double copWeightMax_;
	double impulseWeight_;
	double impulseSigmeFactor_;
	double copCost_;
	Eigen::Matrix<double,12,1> devJoints_copCost_;
	Eigen::Matrix<double,12,1> devLambda_copCost_;

	Eigen::Matrix<double,12,12> hessJoints_copCost_;
	Eigen::Matrix<double,12,12> hessLambda_copCost_;
	Eigen::Matrix<double,12,12> devLambdaJoints_copCost_;

};


#endif /* SWITCHEDMODELCOST_H_ */
