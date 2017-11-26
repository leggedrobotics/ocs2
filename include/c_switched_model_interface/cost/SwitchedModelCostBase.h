/*
 * SwitchedModelCostBase.h
 *
 *  Created on: Nov 23, 2017
 *      Author: farbod
 */

#ifndef SWITCHEDMODELCOSTBASE_H_
#define SWITCHEDMODELCOSTBASE_H_

#include <array>
#include <memory>
#include <iostream>
#include <Eigen/StdVector>
#include <Eigen/Dense>

#include <ocs2_core/misc/LinearInterpolation.h>
#include <ocs2_core/cost/CostFunctionBaseOCS2.h>

#include "c_switched_model_interface/core/SwitchedModel.h"
#include "c_switched_model_interface/core/KinematicsModelBase.h"
#include "c_switched_model_interface/core/ComModelBase.h"
#include "c_switched_model_interface/core/CopEstimator.h"

namespace switched_model {

template <size_t JOINT_COORD_SIZE>
class SwitchedModelCostBase : public ocs2::CostFunctionBaseOCS2<12+JOINT_COORD_SIZE, 12+JOINT_COORD_SIZE>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef ComModelBase<JOINT_COORD_SIZE> com_model_t;
	typedef KinematicsModelBase<JOINT_COORD_SIZE> kinematic_model_t;
	typedef ocs2::CostFunctionBaseOCS2<12+JOINT_COORD_SIZE, 12+JOINT_COORD_SIZE> Base;

	typedef typename SwitchedModel<JOINT_COORD_SIZE>::base_coordinate_t  base_coordinate_t;
	typedef typename SwitchedModel<JOINT_COORD_SIZE>::joint_coordinate_t joint_coordinate_t;

	typedef typename Base::scalar_t       		scalar_t;
	typedef typename Base::scalar_array_t 		scalar_array_t;
	typedef typename Base::state_vector_t 		state_vector_t;
	typedef typename Base::state_vector_array_t state_vector_array_t;
	typedef typename Base::state_matrix_t 		state_matrix_t;
	typedef typename Base::control_vector_t  	control_vector_t;
	typedef typename Base::control_vector_array_t control_vector_array_t;
	typedef typename Base::control_matrix_t 	control_matrix_t;
	typedef typename Base::control_feedback_t 	control_feedback_t;

	SwitchedModelCostBase(kinematic_model_t* kinematicModelPtr, com_model_t* comModelPtr,
			const std::array<bool,4>& stanceLegs,
			const state_matrix_t& Q,
			const control_matrix_t& R,
			const scalar_array_t& tNominalTrajectory,
			const state_vector_array_t& xNominalTrajectory,
			const control_vector_array_t& uNominalTrajectory,
			const state_matrix_t& QFinal,
			const state_vector_t& xFinal,
			const double& copWeightMax = 0.0,
			const state_matrix_t& QIntermediate = state_matrix_t::Zero(),
			const state_vector_t& xNominalIntermediate = state_vector_t::Zero(),
			const double& sigma = 1,
			const double& tp = 0)

	: kinematicModelPtr_(kinematicModelPtr),
	  comModelPtr_(comModelPtr),
	  stanceLegs_(stanceLegs),
	  Q_desired_(Q),
	  R_(R),
	  tNominalTrajectory_(tNominalTrajectory),
	  xNominalTrajectory_(xNominalTrajectory),
	  uNominalTrajectory_(uNominalTrajectory),
	  xNominalFunc_(&tNominalTrajectory_, &xNominalTrajectory_),
	  uNominalFunc_(&tNominalTrajectory_, &uNominalTrajectory_),
	  QFinal_desired_(QFinal),
	  xFinal_(xFinal),
	  copWeightMax_(copWeightMax),
	  QIntermediate_(QIntermediate),
	  xNominalIntermediate_(xNominalIntermediate),
	  sigma_(sigma),
	  sigmaSquared_(sigma*sigma),
	  normalization_(1.0 / (sigma_ * std::sqrt(2.0*3.14)) ),
	  tp_(tp),
	  dtSquared_(0.0),
	  copEstimatorPtr_(new CopEstimator<JOINT_COORD_SIZE>(kinematicModelPtr->clone(), comModelPtr->clone()))
	{}

	/**
	 * copy construntor
	 */
	SwitchedModelCostBase(const SwitchedModelCostBase& rhs)

	: Base(rhs),
	  kinematicModelPtr_(rhs.kinematicModelPtr_->clone()),
	  comModelPtr_(rhs.comModelPtr_->clone()),
	  stanceLegs_(rhs.stanceLegs_),
	  Q_desired_(rhs.Q_desired_),
	  R_(rhs.R_),
	  tNominalTrajectory_(rhs.tNominalTrajectory_),
	  xNominalTrajectory_(rhs.xNominalTrajectory_),
	  uNominalTrajectory_(rhs.uNominalTrajectory_),
	  xNominalFunc_(&tNominalTrajectory_, &xNominalTrajectory_),
	  uNominalFunc_(&tNominalTrajectory_, &uNominalTrajectory_),
	  QFinal_desired_(rhs.QFinal_desired_),
	  xFinal_(rhs.xFinal_),
	  copWeightMax_(rhs.copWeightMax_),
	  QIntermediate_(rhs.QIntermediate_),
	  xNominalIntermediate_(rhs.xNominalIntermediate_),
	  sigma_(rhs.sigma_),
	  sigmaSquared_(rhs.sigmaSquared_),
	  normalization_(rhs.normalization_),
	  tp_(rhs.tp_),
	  dtSquared_(rhs.dtSquared_),
	  copEstimatorPtr_(new CopEstimator<JOINT_COORD_SIZE>(rhs.kinematicModelPtr_->clone(), rhs.comModelPtr_->clone()))
	{}

	virtual ~SwitchedModelCostBase() {}

	/******************************************************************************************************/
	std::shared_ptr<Base> clone() const;

	/******************************************************************************************************/
	void setCurrentStateAndControl(const scalar_t& t, const state_vector_t& x, const control_vector_t& u) override;

	/******************************************************************************************************/
	void setCostNominalState(const scalar_array_t& timeTrajectory, const state_vector_array_t& stateTrajectory) override;

	/******************************************************************************************************/
	void getCostNominalState(scalar_array_t& timeTrajectory, state_vector_array_t& stateTrajectory) const override;

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

protected:
	/******************************************************************************************************/
	void copErrorCostFunc(const joint_coordinate_t& qJoints, const joint_coordinate_t& lambda,
			double& copCost,
			Eigen::Matrix<double,JOINT_COORD_SIZE,1>& devJoints_copCost,
			Eigen::Matrix<double,JOINT_COORD_SIZE,1>& devLambda_copCost,
			Eigen::Matrix<double,JOINT_COORD_SIZE,JOINT_COORD_SIZE>& hessJoints_copCost,
			Eigen::Matrix<double,JOINT_COORD_SIZE,JOINT_COORD_SIZE>& hessLambda_copCost,
			Eigen::Matrix<double,JOINT_COORD_SIZE,JOINT_COORD_SIZE>& devLambdaJoints_copCost);

	/******************************************************************************************************/
	static double GaussianFunc (const double& mu, const double& sigma, const double& x);

private:
	typename kinematic_model_t::Ptr kinematicModelPtr_;
	typename com_model_t::Ptr comModelPtr_;

	std::array<bool,4> stanceLegs_;

	state_matrix_t Q_;
	state_matrix_t Q_desired_;
	control_matrix_t R_;

	scalar_array_t tNominalTrajectory_;
	state_vector_array_t xNominalTrajectory_;
	control_vector_array_t uNominalTrajectory_;
	state_vector_t xDeviation_;
	control_vector_t uDeviation_;

	ocs2::LinearInterpolation<state_vector_t,Eigen::aligned_allocator<state_vector_t> > xNominalFunc_;
	ocs2::LinearInterpolation<control_vector_t,Eigen::aligned_allocator<control_vector_t> > uNominalFunc_;

	state_matrix_t QFinal_;
	state_matrix_t QFinal_desired_;
	state_vector_t xFinal_;

	double copWeightMax_;
	double copWeight_;
	double copCost_;

	state_matrix_t QIntermediate_;
	state_vector_t xNominalIntermediate_;
	state_vector_t xDeviationIntermediate_;

	const double sigma_;
	const double sigmaSquared_;
	const double normalization_;
	const double tp_;
	double dtSquared_;

	Eigen::Matrix<double,12,1> devJoints_copCost_;
	Eigen::Matrix<double,12,1> devLambda_copCost_;
	Eigen::Matrix<double,12,12> hessJoints_copCost_;
	Eigen::Matrix<double,12,12> hessLambda_copCost_;
	Eigen::Matrix<double,12,12> devLambdaJoints_copCost_;

	typename CopEstimator<JOINT_COORD_SIZE>::Ptr copEstimatorPtr_;

};

} //end of namespace switched_model

#include "implementation/SwitchedModelCostBase.h"

#endif /* SWITCHEDMODELCOSTBASE_H_ */
