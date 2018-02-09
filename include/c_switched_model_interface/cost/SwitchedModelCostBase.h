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
#include <ocs2_core/cost/CostFunctionBase.h>

#include "c_switched_model_interface/core/SwitchedModel.h"
#include "c_switched_model_interface/core/KinematicsModelBase.h"
#include "c_switched_model_interface/core/ComModelBase.h"
#include "c_switched_model_interface/core/CopEstimator.h"
#include "c_switched_model_interface/logic/SwitchedModelLogicRulesBase.h"

namespace switched_model {

template <size_t JOINT_COORD_SIZE>
class SwitchedModelCostBase : public ocs2::CostFunctionBase<12+JOINT_COORD_SIZE, 12+JOINT_COORD_SIZE, SwitchedModelLogicRulesBase<JOINT_COORD_SIZE>>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	enum
	{
		STATE_DIM = 12+JOINT_COORD_SIZE,
		INPUT_DIM = 12+JOINT_COORD_SIZE
	};

	typedef SwitchedModelLogicRulesBase<JOINT_COORD_SIZE> logic_rules_t;
	typedef ocs2::LogicRulesMachine<STATE_DIM, INPUT_DIM, logic_rules_t> logic_rules_machine_t;

	typedef ocs2::CostFunctionBase<STATE_DIM, INPUT_DIM, logic_rules_t> Base;

	typedef ComModelBase<JOINT_COORD_SIZE> com_model_t;
	typedef KinematicsModelBase<JOINT_COORD_SIZE> kinematic_model_t;

	typedef typename SwitchedModel<JOINT_COORD_SIZE>::base_coordinate_t  base_coordinate_t;
	typedef typename SwitchedModel<JOINT_COORD_SIZE>::joint_coordinate_t joint_coordinate_t;

	typedef typename Base::scalar_t       		scalar_t;
	typedef typename Base::scalar_array_t 		scalar_array_t;
	typedef typename Base::state_vector_t 		state_vector_t;
	typedef typename Base::state_vector_array_t state_vector_array_t;
	typedef typename Base::state_matrix_t 		state_matrix_t;
	typedef typename Base::input_vector_t  		input_vector_t;
	typedef typename Base::input_vector_array_t input_vector_array_t;
	typedef typename Base::control_matrix_t 	control_matrix_t;
	typedef typename Base::control_feedback_t 	control_feedback_t;

	SwitchedModelCostBase(const kinematic_model_t* kinematicModelPtr, const com_model_t* comModelPtr,
			const state_matrix_t& Q,
			const control_matrix_t& R,
			const state_matrix_t& QFinal,
			const state_vector_t& xFinal,
			const scalar_t& copWeightMax = 0.0,
			const state_matrix_t& QIntermediate = state_matrix_t::Zero(),
			const state_vector_t& xNominalIntermediate = state_vector_t::Zero(),
			const scalar_t& sigma = 1,
			const scalar_t& tp = 0)

	: Base(),
	  kinematicModelPtr_(kinematicModelPtr->clone()),
	  comModelPtr_(comModelPtr->clone()),
	  Q_desired_(Q),
	  R_(R),
	  tNominalTrajectory_(0),
	  xNominalTrajectory_(0),
	  uNominalTrajectory_(0),
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
	{
		xNominalFunc_.setZero();
		uNominalFunc_.setZero();
	}

	/**
	 * copy construntor
	 */
	SwitchedModelCostBase(const SwitchedModelCostBase& rhs)

	: Base(rhs),
	  kinematicModelPtr_(rhs.kinematicModelPtr_->clone()),
	  comModelPtr_(rhs.comModelPtr_->clone()),
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

	/**
	 * clone SwitchedModelCostBase class.
	 */
	virtual SwitchedModelCostBase<JOINT_COORD_SIZE>* clone() const override;

	/**
	 * Initializes the system dynamics. This method should always be called at the very first call of the model.
	 *
	 * @param [in] logicRulesMachine: A class which contains and parse the logic rules e.g
	 * method findActiveSubsystemHandle returns a Lambda expression which can be used to
	 * find the ID of the current active subsystem.
	 * @param [in] partitionIndex: index of the time partition.
	 * @param [in] algorithmName: The algorithm that class this class (default not defined).
	 */
	virtual void initializeModel(const logic_rules_machine_t& logicRulesMachine,
			const size_t& partitionIndex, const char* algorithmName=NULL) override;

	/**
	 * Set the current state and contact force input
	 *
	 * @param t: current time
	 * @param x: current switched state vector (centrodial dynamics plus joints' angles)
	 * @param u: current switched input vector (contact forces plus joints' velocities)
	 */
	virtual void setCurrentStateAndControl(const scalar_t& t, const state_vector_t& x, const input_vector_t& u) override;

    /**
     * Sets the nominal state and input trajectories used in the cost function.
     *
     * @param [in] timeTrajectory: The time trajectory.
     * @param [in] stateTrajectory: The state trajectory.
     * @param [in] inputTrajectory: The inout trajectory.
     */
	virtual void setCostNominalTrajectories(const scalar_array_t& timeTrajectory,
			const state_vector_array_t& stateTrajectory,
			const input_vector_array_t& inputTrajectory = input_vector_array_t()) override;

	/**
	 * Gets the nominal state used in the cost function.
	 *
	 * @param [out] timeTrajectory: The time trajectory.
	 * @param [out] stateTrajectory: The state trajectory.
	 * @param [out] inputTrajectory: The input trajectory.
	 */
	virtual void getCostNominalTrajectories(scalar_array_t& timeTrajectory,
			state_vector_array_t& stateTrajectory,
			input_vector_array_t& inputTrajectory) const override;

    /**
     * Gets the nominal state used in the cost function.
     *
     * @param [out] timeTrajectory: The time trajectory.
     * @param [out] stateTrajectory: The state trajectory.
     */
	virtual void getCostNominalState(scalar_array_t& timeTrajectory, state_vector_array_t& stateTrajectory) const override;

    /**
     * Gets the nominal input used in the cost function.
     *
     * @param [out] timeTrajectory: The time trajectory.
     * @param [out] inputTrajectory: The input trajectory.
     */
	virtual void getCostNominalInput(scalar_array_t& timeTrajectory, input_vector_array_t& inputTrajectory) const override;

    /**
     * Evaluates the cost.
     *
     * @param [out] L: The cost value.
     */
	virtual void evaluate(scalar_t& L) override;

    /**
     * Gets the state derivative.
     *
     * @param [out] dLdx: First order cost derivative with respect to state vector.
     */
	virtual void stateDerivative(state_vector_t& dLdx) override;

    /**
     * Gets state second order derivative.
     *
     * @param [out] dLdxx: Second order cost derivative with respect to state vector.
     */
	virtual void stateSecondDerivative(state_matrix_t& dLdxx) override;

    /**
     * Gets control input derivative.
     *
     * @param [out] dLdu: First order cost derivative with respect to input vector.
     */
	virtual void controlDerivative(input_vector_t& dLdu) override;

    /**
     * Gets control input second derivative.
     *
     * @param [out] dLduu: Second order cost derivative with respect to input vector.
     */
	virtual void controlSecondDerivative(control_matrix_t& dLduu) override;

    /**
     * Gets the state, control input derivative.
     *
     * @param [out] dLdxu: Second order cost derivative with respect to state and input vector.
     */
	virtual void stateControlDerivative(control_feedback_t& dLdxu) override;

    /**
     * Gets the terminal cost.
     *
     * @param [out] Phi: The final cost value
     */
	virtual void terminalCost(scalar_t& Phi) override;

    /**
     * Gets the terminal cost state derivative.
     *
     * @param [out] dPhidx: First order final cost derivative with respect to state vector.
     */
	virtual void terminalCostStateDerivative(state_vector_t& dPhidx) override;

    /**
     * Gets the terminal cost state second derivative
     *
     * @param [out] dPhidxx: Second order final cost derivative with respect to state vector.
     */
	virtual void terminalCostStateSecondDerivative(state_matrix_t& dPhidxx) override;

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
	static double GaussianFunc (const scalar_t& mu, const scalar_t& sigma, const scalar_t& x);

private:
	typename kinematic_model_t::Ptr kinematicModelPtr_;
	typename com_model_t::Ptr comModelPtr_;

	const logic_rules_t* logicRulesPtr_;

	std::function<size_t(scalar_t)> findActiveSubsystemFnc_;

	std::array<bool,4> stanceLegs_;

	state_matrix_t Q_;
	state_matrix_t Q_desired_;
	control_matrix_t R_;

	scalar_array_t tNominalTrajectory_;
	state_vector_array_t xNominalTrajectory_;
	input_vector_array_t uNominalTrajectory_;
	state_vector_t xDeviation_;
	input_vector_t uDeviation_;

	ocs2::LinearInterpolation<state_vector_t,Eigen::aligned_allocator<state_vector_t> > xNominalFunc_;
	ocs2::LinearInterpolation<input_vector_t,Eigen::aligned_allocator<input_vector_t> > uNominalFunc_;

	state_matrix_t QFinal_;
	state_matrix_t QFinal_desired_;
	state_vector_t xFinal_;

	scalar_t copWeightMax_;
	scalar_t copWeight_;
	scalar_t copCost_;

	state_matrix_t QIntermediate_;
	state_vector_t xNominalIntermediate_;
	state_vector_t xDeviationIntermediate_;

	const scalar_t sigma_;
	const scalar_t sigmaSquared_;
	const scalar_t normalization_;
	const scalar_t tp_;
	scalar_t dtSquared_;

	Eigen::Matrix<scalar_t,12,1> devJoints_copCost_;
	Eigen::Matrix<scalar_t,12,1> devLambda_copCost_;
	Eigen::Matrix<scalar_t,12,12> hessJoints_copCost_;
	Eigen::Matrix<scalar_t,12,12> hessLambda_copCost_;
	Eigen::Matrix<scalar_t,12,12> devLambdaJoints_copCost_;

	typename CopEstimator<JOINT_COORD_SIZE>::Ptr copEstimatorPtr_;

	std::string algorithmName_;
};

} //end of namespace switched_model

#include "implementation/SwitchedModelCostBase.h"

#endif /* SWITCHEDMODELCOSTBASE_H_ */
