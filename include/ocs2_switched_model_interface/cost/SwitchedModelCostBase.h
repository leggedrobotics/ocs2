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
#include <vector>
#include <map>

#include <ocs2_core/cost/QuadraticCostFunction.h>

#include "ocs2_switched_model_interface/core/ComModelBase.h"
#include "ocs2_switched_model_interface/core/CopEstimator.h"
#include "ocs2_switched_model_interface/core/KinematicsModelBase.h"
#include "ocs2_switched_model_interface/core/MotionPhaseDefinition.h"
#include "ocs2_switched_model_interface/core/SwitchedModel.h"
#include "ocs2_switched_model_interface/logic/SwitchedModelLogicRulesBase.h"

namespace switched_model {

template <size_t JOINT_COORD_SIZE,
		size_t STATE_DIM=12+JOINT_COORD_SIZE,
		size_t INPUT_DIM=12+JOINT_COORD_SIZE>
class SwitchedModelCostBase : public ocs2::QuadraticCostFunction<STATE_DIM, INPUT_DIM>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	enum : size_t
	{
		STATE_DIM_ = STATE_DIM,
		INPUT_DIM_ = INPUT_DIM,
		NUM_CONTACT_POINTS_ = SwitchedModel<JOINT_COORD_SIZE>::NUM_CONTACT_POINTS
	};

	typedef SwitchedModelPlannerLogicRules<JOINT_COORD_SIZE, double> logic_rules_t;

	typedef ocs2::QuadraticCostFunction<STATE_DIM, INPUT_DIM> BASE;

	typedef ComModelBase<JOINT_COORD_SIZE> com_model_t;
	typedef KinematicsModelBase<JOINT_COORD_SIZE> kinematic_model_t;
	typedef CopEstimator<JOINT_COORD_SIZE> cop_estimator_t;

	typedef typename SwitchedModel<JOINT_COORD_SIZE>::contact_flag_t	 contact_flag_t;
	typedef typename SwitchedModel<JOINT_COORD_SIZE>::base_coordinate_t  base_coordinate_t;
	typedef typename SwitchedModel<JOINT_COORD_SIZE>::joint_coordinate_t joint_coordinate_t;

	typedef typename BASE::scalar_t       		scalar_t;
	typedef typename BASE::scalar_array_t 		scalar_array_t;
	typedef typename BASE::state_vector_t 		state_vector_t;
	typedef typename BASE::state_vector_array_t state_vector_array_t;
	typedef typename BASE::state_matrix_t 		state_matrix_t;
	typedef typename BASE::input_vector_t  		input_vector_t;
	typedef typename BASE::input_vector_array_t input_vector_array_t;
	typedef typename BASE::input_matrix_t 	    input_matrix_t;
	typedef typename BASE::input_state_matrix_t input_state_matrix_t;
	typedef typename BASE::dynamic_vector_t       dynamic_vector_t;
	typedef typename BASE::dynamic_vector_array_t dynamic_vector_array_t;
	typedef typename BASE::cost_desired_trajectories_t cost_desired_trajectories_t;

	/**
	 * Constructor
	 */
	SwitchedModelCostBase(
			const kinematic_model_t& kinematicModel,
			const com_model_t& comModel,
			std::shared_ptr<const logic_rules_t> logicRulesPtr,
			const state_matrix_t& Q,
			const input_matrix_t& R,
			const state_matrix_t& QFinal,
			const state_vector_t& xNominalFinal,
			const scalar_t& copWeightMax = 0.0,
			const state_matrix_t& QIntermediateGoal = state_matrix_t::Zero(),
			const state_vector_t& xIntermediateGoal = state_vector_t::Zero(),
			const scalar_t& sigma = 1.0,
			const scalar_t& tp = 0.0);

	/**
	 * copy constructor
	 */
	SwitchedModelCostBase(const SwitchedModelCostBase& rhs);

	/**
	 * Destructor
	 */
	 ~SwitchedModelCostBase() override = default;

	/**
	 * clone SwitchedModelCostBase class.
	 */
	 SwitchedModelCostBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>* clone() const override;
	
	/**
	 * Set the current state and contact force input
	 *
	 * @param t: current time
	 * @param x: current switched state vector (centroidal dynamics plus joints' angles)
	 * @param u: current switched input vector (contact forces plus joints' velocities)
	 */
	 void setCurrentStateAndControl(
			const scalar_t& t,
			const state_vector_t& x,
			const input_vector_t& u) override;

    /**
     * Evaluates the cost.
     *
     * @param [out] L: The cost value.
     */
	 void getIntermediateCost(scalar_t& L) override;

    /**
     * Gets the state derivative.
     *
     * @param [out] dLdx: First order cost derivative with respect to state vector.
     */
	 void getIntermediateCostDerivativeState(state_vector_t& dLdx) override;

    /**
     * Gets state second order derivative.
     *
     * @param [out] dLdxx: Second order cost derivative with respect to state vector.
     */
	 void getIntermediateCostSecondDerivativeState(state_matrix_t& dLdxx) override;

    /**
     * Gets control input derivative.
     *
     * @param [out] dLdu: First order cost derivative with respect to input vector.
     */
	 void getIntermediateCostDerivativeInput(input_vector_t& dLdu) override;

    /**
     * Gets control input second derivative.
     *
     * @param [out] dLduu: Second order cost derivative with respect to input vector.
     */
	 void getIntermediateCostSecondDerivativeInput(input_matrix_t& dLduu) override;

    /**
     * Gets the state, control input derivative.
     *
     * @param [out] dLdux: Second order cost derivative with respect to state and input vector.
     */
	 void getIntermediateCostDerivativeInputState(input_state_matrix_t& dLdux) override;

    /**
     * Gets the terminal cost.
     *
     * @param [out] Phi: The final cost value
     */
	 void getTerminalCost(scalar_t& Phi) override;

    /**
     * Gets the terminal cost state derivative.
     *
     * @param [out] dPhidx: First order final cost derivative with respect to state vector.
     */
	 void getTerminalCostDerivativeState(state_vector_t& dPhidx) override;

    /**
     * Gets the terminal cost state second derivative
     *
     * @param [out] dPhidxx: Second order final cost derivative with respect to state vector.
     */
	 void getTerminalCostSecondDerivativeState(state_matrix_t& dPhidxx) override;

protected:

	/**
	 *
	 * @param stanceLeg
	 * @param R
	 * @return
	 */
	input_matrix_t correctedInputCost(
			const contact_flag_t& stanceLeg,
			const input_matrix_t& R);

	/**
	 *
	 * @param qJoints
	 * @param lambda
	 * @param copCost
	 * @param devJoints_copCost
	 * @param devLambda_copCost
	 * @param hessJoints_copCost
	 * @param hessLambda_copCost
	 * @param devLambdaJoints_copCost
	 */
	void copErrorCostFunc(
			const joint_coordinate_t& qJoints,
			const joint_coordinate_t& lambda,
			double& copCost,
			Eigen::Matrix<double,JOINT_COORD_SIZE,1>& devJoints_copCost,
			Eigen::Matrix<double,JOINT_COORD_SIZE,1>& devLambda_copCost,
			Eigen::Matrix<double,JOINT_COORD_SIZE,JOINT_COORD_SIZE>& hessJoints_copCost,
			Eigen::Matrix<double,JOINT_COORD_SIZE,JOINT_COORD_SIZE>& hessLambda_copCost,
			Eigen::Matrix<double,JOINT_COORD_SIZE,JOINT_COORD_SIZE>& devLambdaJoints_copCost);

	/**
	 *
	 * @param mu
	 * @param sigma
	 * @param x
	 * @return
	 */
	static double GaussianFunc (const scalar_t& mu, const scalar_t& sigma, const scalar_t& x);

private:
	typename kinematic_model_t::Ptr kinematicModelPtr_;
	typename com_model_t::Ptr comModelPtr_;

	std::shared_ptr<const logic_rules_t> logicRulesPtr_;

	std::function<size_t(scalar_t)> findActiveSubsystemFnc_;

	contact_flag_t stanceLegs_;

	std::map<contact_flag_t, input_matrix_t, std::less<contact_flag_t>,
	  Eigen::aligned_allocator<std::pair<const contact_flag_t, input_matrix_t>> > R_Bank_;

	scalar_t copWeightMax_;
	scalar_t copWeight_;
	scalar_t copCost_;

	state_matrix_t QIntermediateGoal_;
	state_vector_t xIntermediateGoal_;
	state_vector_t xIntermediateDeviationGoal_;

	const scalar_t sigma_;
	const scalar_t sigmaSquared_;
	const scalar_t normalization_;
	const scalar_t tp_;
	scalar_t dtSquared_;

	state_vector_t xIntermediateDeviation_;
	input_vector_t uIntermediateDeviation_;
  	Eigen::VectorXd xNominal_;
  	Eigen::VectorXd uNominal_;

	typename cop_estimator_t::Ptr copEstimatorPtr_;

	Eigen::Matrix<scalar_t,12,1>  devJoints_copCost_;
	Eigen::Matrix<scalar_t,12,1>  devLambda_copCost_;
	Eigen::Matrix<scalar_t,12,12> hessJoints_copCost_;
	Eigen::Matrix<scalar_t,12,12> hessLambda_copCost_;
	Eigen::Matrix<scalar_t,12,12> devLambdaJoints_copCost_;

  	void inputFromContactFlags(contact_flag_t contactFlags, dynamic_vector_t& inputs);
};

} //end of namespace switched_model

#include "implementation/SwitchedModelCostBase.h"

#endif /* SWITCHEDMODELCOSTBASE_H_ */
