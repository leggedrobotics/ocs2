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

#include <ocs2_core/cost/CostFunctionBase.h>

#include "c_switched_model_interface/core/SwitchedModel.h"
#include "c_switched_model_interface/core/MotionPhaseDefinition.h"
#include "c_switched_model_interface/core/KinematicsModelBase.h"
#include "c_switched_model_interface/core/ComModelBase.h"
#include "c_switched_model_interface/core/CopEstimator.h"
#include "c_switched_model_interface/logic/SwitchedModelLogicRulesBase.h"

namespace switched_model {

template <size_t JOINT_COORD_SIZE>
class SwitchedModelCostBase : public ocs2::CostFunctionBase<12+JOINT_COORD_SIZE, 12+JOINT_COORD_SIZE, SwitchedModelPlannerLogicRules<JOINT_COORD_SIZE>>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	enum
	{
		STATE_DIM = 12+JOINT_COORD_SIZE,
		INPUT_DIM = 12+JOINT_COORD_SIZE
	};

	typedef SwitchedModelPlannerLogicRules<JOINT_COORD_SIZE> logic_rules_t;
	typedef ocs2::LogicRulesMachine<STATE_DIM, INPUT_DIM, logic_rules_t> logic_rules_machine_t;

	typedef ocs2::CostFunctionBase<STATE_DIM, INPUT_DIM, logic_rules_t> Base;

	typedef ComModelBase<JOINT_COORD_SIZE> com_model_t;
	typedef KinematicsModelBase<JOINT_COORD_SIZE> kinematic_model_t;

	typedef typename SwitchedModel<JOINT_COORD_SIZE>::contact_flag_t	 contact_flag_t;
	typedef typename SwitchedModel<JOINT_COORD_SIZE>::base_coordinate_t  base_coordinate_t;
	typedef typename SwitchedModel<JOINT_COORD_SIZE>::joint_coordinate_t joint_coordinate_t;

	typedef typename Base::scalar_t       		scalar_t;
	typedef typename Base::scalar_array_t 		scalar_array_t;
	typedef typename Base::state_vector_t 		state_vector_t;
	typedef typename Base::state_vector_array_t state_vector_array_t;
	typedef typename Base::state_matrix_t 		state_matrix_t;
	typedef typename Base::input_vector_t  		input_vector_t;
	typedef typename Base::input_vector_array_t input_vector_array_t;
	typedef typename Base::input_matrix_t 	    input_matrix_t;
	typedef typename Base::input_state_matrix_t input_state_matrix_t;

	typedef typename Base::cost_desired_trajectories_t 	cost_desired_trajectories_t;
	typedef typename Base::dynamic_vector_t 			dynamic_vector_t;
	typedef typename Base::dynamic_vector_array_t 		dynamic_vector_array_t;

	/**
	 * Construntor
	 */
	SwitchedModelCostBase(const kinematic_model_t& kinematicModel, const com_model_t& comModel,
			const state_matrix_t& Q,
			const input_matrix_t& R,
			const state_matrix_t& QFinal,
			const state_vector_t& xFinal,
			const scalar_t& copWeightMax = 0.0,
			const state_matrix_t& QIntermediate = state_matrix_t::Zero(),
			const state_vector_t& xNominalIntermediate = state_vector_t::Zero(),
			const scalar_t& sigma = 1.0,
			const scalar_t& tp = 0.0);

	/**
	 * copy construntor
	 */
	SwitchedModelCostBase(const SwitchedModelCostBase& rhs)

	: Base(rhs),
	  kinematicModelPtr_(rhs.kinematicModelPtr_->clone()),
	  comModelPtr_(rhs.comModelPtr_->clone()),
	  Q_desired_(rhs.Q_desired_),
	  R_desired_(rhs.R_desired_),
	  R_Bank_(rhs.R_Bank_),
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
	  copEstimatorPtr_(new CopEstimator<JOINT_COORD_SIZE>(*rhs.kinematicModelPtr_, *rhs.comModelPtr_))
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
	virtual void initializeModel(logic_rules_machine_t& logicRulesMachine,
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
     * Evaluates the cost.
     *
     * @param [out] L: The cost value.
     */
	virtual void getIntermediateCost(scalar_t& L) override;

    /**
     * Gets the state derivative.
     *
     * @param [out] dLdx: First order cost derivative with respect to state vector.
     */
	virtual void getIntermediateCostDerivativeState(state_vector_t& dLdx) override;

    /**
     * Gets state second order derivative.
     *
     * @param [out] dLdxx: Second order cost derivative with respect to state vector.
     */
	virtual void getIntermediateCostSecondDerivativeState(state_matrix_t& dLdxx) override;

    /**
     * Gets control input derivative.
     *
     * @param [out] dLdu: First order cost derivative with respect to input vector.
     */
	virtual void getIntermediateCostDerivativeInput(input_vector_t& dLdu) override;

    /**
     * Gets control input second derivative.
     *
     * @param [out] dLduu: Second order cost derivative with respect to input vector.
     */
	virtual void getIntermediateCostSecondDerivativeInput(input_matrix_t& dLduu) override;

    /**
     * Gets the state, control input derivative.
     *
     * @param [out] dLdux: Second order cost derivative with respect to state and input vector.
     */
	virtual void getIntermediateCostDerivativeInputState(input_state_matrix_t& dLdux) override;

    /**
     * Gets the terminal cost.
     *
     * @param [out] Phi: The final cost value
     */
	virtual void getTerminalCost(scalar_t& Phi) override;

    /**
     * Gets the terminal cost state derivative.
     *
     * @param [out] dPhidx: First order final cost derivative with respect to state vector.
     */
	virtual void getTerminalCostDerivativeState(state_vector_t& dPhidx) override;

    /**
     * Gets the terminal cost state second derivative
     *
     * @param [out] dPhidxx: Second order final cost derivative with respect to state vector.
     */
	virtual void getTerminalCostSecondDerivativeState(state_matrix_t& dPhidxx) override;

protected:
	/**
	 *
	 * @param stanceLeg
	 * @param R
	 * @return
	 */
	input_matrix_t correctedInputCost(const contact_flag_t& stanceLeg, const input_matrix_t& R);

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
	void copErrorCostFunc(const joint_coordinate_t& qJoints, const joint_coordinate_t& lambda,
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

	logic_rules_t* logicRulesPtr_;

	std::function<size_t(scalar_t)> findActiveSubsystemFnc_;

	contact_flag_t stanceLegs_;

	state_matrix_t Q_;
	state_matrix_t Q_desired_;
	input_matrix_t R_;
	input_matrix_t R_desired_;
	std::map<contact_flag_t, input_matrix_t, std::less<contact_flag_t>,
	  Eigen::aligned_allocator<std::pair<const contact_flag_t, input_matrix_t>> > R_Bank_;

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

	state_vector_t xDeviation_;
	input_vector_t uDeviation_;

	typename CopEstimator<JOINT_COORD_SIZE>::Ptr copEstimatorPtr_;

	Eigen::Matrix<scalar_t,12,1>  devJoints_copCost_;
	Eigen::Matrix<scalar_t,12,1>  devLambda_copCost_;
	Eigen::Matrix<scalar_t,12,12> hessJoints_copCost_;
	Eigen::Matrix<scalar_t,12,12> hessLambda_copCost_;
	Eigen::Matrix<scalar_t,12,12> devLambdaJoints_copCost_;

	std::string algorithmName_;
};

} //end of namespace switched_model

#include "implementation/SwitchedModelCostBase.h"

#endif /* SWITCHEDMODELCOSTBASE_H_ */
