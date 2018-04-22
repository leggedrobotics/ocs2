/*
 * GLQP.h
 *
 *  Created on: Jan 5, 2016
 *      Author: farbod
 */

#ifndef GLQP_OCS2_H_
#define GLQP_OCS2_H_

#include <iostream>
#include <memory>
#include <algorithm>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <vector>

#include <ocs2_core/Dimensions.h>
#include <ocs2_core/dynamics/ControlledSystemBase.h>
#include <ocs2_core/dynamics/DerivativesBase.h>
#include <ocs2_core/constraint/ConstraintBase.h>
#include <ocs2_core/cost/CostFunctionBase.h>
#include <ocs2_core/integration/SystemEventHandler.h>
#include <ocs2_core/integration/Integrator.h>
#include <ocs2_core/initialization/SystemOperatingTrajectoriesBase.h>
#include <ocs2_core/logic/rules/LogicRulesBase.h>
#include <ocs2_core/logic/machine/LogicRulesMachine.h>
#include <ocs2_core/misc/LinearInterpolation.h>

#include "ocs2_slq/SLQ_Settings.h"
#include "ocs2_slq/PartialRiccatiEquations.h"


namespace ocs2{

/**
 * This class implements the LQ method. The method is equivalent to SLQ. However, the nominal trajectories
 * are fixed and given in contrast to SLQ which derives them by forward integrating the system dynamics.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 * @tparam LOGIC_RULES_T: Logic Rules type (default NullLogicRules).
 */
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T=NullLogicRules<STATE_DIM,INPUT_DIM> >
class GLQP
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	static_assert(std::is_base_of<LogicRulesBase<STATE_DIM, INPUT_DIM>, LOGIC_RULES_T>::value,
			"LOGIC_RULES_T must inherit from LogicRulesBase");

	typedef std::shared_ptr<GLQP<STATE_DIM, INPUT_DIM, LOGIC_RULES_T> > Ptr;

	typedef PartialRiccatiEquations<STATE_DIM, INPUT_DIM> riccati_equations_t;
	//
	typedef Dimensions<STATE_DIM, INPUT_DIM> DIMENSIONS;
	typedef typename DIMENSIONS::controller_t 		controller_t;
	typedef typename DIMENSIONS::controller_array_t controller_array_t;
	typedef typename DIMENSIONS::scalar_t 		scalar_t;
	typedef typename DIMENSIONS::scalar_array_t scalar_array_t;
	typedef typename DIMENSIONS::size_array_t	size_array_t;
	typedef typename DIMENSIONS::eigen_scalar_t        eigen_scalar_t;
	typedef typename DIMENSIONS::eigen_scalar_array_t  eigen_scalar_array_t;
	typedef typename DIMENSIONS::eigen_scalar_array2_t eigen_scalar_array2_t;
	typedef typename DIMENSIONS::state_vector_t 	   state_vector_t;
	typedef typename DIMENSIONS::state_vector_array_t  state_vector_array_t;
	typedef typename DIMENSIONS::state_vector_array2_t state_vector_array2_t;
	typedef typename DIMENSIONS::control_vector_t 		 input_vector_t;
	typedef typename DIMENSIONS::control_vector_array_t  input_vector_array_t;
	typedef typename DIMENSIONS::control_vector_array2_t input_vector_array2_t;
	typedef typename DIMENSIONS::control_feedback_t 	   control_feedback_t;
	typedef typename DIMENSIONS::control_feedback_array_t  control_feedback_array_t;
	typedef typename DIMENSIONS::control_feedback_array2_t control_feedback_array2_t;
	typedef typename DIMENSIONS::state_matrix_t 	   state_matrix_t;
	typedef typename DIMENSIONS::state_matrix_array_t  state_matrix_array_t;
	typedef typename DIMENSIONS::state_matrix_array2_t state_matrix_array2_t;
	typedef typename DIMENSIONS::control_matrix_t 		 control_matrix_t;
	typedef typename DIMENSIONS::control_matrix_array_t  control_matrix_array_t;
	typedef typename DIMENSIONS::control_matrix_array2_t control_matrix_array2_t;
	typedef typename DIMENSIONS::control_gain_matrix_t 		  control_gain_matrix_t;
	typedef typename DIMENSIONS::control_gain_matrix_array_t  control_gain_matrix_array_t;
	typedef typename DIMENSIONS::control_gain_matrix_array2_t control_gain_matrix_array2_t;

	typedef LogicRulesMachine<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>		logic_rules_machine_t;
	typedef ControlledSystemBase<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>	controlled_system_base_t;
	typedef SystemEventHandler<STATE_DIM>								event_handler_t;
	typedef DerivativesBase<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>		derivatives_base_t;
	typedef ConstraintBase<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>			constraint_base_t;
	typedef CostFunctionBase<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>		cost_function_base_t;
	typedef SystemOperatingTrajectoriesBase<STATE_DIM, INPUT_DIM, LOGIC_RULES_T> operating_trajectories_base_t;

	/**
	 * Constructor
	 *
	 * @param [in] systemDynamicsPtr: A pointer to the system's dynamics.
	 * @param [in] systemDerivativesPtr: A pointer to the derivatives of the system's dynamics.
	 * @param [in] costFunctionPtr: A pointer to the cost function and its derivatives.
	 */
	GLQP(const controlled_system_base_t* systemDynamicsPtr,
		 const derivatives_base_t* systemDerivativesPtr,
		 const cost_function_base_t* costFunctionPtr,
		 const operating_trajectories_base_t* operatingTrajectoriesPtr,
		 const SLQ_Settings& settings = SLQ_Settings(),
		 const LOGIC_RULES_T& logicRules = LOGIC_RULES_T(),
		 const cost_function_base_t* heuristicsFunctionPtr = nullptr);

	/**
	 * Default destructor.
	 */
	~GLQP() {}

	/**
	 * Finds the operating points in the given time period (initTime, finalTime]
	 *
	 * @param [in] initTime: The initial time.
	 * @param [in] initState: The initial state.
	 * @param [in] finalTime: The final time.
	 * @param [in] partitioningTimes: Time partitioning.
	 * @param [out] stateOperatingPointsStock: An array of state operating points.
	 * @param [out] inputOperatingPointsStock: An array of input operating points.
	 * @param [in] threadId: Working thread (default is 0).
	 */
	void findOperatingPoints(
			const scalar_t& initTime,
			const state_vector_t& initState,
			const scalar_t& finalTime,
			const scalar_array_t& partitioningTimes,
			std::vector<scalar_array_t>& timeOperatingPointsStock,
			std::vector<size_array_t>& eventsPastTheEndIndecesStock,
			state_vector_array2_t& stateOperatingPointsStock,
			input_vector_array2_t& inputOperatingPointsStock,
			size_t threadId = 0);

//	/**
//	 * Forward integrate the system dynamics with given controller. It uses the given control policies and initial state,
//	 * to integrate the system dynamics in time period [initTime, finalTime].
//	 *
//	 * @param [in] initState: Initial state.
//	 * @param [in] controllersStock: Array of control policies.
//	 * @param [out] timeTrajectoriesStock: Array of trajectories containing the output time trajectory stamp.
//	 * @param [out] stateTrajectoriesStock: Array of trajectories containing the output state trajectory.
//	 * @param [out] inputTrajectoriesStock: Array of trajectories containing the output control input trajectory.
//	 */
//	void rollout(const state_vector_t& initState,
//			const controller_array_t& controllersStock,
//			std::vector<scalar_array_t>& timeTrajectoriesStock,
//			state_vector_array2_t& stateTrajectoriesStock,
//			input_vector_array2_t& inputTrajectoriesStock);
//
//	/**
//	 * Calculates cost of a rollout.
//	 *
//	 * @param [in] timeTrajectoriesStock: Array of trajectories containing the time trajectory stamp of a roll-out.
//	 * @param [in] stateTrajectoriesStock: Array of trajectories containing the state trajectory of a roll-out.
//	 * @param [in] controlTrajectoriesStock: Array of trajectories containing the control input trajectory of a roll-out.
//	 * @param [out] totalCost: The total cost of the roll-out.
//	 */
//	void rolloutCost(const std::vector<scalar_array_t>& timeTrajectoriesStock,
//			const state_vector_array2_t& stateTrajectoriesStock,
//			const input_vector_array2_t& controlTrajectoriesStock,
//			scalar_t& totalCost);

	/**
	 * Approximates optimal control problem
	 */
	void approximateOptimalControlProblem();

	/**
	 * Solves Riccati equations for all the partitions.
	 *
	 * @param [in] SmFinal: The final Sm for Riccati equation.
	 * @param [in] SvFinal: The final Sv for Riccati equation.
	 * @param [in] sFinal: The final s for Riccati equation.
	 */
	void solveRiccatiEquations(
			const state_matrix_t& SmFinal,
			const state_vector_t& SvFinal,
			const eigen_scalar_t& sFinal);

//	/**
//	 * Calculates the controller.
//	 *
//	 * @param [in] learningRate: Learning rate.
//	 * @param [out] controllersStock: the output array of control policies.
//	 */
//	void calculateController(const scalar_t& learningRate, controller_array_t& controllersStock);


	/**
	 * Gets the optimal array of the control policies.
	 *
	 * @param [out] controllersStock: The optimal array of the control policies.
	 */
	void getController(controller_array_t& controllersStock);

//	/**
//	 * Calculates the value function at the given time and state.
//	 *
//	 * @param [in] time: The inquiry time
//	 * @param [in] state: The inquiry state.
//	 * @param [out] valueFuntion: value function at the inquiry time and state.
//	 */
//	void getValueFuntion(const scalar_t& time, const state_vector_t& state, scalar_t& valueFuntion);

	/**
	 * The main routine of LQ which runs GLQP for a given state, time, and partitioning times.
	 *
	 * @param [in] initTime: The initial time.
	 * @param [in] initState: The initial state.
	 * @param [in] finalTime: The final time.
	 * @param [in] partitioningTimes: The time partitioning.
	 * @param [in] learningRate: Learning rate for the line search.
	 * @param [in] desiredTimeTrajectoriesStock: The time stamp trajectory for each subsystem's cost.
	 * @param [in] desiredStateTrajectoriesStock: The desired state trajectory for each partition's cost.
	 * @param [in] desiredInputTrajectoriesStock: The desired input trajectory for each partition's cost.
	 */
	void run(const scalar_t& initTime,
			const state_vector_t& initState,
			const scalar_t& finalTime,
			const scalar_array_t& partitioningTimes,
			const scalar_t& learningRate = 1.0,
			const std::vector<scalar_array_t>& desiredTimeTrajectoriesStock = std::vector<scalar_array_t>(),
			const state_vector_array2_t& desiredStateTrajectoriesStock = state_vector_array2_t(),
			const input_vector_array2_t& desiredInputTrajectoriesStock = input_vector_array2_t());


protected:
	/**
	 * Computes the linearized dynamics for a particular time partition
	 *
	 * @param [in] partitionIndex: Time partition index
	 */
	void approximatePartitionLQ(const size_t& partitionIndex);

	/**
	 * Calculates an LQ approximate of the optimal control problem at a given partition and a node.
	 *
	 * @param [in] workerIndex: Working agent index.
	 * @param [in] partitionIndex: Time partition index
	 * @param [in] timeIndex: Time index in the partition
	 */
	void approximateLQWorker(size_t workerIndex,
			const size_t& partitionIndex,
			const size_t& timeIndex);

	/**
	 * Sets up optimizer for different number of partitionings.
	 *
	 * @param [in] numPartitionings: number of partitionings
	 */
	void setupOptimizer(const size_t& numPartitionings);

	/**
	 * Finds the operating points in the given partition and time period (initTime, finalTime]
	 *
	 * @param [in] workerIndex: Working agent index.
	 * @param [in] partitionIndex: Time partition index.
	 * @param [in] initTime: The initial time.
	 * @param [in] initState: The initial state.
	 * @param [in] finalTime: The final time.
	 * @param [in] eventTimes: An array of event times in "partitionIndex" time partition.
	 * @param stateOperatingPoints: State operating points
	 * @param inputOperatingPoints: Input operating points
	 */
	void findOperatingPointsWorker(
			size_t workerIndex,
			const size_t& partitionIndex,
			const scalar_t& initTime,
			const state_vector_t& initState,
			const scalar_t& finalTime,
			const scalar_array_t& eventTimes,
			scalar_array_t& timeOperatingPointsStock,
			size_array_t& eventsPastTheEndIndeces,
			state_vector_array_t& stateOperatingPoints,
			input_vector_array_t& inputOperatingPoints);

	/**
	 * Solves a set of Riccati equations for the partition in the given index.
	 *
	 * @param [in] workerIndex: Working agent index.
	 * @param [in] partitionIndex: The requested partition index to solve Riccati equations.
	 * @param [in] SmFinal: The final Sm for Riccati equation.
	 * @param [in] SvFinal: The final Sv for Riccati equation.
	 * @param [in] sFinal: The final s for Riccati equation.
	 */
	void solveRiccatiEquationsWorker(
			size_t workerIndex,
			const size_t& partitionIndex,
			const state_matrix_t& SmFinal,
			const state_vector_t& SvFinal,
			const eigen_scalar_t& sFinal);

//	/**
//	 * Transforms local value function to global
//	 */
//	void transformeLocalValueFuntion2Global();

	/**
	 * Finds the interval of partitioningTimes to which the input time belongs to it.
	 * time is in interval i if: partitioningTimes[i] < t <= partitioningTimes[i+1]
	 * Exception: if time=partitioningTimes[0] then time is interval 0
	 *
	 * @param [in] partitioningTimes: a sorted time sequence.
	 * @param [in] time: Enquiry time.
	 * @return Active subsystem index.
	 */
	static size_t findActiveSubsystemIndex(const scalar_array_t& partitioningTimes, const double& time);

	/**
	 * Makes the matrix PSD.
	 * @tparam Derived type.
	 * @param [out] squareMatrix: The matrix to become PSD.
	 * @return boolean
	 */
	template <typename Derived>
	bool makePSD(Eigen::MatrixBase<Derived>& squareMatrix);


	/****************
	 *** Variables **
	 ****************/
	SLQ_Settings settings_;
	logic_rules_machine_t logicRulesMachine_;

	scalar_t initTime_;
	scalar_t finalTime_;
	state_vector_t initState_;

	size_t initActivePartition_;
	size_t finalActivePartition_;

	size_t numPartitionings_;
	scalar_array_t partitioningTimes_;

	std::vector<std::shared_ptr<const scalar_array_t>> 		 desiredTimeTrajectoryPtrStock_;
	std::vector<std::shared_ptr<const state_vector_array_t>> desiredStateTrajectoryPtrStock_;
	std::vector<std::shared_ptr<const input_vector_array_t>> desiredInputTrajectoryPtrStock_;

	std::vector<typename controlled_system_base_t::Ptr> systemDynamicsPtrStock_;
	std::vector<typename derivatives_base_t::Ptr> 		systemDerivativesPtrStock_;
	std::vector<typename cost_function_base_t::Ptr> 	costFunctionsPtrStock_;
	std::vector<typename cost_function_base_t::Ptr> 	heuristicsFunctionsPtrStock_;
	std::vector<typename event_handler_t::Ptr>			systemEventHandlersPtrStock_;
	std::vector<std::shared_ptr<ODE45<STATE_DIM>>> 		dynamicsIntegratorsStockPtr_;
	std::vector<typename operating_trajectories_base_t::Ptr> operatingTrajectoriesPtrStock_;

	std::vector<scalar_array_t> timeOperatingPointsStock_;
	std::vector<size_array_t> eventsPastTheEndIndecesStock_;
	state_vector_array2_t stateOperatingPointsStock_;
	input_vector_array2_t inputOperatingPointsStock_;

	controller_array_t controllersStock_;

	state_matrix_array2_t        AmStock_;
	control_gain_matrix_array2_t BmStock_;

	eigen_scalar_array2_t qFinalStock_;
	state_vector_array2_t QvFinalStock_;
	state_matrix_array2_t QmFinalStock_;

	eigen_scalar_array2_t qStock_;
	state_vector_array2_t QvStock_;
	state_matrix_array2_t QmStock_;
	input_vector_array2_t RvStock_;
	control_feedback_array2_t PmStock_;
	control_matrix_array2_t RmStock_;
	control_matrix_array2_t RmInverseStock_;

	std::vector<std::shared_ptr<riccati_equations_t>> 							riccatiEquationsPtrStock_;
	std::vector<std::shared_ptr<IntegratorBase<riccati_equations_t::S_DIM_>>> 	riccatiIntegratorPtrStock_;

	std::vector<scalar_array_t> SsTimeTrajectoryStock_;
	std::vector<scalar_array_t> SsNormalizedTimeTrajectoryStock_;
	std::vector<size_array_t> 	SsNormalizedEventsPastTheEndIndecesStock_;
	eigen_scalar_array2_t  		sTrajectoryStock_;
	state_vector_array2_t 		SvTrajectoryStock_;
	state_matrix_array2_t  		SmTrajectoryStock_;

	eigen_scalar_array_t sFinalStock_;
	state_vector_array_t SvFinalStock_;
	state_matrix_array_t SmFinalStock_;

	eigen_scalar_t sHeuristics_;
	state_vector_t SvHeuristics_;
	state_matrix_t SmHeuristics_;
};

} // namespace ocs2

#include "implementation/GLQP.h"

#endif /* GLQP_H_ */
