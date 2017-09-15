/*
 * SLQP_BASE.h
 *
 *  Created on: August 14, 2016
 *      Author: farbodf
 */

#ifndef SLQP_BASE_OCS2_H_
#define SLQP_BASE_OCS2_H_

#include <vector>
#include <array>
#include <algorithm>
#include <numeric>
#include <cstddef>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <unsupported/Eigen/MatrixFunctions>

#include "Dimensions.h"

#include <ocs2_core/dynamics/ControlledSystemBase.h>
#include <ocs2_core/dynamics/DerivativesBase.h>
#include <ocs2_core/cost/CostFunctionBaseOCS2.h>
#include <ocs2_core/integration/Integrator.h>
#include <ocs2_core/misc/LinearInterpolation.h>
#include <ocs2_core/misc/LTI_Equations.h>

#include <ocs2_slq/SequentialRiccatiEquations.h>
#include <ocs2_slq/SequentialRiccatiEquationsNormalized.h>
#include <ocs2_slq/SequentialErrorEquation.h>
#include <ocs2_slq/SequentialErrorEquationNormalized.h>

#include <chrono>

//#define BENCHMARK

namespace ocs2{

/**
 * This class is an interface class for the single-thread and multi-thread SLQ.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class SLQP_BASE
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef std::shared_ptr<SLQP_BASE<STATE_DIM, INPUT_DIM>> 			Ptr;
	//	typedef SequentialRiccatiEquations<STATE_DIM, INPUT_DIM> 			RiccatiEquations_t;
	typedef SequentialRiccatiEquationsNormalized<STATE_DIM, INPUT_DIM>	RiccatiEquations_t;
	//	typedef SequentialErrorEquation<STATE_DIM, INPUT_DIM>			 	ErrorEquation_t;
	typedef SequentialErrorEquationNormalized<STATE_DIM, INPUT_DIM> 	ErrorEquation_t;
	//	typedef LTI_Equations<STATE_DIM> LTI_Equation_t;

	typedef Dimensions<STATE_DIM, INPUT_DIM> DIMENSIONS;

	typedef typename DIMENSIONS::template LinearFunction_t<Eigen::Dynamic> lagrange_t;

	typedef typename DIMENSIONS::controller_t controller_t;
	typedef typename DIMENSIONS::controller_array_t controller_array_t;
	typedef typename DIMENSIONS::Options Options_t;
	typedef typename DIMENSIONS::scalar_t scalar_t;
	typedef typename DIMENSIONS::scalar_array_t scalar_array_t;
	typedef typename DIMENSIONS::eigen_scalar_t eigen_scalar_t;
	typedef typename DIMENSIONS::eigen_scalar_array_t eigen_scalar_array_t;
	typedef typename DIMENSIONS::eigen_scalar_array2_t eigen_scalar_array2_t;
	typedef typename DIMENSIONS::state_vector_t state_vector_t;
	typedef typename DIMENSIONS::state_vector_array_t state_vector_array_t;
	typedef typename DIMENSIONS::state_vector_array2_t state_vector_array2_t;
	typedef typename DIMENSIONS::control_vector_t control_vector_t;
	typedef typename DIMENSIONS::control_vector_array_t control_vector_array_t;
	typedef typename DIMENSIONS::control_vector_array2_t control_vector_array2_t;
	typedef typename DIMENSIONS::control_feedback_t control_feedback_t;
	typedef typename DIMENSIONS::control_feedback_array_t control_feedback_array_t;
	typedef typename DIMENSIONS::control_feedback_array2_t control_feedback_array2_t;
	typedef typename DIMENSIONS::state_matrix_t state_matrix_t;
	typedef typename DIMENSIONS::state_matrix_array_t state_matrix_array_t;
	typedef typename DIMENSIONS::state_matrix_array2_t state_matrix_array2_t;
	typedef typename DIMENSIONS::control_matrix_t control_matrix_t;
	typedef typename DIMENSIONS::control_matrix_array_t control_matrix_array_t;
	typedef typename DIMENSIONS::control_matrix_array2_t control_matrix_array2_t;
	typedef typename DIMENSIONS::control_gain_matrix_t control_gain_matrix_t;
	typedef typename DIMENSIONS::control_gain_matrix_array_t control_gain_matrix_array_t;
	typedef typename DIMENSIONS::control_gain_matrix_array2_t control_gain_matrix_array2_t;
	typedef typename DIMENSIONS::constraint1_vector_t constraint1_vector_t;
	typedef typename DIMENSIONS::constraint1_vector_array_t constraint1_vector_array_t;
	typedef typename DIMENSIONS::constraint1_vector_array2_t constraint1_vector_array2_t;
	typedef typename DIMENSIONS::constraint1_state_matrix_t constraint1_state_matrix_t;
	typedef typename DIMENSIONS::constraint1_state_matrix_array_t constraint1_state_matrix_array_t;
	typedef typename DIMENSIONS::constraint1_state_matrix_array2_t constraint1_state_matrix_array2_t;
	typedef typename DIMENSIONS::constraint1_control_matrix_t constraint1_control_matrix_t;
	typedef typename DIMENSIONS::constraint1_control_matrix_array_t constraint1_control_matrix_array_t;
	typedef typename DIMENSIONS::constraint1_control_matrix_array2_t constraint1_control_matrix_array2_t;
	typedef typename DIMENSIONS::control_constraint1_matrix_t control_constraint1_matrix_t;
	typedef typename DIMENSIONS::control_constraint1_matrix_array_t control_constraint1_matrix_array_t;
	typedef typename DIMENSIONS::control_constraint1_matrix_array2_t control_constraint1_matrix_array2_t;
	typedef typename DIMENSIONS::constraint2_vector_t       constraint2_vector_t;
	typedef typename DIMENSIONS::constraint2_vector_array_t constraint2_vector_array_t;
	typedef typename DIMENSIONS::constraint2_vector_array2_t constraint2_vector_array2_t;
	typedef typename DIMENSIONS::constraint2_state_matrix_t       constraint2_state_matrix_t;
	typedef typename DIMENSIONS::constraint2_state_matrix_array_t constraint2_state_matrix_array_t;
	typedef typename DIMENSIONS::constraint2_state_matrix_array2_t constraint2_state_matrix_array2_t;

	SLQP_BASE()
	: numSubsystems_(0)
	{}

	/**
	 * Constructor
	 *
	 * @param [in] subsystemDynamicsPtr: Array of system dynamics and constraints for system's subsystems.
	 * @param [in] subsystemDerivativesPtr: Array of system dynamics and constraints dervatives for system's subsystems.
	 * @param [in] subsystemCostFunctionsPtr: Array of cost function and its dervatives for system's subsystems.
	 * @param [in] options: Structure containing the settings for the SLQ algorithm.
	 * @param [in] stateOperatingPoints: The state operating points for system's subsystems which will be used for initialization of SLQ.
	 * @param [in] inputOperatingPoints: The input operating points for system's subsystems which will be used for initialization of SLQ.
	 */
	SLQP_BASE(const std::vector<std::shared_ptr<ControlledSystemBase<STATE_DIM, INPUT_DIM> > >& subsystemDynamicsPtr,
			const std::vector<std::shared_ptr<DerivativesBase<STATE_DIM, INPUT_DIM> > >& subsystemDerivativesPtr,
			const std::vector<std::shared_ptr<CostFunctionBaseOCS2<STATE_DIM, INPUT_DIM> > >& subsystemCostFunctionsPtr,
			const Options_t& options = Options_t::Options(),
			const state_vector_array_t& stateOperatingPoints = state_vector_array_t(),
			const control_vector_array_t& inputOperatingPoints = control_vector_array_t() )
	: numSubsystems_(0),
	  subsystemDynamicsPtr_(subsystemDynamicsPtr),
	  subsystemDerivativesPtr_(subsystemDerivativesPtr),
	  subsystemCostFunctionsPtr_(subsystemCostFunctionsPtr),
	  options_(options),
	  stateOperatingPoints_(stateOperatingPoints),
	  inputOperatingPoints_(inputOperatingPoints)
	{
		if (subsystemDynamicsPtr.size() != subsystemDerivativesPtr.size())
			throw std::runtime_error("Number of subsystem derivatives is not equal to the number of subsystems.");
		if (subsystemDynamicsPtr.size() != subsystemCostFunctionsPtr.size())
			throw std::runtime_error("Number of cost functions is not equal to the number of subsystems.");
		if (subsystemDynamicsPtr.size() != stateOperatingPoints.size())
			throw std::runtime_error("Number of state operating points is not equal to the number of subsystems.");
		if (subsystemDynamicsPtr.size() != inputOperatingPoints.size())
			throw std::runtime_error("Number of input operating points is not equal to the number of subsystems.");

		// LQP
		typedef GLQP<STATE_DIM, INPUT_DIM> lqp_t;
		typename lqp_t::Ptr lqpPtr_( new lqp_t(subsystemDynamicsPtr, subsystemDerivativesPtr, subsystemCostFunctionsPtr,
				stateOperatingPoints, inputOperatingPoints) );
		const size_t N = subsystemDynamicsPtr.size();
		std::vector<size_t>   systemStockIndexes(1);
		std::vector<scalar_t> switchingTimes(2);
		lqpControllersStock_.resize(N);
		for (size_t i=0; i<N; i++) {

			systemStockIndexes[0] = i;
			switchingTimes[0] = 0.0;
			switchingTimes[1] = 1.0;

			lqpPtr_->run(systemStockIndexes, switchingTimes, 0.0/*=1.0*/);
			controller_array_t local_controller(1);
			lqpPtr_->getController(local_controller);
			lqpControllersStock_[i].swap(local_controller[0]);
		} // end of i loop

	}

	/**
	 * Default destructor.
	 */
	virtual ~SLQP_BASE() {}

	/**
	 * The interface class for performaing rollout. It uses the given control policies and initial state,
	 * to integrate the system dyanmics and calculate the costraints in time period [initTime, finalTime].
	 *
	 * @param [in] initTime: Initial time.
	 * @param [in] initState: Initial state.
	 * @param [in] finalTime: Final time.
	 * @param [in] controllersStock: Array of control policies.
	 * @param [out] timeTrajectoriesStock: Array of trajectories containing the output time trajectory stamp.
	 * @param [out] stateTrajectoriesStock: Array of trajectories containing the output state trajectory.
	 * @param [out] inputTrajectoriesStock: Array of trajectories containing the output control input trajectory.
	 * @param [out] nc1TrajectoriesStock: Array of trajectories containing the number of the active state-input constraints.
	 * @param [out] EvTrajectoryStock: Array of trajectories containing the value of the state-input constraints (if the rollout is constrained the value is
	 * always zero otherwise it is nonzero).
	 * @param [out] nc2TrajectoriesStock: Array of trajectories containing the number of the active state-only constraints.
	 * @param [out] HvTrajectoryStock: Array of trajectories containing the value of the state-only constraints.
	 * @param [out] nc2FinalStock: Array containing the number of the active final state-only constraints.
	 * @param [out] HvFinalStock: Array containing the value of the final state-only constraints.
	 */
	virtual void rollout(const double& initTime,
			const state_vector_t& initState,
			const scalar_t& finalTime,
			const controller_array_t& controllersStock,
			std::vector<scalar_array_t>& timeTrajectoriesStock,
			state_vector_array2_t& stateTrajectoriesStock,
			control_vector_array2_t& inputTrajectoriesStock,
			std::vector<std::vector<size_t> >& nc1TrajectoriesStock,
			constraint1_vector_array2_t& EvTrajectoryStock,
			std::vector<std::vector<size_t> >& nc2TrajectoriesStock,
			constraint2_vector_array2_t& HvTrajectoryStock,
			std::vector<size_t>& nc2FinalStock,
			constraint2_vector_array_t& HvFinalStock) = 0;

	/**
	 * The interface class for performaing rollout. It uses the given control policies and initial state,
	 * to integrate the system dyanmics in time period [initTime, finalTime].
	 *
	 * @param [in] initTime: Initial time.
	 * @param [in] initState: Initial state.
	 * @param [in] finalTime: Final time.
	 * @param [in] controllersStock: Array of control policies.
	 * @param [out] timeTrajectoriesStock: Array of trajectories containing the output time trajectory stamp.
	 * @param [out] stateTrajectoriesStock: Array of trajectories containing the output state trajectory.
	 * @param [out] inputTrajectoriesStock: Array of trajectories containing the output control input trajectory.
	 */
	virtual void rollout(const double& initTime,
			const state_vector_t& initState,
			const scalar_t& finalTime,
			const controller_array_t& controllersStock,
			std::vector<scalar_array_t>& timeTrajectoriesStock,
			state_vector_array2_t& stateTrajectoriesStock,
			control_vector_array2_t& inputTrajectoriesStock) = 0;

	/**
	 * The interface class for performaing rollout. It uses the given control policies and initial state,
	 * to integrate the system dyanmics in time period [initTime, finalTime] and only return the final state.
	 *
	 * @param [in] initTime: Initial time.
	 * @param [in] initState: Initial state.
	 * @param [in] finalTime: Final time.
	 * @param [in] controllersStock: Array of control policies.
	 * @param [out] finalState: Final state.
	 * @param [out] finalInput: Final control input.
	 * @param [out] finalActiveSubsystemIndex: The final active subsystem.
	 */
	virtual void rollout(const scalar_t& initTime,
			const state_vector_t& initState,
			const scalar_t& finalTime,
			const controller_array_t& controllersStock,
			state_vector_t& finalState,
			control_vector_t& finalInput,
			size_t& finalActiveSubsystemIndex) = 0;

	/**
	 * Calculates cost of a rollout.
	 *
	 * @param [in] timeTrajectoriesStock: Array of trajectories containing the time trajectory stamp of a rollout.
	 * @param [in] stateTrajectoriesStock: Array of trajectories containing the state trajectory of a rollout.
	 * @param [in] inputTrajectoriesStock: Array of trajectories containing the control input trajectory of a rollout.
	 * @param [out] totalCost: The total cost of the rollout.
	 */
	virtual void calculateCostFunction(const std::vector<scalar_array_t>& timeTrajectoriesStock,
			const state_vector_array2_t& stateTrajectoriesStock,
			const control_vector_array2_t& inputTrajectoriesStock,
			scalar_t& totalCost) = 0;

	/**
	 * Calculates the cost function plus penalty for state-only constraints of a rollout.
	 *
	 * @param [in] timeTrajectoriesStock: Array of trajectories containing the time trajectory stamp of a rollout.
	 * @param [in] stateTrajectoriesStock: Array of trajectories containing the state trajectory of a rollout.
	 * @param [in] inputTrajectoriesStock: Array of trajectories containing the control input trajectory of a rollout.
	 * @param [out] nc2TrajectoriesStock: Array of trajectories containing the number of the active state-only constraints.
	 * @param [out] HvTrajectoryStock: Array of trajectories containing the value of the state-only constraints.
	 * @param [out] nc2FinalStock: Array containing the number of the active final state-only constraints.
	 * @param [out] HvFinalStock: Array containing the value of the final state-only constraints.
	 * @param [out] totalCost: The total cost plus state-only constraints penalty.
	 */
	virtual void calculateCostFunction(const std::vector<scalar_array_t>& timeTrajectoriesStock,
			const state_vector_array2_t& stateTrajectoriesStock,
			const control_vector_array2_t& inputTrajectoriesStock,
			const std::vector<std::vector<size_t> >& nc2TrajectoriesStock,
			const constraint2_vector_array2_t& HvTrajectoryStock,
			const std::vector<size_t>& nc2FinalStock,
			const constraint2_vector_array_t& HvFinalStock,
			scalar_t& totalCost) = 0;

	/**
	 * Gets the optimal array of the control policies.
	 *
	 * @param [out] controllersStock: The optimal array of the control policies.
	 */
	void getController(controller_array_t& controllersStock) const;

	/**
	 * Gets a pointer to the optimal array of the control policies.
	 *
	 * @param [out] controllersStock: A pointer to the optimal array of the control policies
	 */
	void getControllerPtr(std::shared_ptr<controller_array_t>& controllersStock) const;

	/**
	 * Gets a specific optimal control policy with the given index.
	 *
	 * @param [in] index: Index of the optimal control policy.
	 * @return The optimal control policy with the given index.
	 */
	const controller_t& controller(size_t index) const;

	/**
	 * Sets up uptomizer
	 */
	virtual void setupOptimizer() = 0;

	/**
	 * Runs the initialization method for SLQ.
	 */
	virtual void runInit() = 0;

	/**
	 * Runs a single iteration of SLQ.
	 *
	 * @param [in] SmFinal: The final Sm for Riccati equation.
	 * @param [in] SvFinal: The final Sv for Riccati equation.
	 * @param [in] sFinal: The final s for Riccati equation.
	 */
	virtual void runIteration(const state_matrix_t& SmFinal  = state_matrix_t::Zero(),
			const state_vector_t& SvFinal = state_vector_t::Zero(),
			const eigen_scalar_t& sFinal   = eigen_scalar_t::Zero()) = 0;

	/**
	 * Runs the exit method SLQ.
	 *
	 * @param [in] SmFinal: The final Sm for Riccati equation.
	 * @param [in] SvFinal: The final Sv for Riccati equation.
	 * @param [in] sFinal: The final s for Riccati equation.
	 */
	virtual void runExit(const state_matrix_t& SmFinal = state_matrix_t::Zero(),
			const state_vector_t& SvFinal = state_vector_t::Zero(),
			const eigen_scalar_t& sFinal = eigen_scalar_t::Zero())  = 0;

	/**
	 * The main routine of SLQ which runs the SLQP algorithm for a given state, time, subsystem arrangement, and switching times.
	 *
	 * @param [in] initTime: Initial time.
	 * @param [in] initState: Initial state.
	 * @param [in] finalTime: Final time.
	 * @param [in] systemStockIndexes: The indexes of the susbsystms in subsystemDynamicsPtr, subsystemDerivativesPtr, or subsystemCostFunctionsPtr.
	 * @param [in] switchingTimes: The switching times between subsystems.
	 * @param [in] controllersStock: Array of the initial control policies. If it is provided as a empty array, SLQ calculats internally the initial policies.
	 * @param [in] desiredTimeTrajectoriesStock
	 * @param [in] desiredStateTrajectoriesStock
	 */
	void run(const double& initTime, const state_vector_t& initState, const double& finalTime,
			const std::vector<size_t>& systemStockIndexes=std::vector<size_t>(),
			const std::vector<scalar_t>& switchingTimes=std::vector<scalar_t>(),
			const controller_array_t& controllersStock=controller_array_t(),
			const std::vector<scalar_array_t>& desiredTimeTrajectoriesStock = std::vector<scalar_array_t>(),
			const state_vector_array2_t& desiredStateTrajectoriesStock = state_vector_array2_t());

	/**
	 * Gets a pointer to the array of the subsystem dynamics.
	 * @return Pointer to the array of the subsystem dynamics.
	 */
	virtual std::vector<std::shared_ptr<ControlledSystemBase<STATE_DIM, INPUT_DIM>> >& getSubsystemDynamicsPtrStock() = 0;

	/**
	 * Calculates the value function at the given time and state.
	 *
	 * @param [in] time: The inquiry time
	 * @param [in] state: The inquiry state.
	 * @param [out] valueFuntion: value function at the inquiry time and state.
	 */
	virtual void getValueFuntion(const scalar_t& time, const state_vector_t& state, scalar_t& valueFuntion);

	/**
	 * Gets the cost function and ISE of the constriants at the initial time.
	 *
	 * @param [out] costFunction: cost function value
	 * @param [out] constraintISE: constraint ISE
	 */
	virtual void getCostFuntion(scalar_t& costFunction, scalar_t& constraintISE);

	/**
	 * Gets the nominal trajectories
	 *
	 * @param [out] nominalTimeTrajectoriesStock: Array of trajectories containing the output time trajectory stamp.
	 * @param [out] nominalStateTrajectoriesStock: Array of trajectories containing the output state trajectory.
	 * @param [out] nominalInputTrajectoriesStock: Array of trajectories containing the output control input trajectory.
	 */
	virtual void getNominalTrajectories(std::vector<scalar_array_t>& nominalTimeTrajectoriesStock,
			state_vector_array2_t& nominalStateTrajectoriesStock,
			control_vector_array2_t& nominalInputTrajectoriesStock);

	/**
	 * Gets a pointer to the nominal trajectories.
	 *
	 * @param [out] nominalTimeTrajectoriesStockPtr: A pointer to an array of trajectories containing the output time trajectory stamp.
	 * @param [out] nominalStateTrajectoriesStockPtr: A pointer to an array of trajectories containing the output state trajectory.
	 * @param [out] nominalInputTrajectoriesStockPtr: A pointer to an array of trajectories containing the output control input trajectory.
	 */
	virtual void getNominalTrajectoriesPtr(std::shared_ptr<std::vector<scalar_array_t>>& nominalTimeTrajectoriesStockPtr,
			std::shared_ptr<state_vector_array2_t>& nominalStateTrajectoriesStockPtr,
			std::shared_ptr<control_vector_array2_t>& nominalInputTrajectoriesStockPtr);

	/**
	 * Gets a reference to the Options structure.
	 *
	 * @return a reference to the Options structure.
	 */
	Options_t& options() {return options_;}

	/**
	 * Gets Iterations Log of SLQ
	 * @param [out] iterationCost: Each iteration cost.
	 * @param [out] iterationISE1: Each iteration constraints ISE.
	 */
	void getIterationsLog(eigen_scalar_array_t& iterationCost, eigen_scalar_array_t& iterationISE1) const {
		iterationCost = iterationCost_;
		iterationISE1 = iterationISE1_;
	}

	/**
	 * Uses Disjoint Riccati approach which effective makes the backward pass of the SLQ parallelizable.
	 *
	 * @param [in] useDisjointRiccati: Set to true if Disjoint Riccati approach should be used.
	 */
	void setUseDisjointRiccati(bool useDisjointRiccati) {useDisjointRiccati_= useDisjointRiccati;}

	/**
	 * Calculates state-input constraints ISE (Integral of Square Error). It also return the maximum norm of the constraints.
	 *
	 * @param [in] timeTrajectoriesStock: Array of trajectories containing the time trajectory stamp.
	 * @param [in] nc1TrajectoriesStock: Array of trajectories containing the number of the active state-input constraints.
	 * @param [in] EvTrajectoriesStock: Array of trajectories containing the value of the state-input constraints.
	 * @param [out] constraintISE: The state-input constraints ISE.
	 * @return maximum norm of the constraints.
	 */
	double calculateConstraintISE(const std::vector<scalar_array_t>& timeTrajectoriesStock,
			const std::vector<std::vector<size_t>>& nc1TrajectoriesStock,
			const constraint1_vector_array2_t& EvTrajectoriesStock,
			scalar_t& constraintISE);

	/**
	 * Finds active subsystem index.
	 *
	 * @param [in] switchingTimes: Switching times.
	 * @param [in] time: Time.
	 * @return Active subsystem index.
	 */
	static size_t findActiveSubsystemIndex(const scalar_array_t& switchingTimes, const double& time);

	/**
	 * Truncates the internal array of the control policies based on the initTime.
	 *
	 * @param [in] switchingTimes: Switching times.
	 * @param [in] initTime: Initial time.
	 * @param [out] controllersStock: Truncated array of the control policies.
	 * @param [out] initActiveSubsystemIndex: Initial active susbsystems.
	 * @param [out] deletedcontrollersStock: The deleted part of the control policies.
	 */
	void truncateConterller(const scalar_array_t& switchingTimes,
			const double& initTime,
			controller_array_t& controllersStock,
			size_t& initActiveSubsystemIndex,
			controller_array_t& deletedcontrollersStock);

	/**
	 * Sets switching times.
	 *
	 * @param [in] switchingTimes: Switching times.
	 */
	void setSwitchingTimes(const scalar_array_t& switchingTimes) {switchingTimes_ = switchingTimes;}

	/**
	 * Gets switching times
	 *
	 * @param [out] switchingTimes: Switching times.
	 */
	void getSwitchingTimes(scalar_array_t& switchingTimes) const {switchingTimes = switchingTimes_;}

	/**
	 * Gets subsystem indexes.
	 *
	 * @param [out] systemStockIndexes: Subsystem indees.
	 */
	void getSubsystemIndexes(std::vector<size_t>& systemStockIndexes) const {systemStockIndexes = systemStockIndexes_;}

	/**
	 * Gets number of iterations.
	 *
	 * @return Number of iterations.
	 */
	size_t getNumIterations() const {return iteration_;}

	/**
	 * Gets a nominal state of subsystem cost in the given index.
	 *
	 * @param [in] index: The requested index.
	 * @param [out] timeTrajectory: The time stamp tarjectory for the requested subsystem.
	 * @param [out] stateTrajectory: The state tarjectory for the requested subsystem.
	 */
	virtual void getSingleCostNominalState(size_t index, scalar_array_t& timeTrajectory,
			state_vector_array_t& stateTrajectory) const = 0;

	/**
	 * Gets nominal state of subsystem costs.
	 *
	 * @param [out] timeTrajectoryStock: The time stamp tarjectory for each subsystem.
	 * @param [out] stateTrajectoryStock: The state tarjectory for each subsystem.
	 */
	void getCostNominalStates(std::vector<scalar_array_t>& timeTrajectoryStock,
			state_vector_array2_t& stateTrajectoryStock) const;

	/**
	 * Rewinds optimizer internal variables.
	 *
	 * @param [in] firstIndex: The index which we want to rewind to.
	 * @param [in] initRun: True only in the very first run of algorithm.
	 */
	void rewindOptimizer(const size_t& firstIndex, bool initRun=false);

protected:
	/**
	 * Sets a nominal state of subsystem cost in the given index.
	 *
	 * @param [in] index: The requested index.
	 * @param [in] timeTrajectory: The time stamp tarjectory for the requested subsystem.
	 * @param [in] stateTrajectory: The state tarjectory for the requested subsystem.
	 */
	virtual void setSingleCostNominalState(size_t index, const scalar_array_t& timeTrajectory,
			const state_vector_array_t& stateTrajectory) = 0;

	/**
	 * Sets nominal state of subsystem costs.
	 *
	 * @param [in] timeTrajectoryStock: The time stamp tarjectory for each subsystem.
	 * @param [in] stateTrajectoryStock: The state tarjectory for each subsystem.
	 */
	void setCostNominalStates(const std::vector<scalar_array_t>& timeTrajectoryStock,
			const state_vector_array2_t& stateTrajectoryStock);

	/**
	 * Solves Riccati equations for all the susbsystems in the systemStockIndexes.
	 *
	 * @param [in] learningRate: The optimal learning rate from line search scheme.
	 * @param [in] SmFinal: The final Sm for Riccati equation.
	 * @param [in] SvFinal: The final Sv for Riccati equation.
	 * @param [in] sFinal: The final s for Riccati equation.
	 */
	virtual void solveSequentialRiccatiEquations(const scalar_t& learningRate,
			const state_matrix_t& SmFinal,
			const state_vector_t& SvFinal,
			const eigen_scalar_t& sFinal) = 0;

	/**
	 * Solves a set of Riccati equations for the susbsystems in the given index.
	 *
	 * @param [in] index: The requested index.
	 * @param [in] learningRate: The optimal learning rate from line search scheme.
	 * @param [in] SmFinal: The final Sm for Riccati equation.
	 * @param [in] SvFinal: The final Sv for Riccati equation.
	 * @param [in] sFinal: The final s for Riccati equation.
	 */
	void solveSingleSequentialRiccatiEquation(const size_t& index, const scalar_t& learningRate,
			const state_matrix_t& SmFinal,
			const state_vector_t& SvFinal,
			const eigen_scalar_t& sFinal);

	/**
	 * Solves a set of Riccati equations for the susbsystems in the given index with given time trajectory stamp.
	 *
	 * @param [in] index: The requested index.
	 * @param [in] learningRate: The optimal learning rate from line search scheme.
	 * @param [in] nominalTimeTrajectory: The input array of the time trajectories.
	 * @param [in] SmFinal: The final Sm for the current Riccati equation.
	 * @param [in] SvFinal: The final Sv for the current Riccati equation.
	 * @param [in] sFinal: The final s for the current Riccati equation.
	 */
	void solveSingleSequentialRiccatiEquation(const size_t& index, const scalar_t& learningRate,
			const scalar_array_t& nominalTimeTrajectory,
			const state_matrix_t& SmFinal,
			const state_vector_t& SvFinal,
			const eigen_scalar_t& sFinal);

	/**
	 * Solves a set of erro Riccati equations for the susbsystems in the given index.
	 *
	 * @param [in] index: The requested index.
	 * @param [in] SveFinal: The final Sve for the current Riccati equation.
	 */
	void solveSingleErrorRiccatiEquation(const size_t& index,
			const state_vector_t& SveFinal);

	/**
	 * Full Backward Sweep method uses exp method instead of ode to solve Riccati equations.
	 * @param [in] index: The requested index.
	 * @param [in] SmFinal: The final Sm for the Riccati equation.
	 * @param [in] SvFinal: The final Sv for the Riccati equation.
	 * @param [in] SveFinal: The final Sve for the Riccati equation.
	 * @param [in] sFinal: The final s for the Riccati equation.
	 */
	void fullBackwardSweep(const size_t& index,
			const state_matrix_t& SmFinal, const state_vector_t& SvFinal,
			const state_vector_t& SveFinal, const eigen_scalar_t& sFinal);

	template<size_t DIM1, size_t DIM2>
	Eigen::Matrix<double, DIM1, DIM2> solveLTIMatrix(
			const Eigen::Matrix<double, DIM1, DIM1>& A,
			const Eigen::Matrix<double, DIM1, DIM2>& x0,
			const double& deltaTime);

	template<int DIM1>
	Eigen::Matrix<double, DIM1, 1> solveLTI(
			const Eigen::Matrix<double, DIM1, DIM1>& Gm,
			const Eigen::Matrix<double, DIM1, 1>& Gv,
			const Eigen::Matrix<double, DIM1, 1>& x0,
			const double& deltaTime);

	/**
	 * Computes the Lagrage multiplier over the given rollout.
	 *
	 * @param [in] timeTrajectoriesStock: rollout simulated time steps
	 * @param [in] stateTrajectoriesStock: rollout outputs
	 * @param [in] lagrangeMultiplierFunctionsStock: the coefficients of the linear function for lagrangeMultiplier
	 * @param [out] lagrangeTrajectoriesStock: lagrangeMultiplier value over the given trajectory
	 */
	void calculateRolloutLagrangeMultiplier(const std::vector<scalar_array_t>& timeTrajectoriesStock,
			const state_vector_array2_t& stateTrajectoriesStock,
			const std::vector<lagrange_t>& lagrangeMultiplierFunctionsStock,
			std::vector<std::vector<Eigen::VectorXd>>& lagrangeTrajectoriesStock);

	/**
	 * Computes the co-state over the given rollout.
	 *
	 * @param [in] timeTrajectoriesStock: rollout simulated time steps
	 * @param [in] stateTrajectoriesStock: rollout outputs
	 * @param [out] costateTrajectoriesStock: co-state vector for the given trajectory
	 */
	void calculateRolloutCostate(const std::vector<scalar_array_t>& timeTrajectoriesStock,
			const state_vector_array2_t& stateTrajectoriesStock,
			state_vector_array2_t& costateTrajectoriesStock);

	/**
	 * Calculates the linear function approximation of the type-1 constraint Lagrangian. This method uses the following variables:
	 * 			- constrained, linearized model
	 * 			- constrained, quadratized cost
	 *
	 * @param [out] lagrangeMultiplierFunctionsStock: the linear function approximation of the type-1 constraint Lagrangian.
	 */
	void calculateInputConstraintLagrangian(std::vector<lagrange_t>& lagrangeMultiplierFunctionsStock);

	/**
	 * compute the merit function for given rollout
	 *
	 * @param [in] timeTrajectoriesStock: simulation time trajectory
	 * @param [in] nc1TrajectoriesStock: rollout's number of active constraints in each time step
	 * @param [in] EvTrajectoryStock: rollout's constraints value
	 * @param [in] lagrangeTrajectoriesStock: constraint Lagrange multiplier for the given rollout
	 * @param [in] totalCost: the total cost of the trajectory
	 * @param [out] meritFunctionValue: the total merit function value of the trajectory
	 * @param [out] constraintISE: Integral of Square Error (ISE)
	 */
	void calculateMeritFunction(
			const std::vector<scalar_array_t>& timeTrajectoriesStock,
			const std::vector<std::vector<size_t> >& nc1TrajectoriesStock,
			const constraint1_vector_array2_t& EvTrajectoryStock,
			const std::vector<std::vector<Eigen::VectorXd>>&  lagrangeTrajectoriesStock,
			const scalar_t& totalCost,
			scalar_t& meritFunctionValue,
			scalar_t& constraintISE);

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

	size_t numSubsystems_;
	controller_array_t  nominalControllersStock_;
	std::vector<size_t> systemStockIndexes_;

	std::vector<std::shared_ptr<ControlledSystemBase<STATE_DIM, INPUT_DIM> > > subsystemDynamicsPtr_;
	std::vector<std::shared_ptr<DerivativesBase<STATE_DIM, INPUT_DIM> > > subsystemDerivativesPtr_;
	std::vector<std::shared_ptr<CostFunctionBaseOCS2<STATE_DIM, INPUT_DIM> > > subsystemCostFunctionsPtr_;
	controller_array_t lqpControllersStock_;

	state_vector_array_t stateOperatingPoints_;
	control_vector_array_t inputOperatingPoints_;

	scalar_t nominalTotalCost_;
	scalar_t nominalConstraint1ISE_;
	scalar_t constraint1MaxNorm_;
	scalar_t nominalConstraint2ISE_;
	scalar_t constraint2MaxNorm_;
	scalar_t learningRateStar_ = 1.0;

	std::vector<scalar_array_t> nominalTimeTrajectoriesStock_;
	state_vector_array2_t	 nominalStateTrajectoriesStock_;
	control_vector_array2_t  nominalInputTrajectoriesStock_;
	state_vector_array2_t   nominalcostateTrajectoriesStock_;

	std::vector<scalar_array_t> desiredTimeTrajectoriesStock_;
	state_vector_array2_t 		desiredStateTrajectoriesStock_;

	state_matrix_array2_t        AmTrajectoryStock_;
	control_gain_matrix_array2_t BmTrajectoryStock_;

	std::vector<std::vector<size_t>>    nc1TrajectoriesStock_;  	// nc1: Number of the Type-1  active constraints
	constraint1_vector_array2_t			EvTrajectoryStock_;
	constraint1_state_matrix_array2_t   CmTrajectoryStock_;
	constraint1_control_matrix_array2_t DmTrajectoryStock_;

	std::vector<std::vector<size_t> > 	nc2TrajectoriesStock_;  // nc2: Number of the Type-2 active constraints
	constraint2_vector_array2_t 		HvTrajectoryStock_;
	constraint2_state_matrix_array2_t 	FmTrajectoryStock_;
	std::vector<size_t> 				nc2FinalStock_;
	constraint2_vector_array_t 			HvFinalStock_;
	constraint2_state_matrix_array_t 	FmFinalStock_;

	eigen_scalar_array_t  qFinalStock_;
	state_vector_array_t  QvFinalStock_;
	state_matrix_array_t  QmFinalStock_;

	eigen_scalar_array2_t 		qTrajectoryStock_;
	state_vector_array2_t 		QvTrajectoryStock_;
	state_matrix_array2_t 		QmTrajectoryStock_;
	control_vector_array2_t		RvTrajectoryStock_;
	control_matrix_array2_t		RmTrajectoryStock_;
	control_feedback_array2_t 	PmTrajectoryStock_;

	control_matrix_array2_t RmInverseTrajectoryStock_;
	state_matrix_array2_t   AmConstrainedTrajectoryStock_;
	state_matrix_array2_t   QmConstrainedTrajectoryStock_;
	state_vector_array2_t  QvConstrainedTrajectoryStock_;
	control_matrix_array2_t RmConstrainedTrajectoryStock_;
	control_constraint1_matrix_array2_t DmDagerTrajectoryStock_;
	control_vector_array2_t   EvProjectedTrajectoryStock_;  // DmDager * Ev
	control_feedback_array2_t CmProjectedTrajectoryStock_;  // DmDager * Cm
	control_matrix_array2_t   DmProjectedTrajectoryStock_;  // DmDager * Dm
	control_gain_matrix_array2_t BmConstrainedTrajectoryStock_;
	control_feedback_array2_t 	PmConstrainedTrajectoryStock_;
	control_vector_array2_t 	RvConstrainedTrajectoryStock_;

	std::vector<scalar_array_t>	SsTimeTrajectoryStock_;
	std::vector<scalar_array_t> SsNormalizedTimeTrajectoryStock_;
	eigen_scalar_array2_t 		sTrajectoryStock_;
	state_vector_array2_t 		SvTrajectoryStock_;
	state_vector_array2_t 		SveTrajectoryStock_;
	state_matrix_array2_t 		SmTrajectoryStock_;

	eigen_scalar_array_t  sFinalStock_;
	state_vector_array_t  SvFinalStock_;
	state_vector_array_t  SveFinalStock_;
	state_matrix_array_t  SmFinalStock_;
	state_vector_array_t xFinalStock_;

	controller_array_t    deletedcontrollersStock_;

	scalar_array_t switchingTimes_;
	scalar_t initTime_;
	scalar_t finalTime_;
	state_vector_t initState_;
	size_t initActiveSubsystem_;
	size_t finalActiveSubsystem_;
	size_t iteration_;

	eigen_scalar_array_t iterationCost_;
	eigen_scalar_array_t iterationISE1_;

	Options_t options_;

	bool isRewinded_ = false;

	bool useDisjointRiccati_ = false;

public:
	template <size_t GSLQP_STATE_DIM, size_t GSLQP_INPUT_DIM>
	friend class GSLQP;
};

} // namespace ocs2

#include <GSLQ/implementation/SLQP_BASE.h>

#endif /* SLQP_H_ */
