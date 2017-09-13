/*
 * SLQP_BASE.h
 *
 *  Created on: August 14, 2016
 *      Author: mgiftthaler
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
 * SLQP Base Class
 * @tparam STATE_DIM
 * @tparam INPUT_DIM
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
	 * @param [in] subsystemDynamicsPtr
	 * @param [in] subsystemDerivativesPtr
	 * @param [in] subsystemCostFunctionsPtr
	 * @param [in] options
	 * @param [in] stateOperatingPoints
	 * @param [in] inputOperatingPoints
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


	virtual ~SLQP_BASE() {}

	/**
	 * only for interfacing with GSLQP
	 * @param [in] initTime
	 * @param [in] initState
	 * @param [in] finalTime
	 * @param [in] controllersStock
	 * @param [out] timeTrajectoriesStock
	 * @param [out] stateTrajectoriesStock
	 * @param [out] inputTrajectoriesStock
	 * @param [out] nc1TrajectoriesStock
	 * @param [out] EvTrajectoryStock
	 * @param [out] nc2TrajectoriesStock
	 * @param [out] HvTrajectoryStock
	 * @param [out] nc2FinalStock
	 * @param [out] HvFinalStock
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
	 * only for interfacing with GSLQP
	 * @param [in] initTime
	 * @param [in] initState
	 * @param [in] finalTime
	 * @param [in] controllersStock
	 * @param [out] timeTrajectoriesStock
	 * @param [out] stateTrajectoriesStock
	 * @param [out] inputTrajectoriesStock
	 */
	virtual void rollout(const double& initTime,
			const state_vector_t& initState,
			const scalar_t& finalTime,
			const controller_array_t& controllersStock,
			std::vector<scalar_array_t>& timeTrajectoriesStock,
			state_vector_array2_t& stateTrajectoriesStock,
			control_vector_array2_t& inputTrajectoriesStock) = 0;

	/**
	 * Rollout
	 * @param [in] initTime
	 * @param [in] initState
	 * @param [in] finalTime
	 * @param [in] controllersStock
	 * @param [out] finalState
	 * @param [out] finalInput
	 * @param [out] finalActiveSubsystemIndex
	 */
	virtual void rollout(const scalar_t& initTime,
			const state_vector_t& initState,
			const scalar_t& finalTime,
			const controller_array_t& controllersStock,
			state_vector_t& finalState,
			control_vector_t& finalInput,
			size_t& finalActiveSubsystemIndex) = 0;

	/**
	 * Calculates cost
	 * @param [in] timeTrajectoriesStock
	 * @param [in] stateTrajectoriesStock
	 * @param [in] inputTrajectoriesStock
	 * @param [out] totalCost
	 */
	virtual void calculateCostFunction(const std::vector<scalar_array_t>& timeTrajectoriesStock,
			const state_vector_array2_t& stateTrajectoriesStock,
			const control_vector_array2_t& inputTrajectoriesStock,
			scalar_t& totalCost) = 0;

	/**
	 * Calaculates cost
	 * @param [in] timeTrajectoriesStock
	 * @param [in] stateTrajectoriesStock
	 * @param [in] inputTrajectoriesStock
	 * @param [in] nc2TrajectoriesStock
	 * @param [in] HvTrajectoryStock
	 * @param [in] nc2FinalStock
	 * @param [in] HvFinalStock
	 * @param [out] totalCost
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
	 * get the calculated optimal controller structure
	 * @param [out] controllersStock
	 */
	void getController(controller_array_t& controllersStock) const;

	/**
	 * get the calculated optimal controller structure
	 * @param [out] controllersStock
	 */
	void getControllerPtr(std::shared_ptr<controller_array_t>& controllersStock) const;

	/**
	 * Controller
	 * @param [in] index
	 * @return
	 */
	const controller_t& controller(size_t index) const;

	/**
	 * Sets up uptomizer
	 */
	virtual void setupOptimizer() = 0;

	/**
	 * Runs Init method
	 */
	virtual void runInit() = 0;

	/**
	 * Runs iteration
	 * @param [in] SmFinal
	 * @param [in] SvFinal
	 * @param [in] sFinal
	 */
	virtual void runIteration(const state_matrix_t& SmFinal  = state_matrix_t::Zero(),
			const state_vector_t& SvFinal = state_vector_t::Zero(),
			const eigen_scalar_t& sFinal   = eigen_scalar_t::Zero()) = 0;

	/**
	 * Runs exit method
	 * @param [in] SmFinal
	 * @param [in] SvFinal
	 * @param [in] sFinal
	 */
	virtual void runExit(const state_matrix_t& SmFinal = state_matrix_t::Zero(),
			const state_vector_t& SvFinal = state_vector_t::Zero(),
			const eigen_scalar_t& sFinal = eigen_scalar_t::Zero())  = 0;

	/**
	 * run the SLQP algorithm for a given state and switching times
	 * @param [in] initTime
	 * @param [in] initState
	 * @param [in] finalTime
	 * @param [in] systemStockIndexes
	 * @param [in] switchingTimes
	 * @param [in] controllersStock
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
	 * Gets subsystem dynamics PtrStock
	 */
	virtual std::vector<std::shared_ptr<ControlledSystemBase<STATE_DIM, INPUT_DIM>> >& getSubsystemDynamicsPtrStock() = 0;

	/**
	 * calculate the value function for the given time and state vector
	 * 		inputs
	 * 			+ time: inquiry time
	 * 			+ state: inquiry state
	 *
	 * 		output:
	 * 			+ valueFuntion: value function at the inquiry time and state
	 * @param [in] time
	 * @param [in] output
	 * @param [out] valueFuntion
	 */
	virtual void getValueFuntion(const scalar_t& time, const state_vector_t& output, scalar_t& valueFuntion);

	/**
	 * calculate the cost function at the initial time
	 * 		inputs
	 * 			+ initOutput: initial state
	 *
	 * 		output:
	 * 			+ cost function value
	 * 			+ cost function value plus the constraint ISE multiplied by pho
	 * @param [out] costFunction
	 * @param [out] constraintISE
	 */
	virtual void getCostFuntion(scalar_t& costFunction, scalar_t& constraintISE);

	/**
	 * Gets nominal trajectories
	 * @param [out] nominalTimeTrajectoriesStock
	 * @param [out] nominalStateTrajectoriesStock
	 * @param [out] nominalInputTrajectoriesStock
	 */
	virtual void getNominalTrajectories(std::vector<scalar_array_t>& nominalTimeTrajectoriesStock,
			state_vector_array2_t& nominalStateTrajectoriesStock,
			control_vector_array2_t& nominalInputTrajectoriesStock);

	/**
	 * Gets nominal trajectories
	 * @param [out] nominalTimeTrajectoriesStock
	 * @param [out] nominalStateTrajectoriesStock
	 * @param [out] nominalInputTrajectoriesStock
	 */
	virtual void getNominalTrajectoriesPtr(std::shared_ptr<std::vector<scalar_array_t>>& nominalTimeTrajectoriesStock,
			std::shared_ptr<state_vector_array2_t>& nominalStateTrajectoriesStock,
			std::shared_ptr<control_vector_array2_t>& nominalInputTrajectoriesStock);

	/**
	 * options
	 * @return Options_t&
	 */
	Options_t& options() {return options_;}

	/**
	 * Gets Iterations Log
	 * @param [out] iterationCost
	 * @param [out] iterationISE1
	 */
	void getIterationsLog(eigen_scalar_array_t& iterationCost, eigen_scalar_array_t& iterationISE1) const {
		iterationCost = iterationCost_;
		iterationISE1 = iterationISE1_;
	}

	/**
	 * Uses Disjoint Riccati
	 * @param [in] useDisjointRiccati
	 */
	void setUseDisjointRiccati(bool useDisjointRiccati) {useDisjointRiccati_= useDisjointRiccati;}

	/**
	 * Calculates constraints
	 * @param [in] timeTrajectoriesStock
	 * @param [in] nc1TrajectoriesStock
	 * @param [in] EvTrajectoriesStock
	 * @param [out] constraintISE
	 * @return double
	 */
	double calculateConstraintISE(const std::vector<scalar_array_t>& timeTrajectoriesStock,
			const std::vector<std::vector<size_t>>& nc1TrajectoriesStock,
			const constraint1_vector_array2_t& EvTrajectoriesStock,
			scalar_t& constraintISE);

	/**
	 * Finds active subsystem index
	 * @param [in] switchingTimes
	 * @param [in] time
	 * @return size_t
	 */
	static size_t findActiveSubsystemIndex(const scalar_array_t& switchingTimes, const double& time);

	/**
	 * Truncates controller
	 * @param [in] switchingTimes
	 * @param [in] initTime
	 * @param [out] controllersStock
	 * @param [out] initActiveSubsystemIndex
	 * @param [out] deletedcontrollersStock
	 */
	void truncateConterller(const scalar_array_t& switchingTimes,
			const double& initTime,
			controller_array_t& controllersStock,
			size_t& initActiveSubsystemIndex,
			controller_array_t& deletedcontrollersStock);

	/**
	 * Sets switching times
	 * @param [in] switchingTimes
	 */
	void setSwitchingTimes(const scalar_array_t& switchingTimes) {switchingTimes_ = switchingTimes;}

	/**
	 * Gets switching times
	 * @param [out] switchingTimes
	 */
	void getSwitchingTimes(scalar_array_t& switchingTimes) const {switchingTimes = switchingTimes_;}

	//	void setSubsystemIndexes(const std::vector<size_t>& systemStockIndexes) {systemStockIndexes_ = systemStockIndexes;}
	/**
	 * Gets subsystem index
	 * @param [out] systemStockIndexes
	 */
	void getSubsystemIndexes(std::vector<size_t>& systemStockIndexes) const {systemStockIndexes = systemStockIndexes_;}

	/**
	 * Gets number of iterations
	 * @return size_t
	 */
	size_t getNumIterations() const {return iteration_;}

	/**
	 * Gets single nominal state
	 * @param [in] index
	 * @param [out] timeTrajectory
	 * @param [out] stateTrajectory
	 */
	virtual void getSingleCostNominalState(size_t index, scalar_array_t& timeTrajectory,
			state_vector_array_t& stateTrajectory) const = 0;

	/**
	 * Gets cost nominal states
	 * @param [out] timeTrajectoryStock
	 * @param [out] stateTrajectoryStock
	 */
	void getCostNominalStates(std::vector<scalar_array_t>& timeTrajectoryStock,
			state_vector_array2_t& stateTrajectoryStock) const;

	/**
	 * Rewinds optimizer
	 * @param [in] firstIndex
	 * @param [in] initRun
	 */
	void rewindOptimizer(const size_t& firstIndex, bool initRun=false);

protected:
	/**
	 * Sets single cost nominal state
	 * @param [in] index
	 * @param [in] timeTrajectory
	 * @param [in] stateTrajectory
	 */
	virtual void setSingleCostNominalState(size_t index, const scalar_array_t& timeTrajectory,
			const state_vector_array_t& stateTrajectory) = 0;

	/**
	 * Sets cost nominal states
	 * @param [in] timeTrajectoryStock
	 * @param [in] stateTrajectoryStock
	 */
	void setCostNominalStates(const std::vector<scalar_array_t>& timeTrajectoryStock,
			const state_vector_array2_t& stateTrajectoryStock);

	/**
	 * Solves sequential Riccati equations
	 * @param [in] learningRate
	 * @param [in] SmFinal
	 * @param [in] SvFinal
	 * @param [in] sFinal
	 */
	virtual void solveSequentialRiccatiEquations(const scalar_t& learningRate,
			const state_matrix_t& SmFinal,
			const state_vector_t& SvFinal,
			const eigen_scalar_t& sFinal) = 0;

	/**
	 * Solves single sequential Riccati equation
	 * @param [in] index
	 * @param [in] learningRate
	 * @param [in] SmFinal
	 * @param [in] SvFinal
	 * @param [in] sFinal
	 */
	void solveSingleSequentialRiccatiEquation(const size_t& index, const scalar_t& learningRate,
			const state_matrix_t& SmFinal,
			const state_vector_t& SvFinal,
			const eigen_scalar_t& sFinal);

	/**
	 * Solves single sequential Riccati equation
	 * @param [in] index
	 * @param [in] learningRate
	 * @param [in] nominalTimeTrajectory
	 * @param [in] SmFinal
	 * @param [in] SvFinal
	 * @param [in] sFinal
	 */
	void solveSingleSequentialRiccatiEquation(const size_t& index, const scalar_t& learningRate,
			const scalar_array_t& nominalTimeTrajectory,
			const state_matrix_t& SmFinal,
			const state_vector_t& SvFinal,
			const eigen_scalar_t& sFinal);

	/**
	 * Solves single error Riccati equation
	 * @param [in] index
	 * @param [in] SveFinal
	 */
	void solveSingleErrorRiccatiEquation(const size_t& index,
			const state_vector_t& SveFinal);

	/**
	 * full Backward Sweep
	 * @param [in] index
	 * @param [in] SmFinal
	 * @param [in] SvFinal
	 * @param [in] SveFinal
	 * @param [in] sFinal
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
	 * compute the Lagrage multiplier over the given rollout
	 * 		inputs:
	 * 			+ timeTrajectoriesStock: rollout simulated time steps
	 * 			+ stateTrajectoriesStock: rollout outputs
	 * 			+ lagrangeMultiplierFunctionsStock: the coefficients of the linear function for lagrangeMultiplier
	 *
	 * 		outputs:
	 * 			+ lagrangeTrajectoriesStock: lagrangeMultiplier value over the given trajectory
	 * @param [in] timeTrajectoriesStock
	 * @param [in] stateTrajectoriesStock
	 * @param [in] lagrangeMultiplierFunctionsStock
	 * @param [out] lagrangeTrajectoriesStock
	 */
	void calculateRolloutLagrangeMultiplier(const std::vector<scalar_array_t>& timeTrajectoriesStock,
			const state_vector_array2_t& stateTrajectoriesStock,
			const std::vector<lagrange_t>& lagrangeMultiplierFunctionsStock,
			std::vector<std::vector<Eigen::VectorXd>>& lagrangeTrajectoriesStock);

	/**
	 * compute the co-state over the given rollout
	 * 		inputs:
	 * 			+ timeTrajectoriesStock: rollout simulated time steps
	 * 			+ stateTrajectoriesStock: rollout outputs
	 *
	 * 		outputs:
	 * 			+ costateTrajectoriesStock: co-state vector for the given trajectory
	 * @param [in] timeTrajectoriesStock
	 * @param [in] stateTrajectoriesStock
	 * @param [out] costateTrajectoriesStock
	 */
	void calculateRolloutCostate(const std::vector<scalar_array_t>& timeTrajectoriesStock,
			const state_vector_array2_t& stateTrajectoriesStock,
			state_vector_array2_t& costateTrajectoriesStock);

	/**
	 * calculates the linear function approximation of the type-1 constraint Lagrangian:
	 * 		This method uses the following variables:
	 * 			+ constrained, linearized model
	 * 			+ constrained, quadratized cost
	 *
	 * 		The method outputs:
	 * 			+ lagrangeMultiplierFunctionsStock: the linear function approximation of the type-1 constraint Lagrangian.
	 * @param [out] lagrangeMultiplierFunctionsStock
	 */
	void calculateInputConstraintLagrangian(std::vector<lagrange_t>& lagrangeMultiplierFunctionsStock);

	/**
	 * compute the merit function for given rollout
	 * 		inputs:
	 * 			+ timeTrajectoriesStock: simulation time trajectory
	 * 			+ nc1TrajectoriesStock: rollout's number of active constraints in each time step
	 * 			+ EvTrajectoryStock: rollout's constraints value
	 * 			+ lagrangeTrajectoriesStock: constraint Lagrange multiplier for the given rollout
	 * 			+ totalCost: the total cost of the trajectory
	 *
	 * 		outputs:
	 * 			+ meritFuntionValue: the merit function value
	 * 			+ constraintISE: integral of Square Error (ISE)
	 * @param [in] timeTrajectoriesStock
	 * @param [in] nc1TrajectoriesStock
	 * @param [in] EvTrajectoryStock
	 * @param [in] lagrangeTrajectoriesStock
	 * @param [in] totalCost
	 * @param [out] meritFunctionValue
	 * @param [out] constraintISE
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
	 * makePSD
	 * @tparam Derived
	 * @param [out] squareMatrix
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
