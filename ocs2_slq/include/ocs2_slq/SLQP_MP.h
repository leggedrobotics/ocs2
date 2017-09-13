/*
 * SLQP_MP.h
 *
 * Multicore version of SLQP
 *
 *  Created on: Jun 20, 2016
 *      Author: mgiftthaler
 */


#ifndef SLQP_MP_OCS2_H_
#define SLQP_MP_OCS2_H_

#include <memory>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>

#include <ocs2_core/Dimensions.h>
#include <ocs2_core/integration/KillIntegrationEventHandler.h>

#include "ocs2_slq/SLQP_BASE.h"

namespace ocs2{

/**
 * SLQP MP Class
 * @tparam STATE_DIM
 * @tparam INPUT_DIM
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class SLQP_MP : public SLQP_BASE<STATE_DIM, INPUT_DIM>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef SLQP_BASE<STATE_DIM, INPUT_DIM> BASE;

	typedef typename BASE::DIMENSIONS DIMENSIONS;

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

	/**
	 * WORKER_STATE Enum
	 */
	enum WORKER_STATE {
		IDLE = 0,
		LINE_SEARCH,
		APPROXIMATE_LQ,
		CALCULATE_CONTROLLER_AND_LAGRANGIAN,
		SOLVE_RICCATI,
		SHUTDOWN
	};

	/**
	 * Constructor
	 * @param [in] subsystemDynamicsPtr
	 * @param [in] subsystemDerivativesPtr
	 * @param [in] subsystemCostFunctionsPtr
	 * @param [in] options
	 * @param [in] stateOperatingPoints
	 * @param [in] inputOperatingPoints
	 */
	SLQP_MP(const std::vector<std::shared_ptr<ControlledSystemBase<STATE_DIM, INPUT_DIM> > >& subsystemDynamicsPtr,
			const std::vector<std::shared_ptr<DerivativesBase<STATE_DIM, INPUT_DIM> > >& subsystemDerivativesPtr,
			const std::vector<std::shared_ptr<CostFunctionBaseOCS2<STATE_DIM, INPUT_DIM> > >& subsystemCostFunctionsPtr,
			const Options_t& options = Options_t(),
			const state_vector_array_t& stateOperatingPoints = state_vector_array_t(),
			const control_vector_array_t& inputOperatingPoints = control_vector_array_t() )
	: BASE(subsystemDynamicsPtr, subsystemDerivativesPtr, subsystemCostFunctionsPtr, options, stateOperatingPoints, inputOperatingPoints),
	  workerTask_(IDLE),
	  subsystemProcessed_(0)
	{
		Eigen::initParallel();

		// resize instances to correct number of threads + 1
		dynamics_.resize(options.nThreads_+1);
		linearizedSystems_.resize(options.nThreads_+1);
		costFunctions_.resize(options.nThreads_+1);
		integratorsODE45_.resize(options.nThreads_+1);
		killIntegrationEventHandlers_.resize(options.nThreads_+1);
		killIntegrationEventHandler_ = std::allocate_shared < KillIntegrationEventHandler<STATE_DIM>,
				Eigen::aligned_allocator<KillIntegrationEventHandler<STATE_DIM>> >(
						Eigen::aligned_allocator<KillIntegrationEventHandler<STATE_DIM>>() );

		// for controller design
		nominalStateFunc_.resize(options.nThreads_+1);
		nominalInputFunc_.resize(options.nThreads_+1);
		BmFunc_.resize(options.nThreads_+1);
		PmFunc_.resize(options.nThreads_+1);
		RmInverseFunc_.resize(options.nThreads_+1);
		RvFunc_.resize(options.nThreads_+1);
		EvProjectedFunc_.resize(options.nThreads_+1);
		CmProjectedFunc_.resize(options.nThreads_+1);
		DmProjectedFunc_.resize(options.nThreads_+1);

		// initialize threads
		launchWorkerThreads();
	}

	~SLQP_MP();

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
	void rollout(const scalar_t& initTime,
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
			constraint2_vector_array_t& HvFinalStock) override;

	// only for interfacing with GSLQP
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
	void rollout(const scalar_t& initTime,
			const state_vector_t& initState,
			const scalar_t& finalTime,
			const controller_array_t& controllersStock,
			std::vector<scalar_array_t>& timeTrajectoriesStock,
			state_vector_array2_t& stateTrajectoriesStock,
			control_vector_array2_t& inputTrajectoriesStock) override;

	/**
	 * Rollout
	 * @param [in] threadId
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
	void rollout(const size_t& threadId,
			const scalar_t& initTime,
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
			constraint2_vector_array_t& HvFinalStock);

	/**
	 * Rollout
	 * @param [in] threadId
	 * @param [in] initTime
	 * @param [in] initState
	 * @param [in] finalTime
	 * @param [in] controllersStock
	 * @param [out] timeTrajectoriesStock
	 * @param [out] stateTrajectoriesStock
	 * @param [out] inputTrajectoriesStock
	 */
	void rollout(const size_t& threadId,
			const scalar_t& initTime,
			const state_vector_t& initState,
			const scalar_t& finalTime,
			const controller_array_t& controllersStock,
			std::vector<scalar_array_t>& timeTrajectoriesStock,
			state_vector_array2_t& stateTrajectoriesStock,
			control_vector_array2_t& inputTrajectoriesStock);

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
	void rollout(const scalar_t& initTime,
			const state_vector_t& initState,
			const scalar_t& finalTime,
			const controller_array_t& controllersStock,
			state_vector_t& finalState,
			control_vector_t& finalInput,
			size_t& finalActiveSubsystemIndex) override;

	/**
	 * Calculates cost function
	 * @param [in] timeTrajectoriesStock
	 * @param [in] stateTrajectoriesStock
	 * @param [in] inputTrajectoriesStock
	 * @param [out] totalCost
	 * @param [out] threadId
	 */
	void calculateCostFunction(
			const std::vector<scalar_array_t>& timeTrajectoriesStock,
			const state_vector_array2_t& stateTrajectoriesStock,
			const control_vector_array2_t& inputTrajectoriesStock,
			scalar_t& totalCost,
			size_t threadId);

	// for interfacing with GSLQP
	/**
	 * compute the cost for a given rollout
	 * 		inputs:
	 * 			+ timeTrajectoriesStock:  rollout simulated time steps
	 * 			+ stateTrajectoriesStock: rollout outputs
	 * 			+ inputTrajectoriesStock: rollout control inputs
	 *			+ threadId: working thread, defaults to the thread with lowest id, thus this is the default thread for single-core cost computation
	 *				(allows to let method be called from the outside)
	 * 		outputs:
	 * 			+ totalCost: the total cost of the trajectory
	 * @param [in] timeTrajectoriesStock
	 * @param [in] stateTrajectoriesStock
	 * @param [in] inputTrajectoriesStock
	 * @param [out] totalCost
	 */
	void calculateCostFunction(
			const std::vector<scalar_array_t>& timeTrajectoriesStock,
			const state_vector_array2_t& stateTrajectoriesStock,
			const control_vector_array2_t& inputTrajectoriesStock,
			scalar_t& totalCost) override;

	/**
	 * Calculates cost function
	 * @param [in] timeTrajectoriesStock
	 * @param [in] stateTrajectoriesStock
	 * @param [in] inputTrajectoriesStock
	 * @param [in] nc2TrajectoriesStock
	 * @param [in] HvTrajectoryStock
	 * @param [in] nc2FinalStock
	 * @param [in] HvFinalStock
	 * @param [out] totalCost
	 */
	void calculateCostFunction(const std::vector<scalar_array_t>& timeTrajectoriesStock,
			const state_vector_array2_t& stateTrajectoriesStock,
			const control_vector_array2_t& inputTrajectoriesStock,
			const std::vector<std::vector<size_t> >& nc2TrajectoriesStock,
			const constraint2_vector_array2_t& HvTrajectoryStock,
			const std::vector<size_t>& nc2FinalStock,
			const constraint2_vector_array_t& HvFinalStock,
			scalar_t& totalCost) override;

	/**
	 * Calculates cost function
	 * @param [in] timeTrajectoriesStock
	 * @param [in] stateTrajectoriesStock
	 * @param [in] inputTrajectoriesStock
	 * @param [in] nc2TrajectoriesStock
	 * @param [in] HvTrajectoryStock
	 * @param [in] nc2FinalStock
	 * @param [in] HvFinalStock
	 * @param [out] totalCost
	 * @param [out] threadId
	 */
	void calculateCostFunction(const std::vector<scalar_array_t>& timeTrajectoriesStock,
			const state_vector_array2_t& stateTrajectoriesStock,
			const control_vector_array2_t& inputTrajectoriesStock,
			const std::vector<std::vector<size_t> >& nc2TrajectoriesStock,
			const constraint2_vector_array2_t& HvTrajectoryStock,
			const std::vector<size_t>& nc2FinalStock,
			const constraint2_vector_array_t& HvFinalStock,
			scalar_t& totalCost,
			size_t threadId);

	/**
	 * Sets up optimizer
	 */
	void setupOptimizer() override;

	/**
	 * Runs Init
	 */
	void runInit() override;

	/**
	 * Runs iteration
	 * @param [in] SmFinal
	 * @param [in] SvFinal
	 * @param [in] sFinal
	 */
	void runIteration(const state_matrix_t& SmFinal = state_matrix_t::Zero(),
			const state_vector_t& SvFinal = state_vector_t::Zero(),
			const eigen_scalar_t& sFinal = eigen_scalar_t::Zero()) override;

	/**
	 *
	 * @param [in] SmFinal
	 * @param [in] SvFinal
	 * @param [in] sFinal
	 */
	void runExit(const state_matrix_t& SmFinal = state_matrix_t::Zero(),
			const state_vector_t& SvFinal = state_vector_t::Zero(),
			const eigen_scalar_t& sFinal = eigen_scalar_t::Zero()) override;

	// get subsystem dynamics on main thread
	/**
	 *
	 */
	std::vector<std::shared_ptr<ControlledSystemBase<STATE_DIM, INPUT_DIM>> >& getSubsystemDynamicsPtrStock() override {
		return dynamics_[BASE::options_.nThreads_];
	}

	/**
	 * Gets single cost nominal state
	 * @param [in] index
	 * @param [out] timeTrajectory
	 * @param [out] stateTrajectory
	 */
	void getSingleCostNominalState(size_t index, scalar_array_t& timeTrajectory,
			state_vector_array_t& stateTrajectory) const override;

protected:
	/**
	 * Sets single cost nominal state
	 * @param [in] index
	 * @param [in] timeTrajectory
	 * @param [in] stateTrajectory
	 */
	void setSingleCostNominalState(size_t index, const scalar_array_t& timeTrajectory,
			const state_vector_array_t& stateTrajectory) override;

	/**
	 * Solves sequential Riccati equations
	 * @param [in] learningRate
	 * @param [in] SmFinal
	 * @param [in] SvFinal
	 * @param [in] sFinal
	 */
	void solveSequentialRiccatiEquations(const scalar_t& learningRate,
			const state_matrix_t& SmFinal,
			const state_vector_t& SvFinal,
			const eigen_scalar_t& sFinal) override;

	/**
	 * approximates the nonlinear problem as a linear-quadratic problem around the nominal
	 * state and control trajectories. This method updates the following variables:
	 *
	 * 		+ linearized system model
	 * 		+ dxdt = Am(t)x(t) + Bm(t)u(t)
	 * 		+ s.t. Cm(t)x(t) + Dm(t)t(t) + Ev(t) = 0
	 * 		+ BASE::AmTrajectoryStock_: Am matrix
	 * 		+ BASE::BmTrajectoryStock_: Bm matrix
	 * 		+ BASE::CmTrajectoryStock_: Cm matrix
	 * 		+ BASE::DmTrajectoryStock_: Dm matrix
	 * 		+ BASE::EvTrajectoryStock_: Ev vector
	 *
	 * 		+ quadratized intermediate cost function
	 * 		+ intermediate cost: q(t) + 0.5 y(t)Qm(t)y(t) + y(t)'Qv(t) + u(t)'Pm(t)y(t) + 0.5u(t)'Rm(t)u(t) + u(t)'Rv(t)
	 * 		+ BASE::qTrajectoryStock_:  q
	 * 		+ BASE::QvTrajectoryStock_: Qv vector
	 * 		+ BASE::QmTrajectoryStock_: Qm matrix
	 * 		+ BASE::PmTrajectoryStock_: Pm matrix
	 * 		+ BASE::RvTrajectoryStock_: Rv vector
	 * 		+ BASE::RmTrajectoryStock_: Rm matrix
	 * 		+ BASE::RmInverseTrajectoryStock_: inverse of Rm matrix
	 *
	 * 		+ quadratized final cost in the last subsystem: qFinal(t) + 0.5 y(t)QmFinal(t)y(t) + y(t)'QvFinal(t)
	 * 		+ BASE::qFinal_: qFinal
	 * 		+ BASE::qFinal_: QvFinal vector
	 * 		+ BASE::qFinal_: QmFinal matrix
	 *
	 * 		+ as well as the constrained coefficients of
	 * 			linearized system model
	 * 			quadratized intermediate cost function
	 * 			quadratized final cost
	 */
	void approximateOptimalControlProblem();

	/**
	 * Calculates controller
	 */
	void calculateController();

	/**
	 * line search on the feedforward parts of the controller and lagrange multipliers.
	 * Based on the option flag lineSearchByMeritFuntion_ it uses two different approaches for line search:
	 * 		+ the constraint correction term is added by a user defined stepSize.
	 * 		The line search uses the pure cost function for choosing the best stepSize.
	 */
	void lineSearch();

private:

	/**
	 * Launches worker threads
	 */
	void launchWorkerThreads();
	/**
	 * Sets number of worker threads
	 * @param [in] threadId
	 */
	void threadWork(size_t threadId);

	// main function for sub-tasks
	/**
	 * Computes the linearized dynamics for a particular subsystem
	 * @param [in] sysIndex
	 */
	void approximateSubsystemLQ(const size_t sysIndex); // computes the linearized dynamics for a particular subsystem

	// worker functions
	/**
	 * Approximates subsystem LQWorker
	 * @param [in] threadId
	 * @param [in] subsystemProcessed
	 * @return
	 */
	size_t approximateSubsystemLQWorker(size_t threadId, size_t subsystemProcessed);		//Worker functions

	/**
	 * Calculates controller
	 * @param [in] threadId
	 * @param [in] subsystemProcessed
	 * @return
	 */
	size_t calculateControllerWorker(size_t threadId, size_t subsystemProcessed);

	/**
	 * Line search worker
	 * @param [in] threadId
	 */
	void lineSearchWorker(size_t threadId);

	// execute methods
	/**
	 * Execute methods
	 * @param [in] threadId
	 * @param [in] k
	 * @param [in] subsystemProcessed
	 * @return size_t
	 */
	size_t executeApproximateSubsystemLQ(size_t threadId, size_t k, size_t subsystemProcessed);	// Computes the linearized dynamics

	/**
	 * Execute calculate controller
	 * @param [in] threadId
	 * @param [in] k
	 * @param [in] subsystemProcessed
	 * @return size_t
	 */
	size_t executeCalculateController(size_t threadId, size_t k, size_t subsystemProcessed);

	/**
	 * Executes LineSearch
	 * @param [in] threadId
	 * @param [in] learningRate
	 * @param [out] lsTotalCost
	 * @param [out] lsControllersStock
	 * @param [out] lsTimeTrajectoriesStock
	 * @param [out] lsStateTrajectoriesStock
	 * @param [out] lsInputTrajectoriesStock
	 * @param [out] lsNc1TrajectoriesStock
	 * @param [out] lsEvTrajectoryStock
	 * @param [out] lsNc2TrajectoriesStock
	 * @param [out] lsHvTrajectoryStock
	 * @param [out] lsNc2FinalStock
	 * @param [out] lsHvFinalStock
	 */
	void executeLineSearch(
			size_t threadId, double learningRate,
			scalar_t& lsTotalCost,
			controller_array_t& lsControllersStock,
			std::vector<scalar_array_t>& lsTimeTrajectoriesStock,
			state_vector_array2_t& lsStateTrajectoriesStock,
			control_vector_array2_t& lsInputTrajectoriesStock,
			std::vector<std::vector<size_t> >& lsNc1TrajectoriesStock,
			constraint1_vector_array2_t& lsEvTrajectoryStock,
			std::vector<std::vector<size_t> >& lsNc2TrajectoriesStock,
			constraint2_vector_array2_t& lsHvTrajectoryStock,
			std::vector<size_t>& lsNc2FinalStock,
			constraint2_vector_array_t& lsHvFinalStock);

	/**
	 * Distributes work
	 */
	void distributeWork();

	/**
	 * Solves sequential Riccati equations worker
	 * @param [in] threadId
	 */
	void solveSequentialRiccatiEquationsWorker(size_t threadId);

	/**
	 * a heuristic that generates a unique id for a process, such that we can manage the tasks.
	 * Generates a unique identifiers for subsystem, task, iteration:
	 * @param [in] iterateNo
	 * @param [in] workerState
	 * @param [in] subsystemId
	 * @return
	 */
	size_t generateUniqueProcessID (const size_t& iterateNo, const int workerState, const int subsystemId)
	{
		return (10e9*(workerState +1) + 10e6 * (subsystemId +1) + iterateNo+1);
	}

	/**
	 * for nice debug printing
	 * @param [in] text
	 */
	inline void printString(const std::string& text){std::cout << text << std::endl;}

	std::vector<std::vector<std::shared_ptr<ControlledSystemBase<STATE_DIM, INPUT_DIM> > > > dynamics_;
	std::vector<std::vector<std::shared_ptr<DerivativesBase<STATE_DIM, INPUT_DIM> > > > linearizedSystems_;
	std::vector<std::vector<std::shared_ptr<CostFunctionBaseOCS2<STATE_DIM, INPUT_DIM> > > > costFunctions_;

	std::vector<std::vector<std::shared_ptr<ODE45<STATE_DIM> > > > integratorsODE45_;
	std::vector<std::vector<std::shared_ptr<KillIntegrationEventHandler<STATE_DIM>>>> killIntegrationEventHandlers_;
	std::shared_ptr<KillIntegrationEventHandler<STATE_DIM>> killIntegrationEventHandler_;

	// multithreading helper variables
	std::vector<std::thread> workerThreads_;
	std::atomic_bool workersActive_;
	std::atomic_int workerTask_;
	std::atomic_int subsystemProcessed_;		// the subsystem the threads are currently working on

	std::mutex workerWakeUpMutex_;
	std::condition_variable workerWakeUpCondition_;
	std::mutex kCompletedMutex_;
	std::condition_variable kCompletedCondition_;
	std::mutex lineSearchResultMutex_;
	std::mutex alphaBestFoundMutex_;
	std::condition_variable alphaBestFoundCondition_;

	std::vector<size_t> KMax_subsystem_approx_;		// denotes the number of integration steps for a particular subsystem i
	std::vector<size_t> KMax_subsystem_ctrl_;
	std::array<std::atomic_size_t, 100> kTaken_approx_;
	std::array<std::atomic_size_t, 100> kCompleted_approx_;
	std::array<std::atomic_size_t, 100> kTaken_ctrl_;
	std::array<std::atomic_size_t, 100> kCompleted_ctrl_;

	// parallel Riccati solver
	std::array<std::atomic_bool, 100> subsystemsDone_;
	std::array<std::atomic_bool, 100> subsystemsProcessing_;
	std::atomic_int numSubsystemsProcessed_;
	std::mutex riccatiSolverMutex_;
	std::mutex riccatiSolverBarrierMutex_;
	std::mutex riccatiSolverBarrierNotifyMutex_;
	std::mutex riccatiSolverDataMutex_;
	std::condition_variable riccatiSolverCompletedCondition_;
	scalar_t riccatiSolverLearningRate_ = 1.0;
	std::vector<int> startingIndicesRiccatiWorker_;
	std::vector<int> endingIndicesRiccatiWorker_;

	std::atomic_size_t alphaTaken_;
	size_t alphaMax_;
	size_t alphaExpBest_;
	size_t alphaExpMax_;
	std::atomic_bool alphaBestFound_;
	std::vector<size_t> alphaProcessed_;
	double lowestTotalCost_;
	std::atomic_size_t lsWorkerCompleted_;

	// for controller design
	// functions for controller and lagrange multiplier
	std::vector<LinearInterpolation<state_vector_t,Eigen::aligned_allocator<state_vector_t> >>   	nominalStateFunc_;
	std::vector<LinearInterpolation<control_vector_t,Eigen::aligned_allocator<control_vector_t> >> 	nominalInputFunc_;
	std::vector<LinearInterpolation<control_gain_matrix_t,Eigen::aligned_allocator<control_gain_matrix_t> >> BmFunc_;
	std::vector<LinearInterpolation<control_feedback_t,Eigen::aligned_allocator<control_feedback_t> >> PmFunc_;
	std::vector<LinearInterpolation<control_matrix_t,Eigen::aligned_allocator<control_matrix_t> >>     RmInverseFunc_;
	std::vector<LinearInterpolation<control_vector_t,Eigen::aligned_allocator<control_vector_t> >>     RvFunc_;
	std::vector<LinearInterpolation<control_vector_t,Eigen::aligned_allocator<control_vector_t> >>     EvProjectedFunc_;
	std::vector<LinearInterpolation<control_feedback_t,Eigen::aligned_allocator<control_feedback_t> >> CmProjectedFunc_;
	std::vector<LinearInterpolation<control_matrix_t,Eigen::aligned_allocator<control_matrix_t> >>     DmProjectedFunc_;

	// needed for lineSearch
	controller_array_t 		initLScontrollersStock_;
	std::vector<lagrange_t> initLSlagrangeMultiplierFunctionsStock_;

public:
	template <size_t GSLQP_STATE_DIM, size_t GSLQP_INPUT_DIM>
	friend class GSLQP;
};

} // namespace ocs2

#include "implementation/SLQP_MP.h"


#endif /* SLQP_MP_H_ */
