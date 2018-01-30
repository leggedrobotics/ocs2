/*
 * SLQP_MP.h
 *
 * Multicore version of SLQP
 *
 *  Created on: Jun 20, 2016
 *      Author: farbodf, mgiftthaler
 */


#ifndef SLQP_MP_OCS2_H_
#define SLQP_MP_OCS2_H_

#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>

#include "ocs2_slq/SLQP_BASE.h"

namespace ocs2{

/**
 * This class implements multi-thread SLQ algorithm.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 * @tparam LOGIC_RULES_T: Logic Rules type (default NullLogicRules).
 */
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T=NullLogicRules<STATE_DIM,INPUT_DIM>>
class SLQP_MP : public SLQP_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef SLQP_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T> BASE;

	typedef typename BASE::DIMENSIONS DIMENSIONS;

	typedef typename DIMENSIONS::template LinearFunction_t<Eigen::Dynamic> lagrange_t;
	typedef typename DIMENSIONS::controller_t controller_t;
	typedef typename DIMENSIONS::controller_array_t controller_array_t;
	typedef typename DIMENSIONS::Options Options_t;
	typedef typename DIMENSIONS::size_array_t size_array_t;
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

	typedef typename BASE::controlled_system_base_t		 controlled_system_base_t;
	typedef typename BASE::event_handler_t	 			 event_handler_t;
	typedef typename BASE::derivatives_base_t			 derivatives_base_t;
	typedef typename BASE::constraint_base_t			 constraint_base_t;
	typedef typename BASE::cost_function_base_t			 cost_function_base_t;
	typedef typename BASE::operating_trajectories_base_t operating_trajectories_base_t;

	/**
	 * worker state enum
	 */
	enum WORKER_STATE {
		IDLE = 0,
		LINE_SEARCH,
		APPROXIMATE_LQ,
		CALCULATE_CONTROLLER,
		SOLVE_RICCATI,
		SHUTDOWN
	};

	/**
	 * Constructor
	 *
	 * @param [in] subsystemDynamicsPtr: Array of system dynamics and constraints for system's subsystems.
	 * @param [in] subsystemDerivativesPtr: Array of system dynamics and constraints derivatives for system's subsystems.
	 * @param [in] subsystemConstraintPtr: Array of constraint function and its derivatives for system's subsystems.
	 * @param [in] subsystemCostFunctionsPtr: Array of cost function and its derivatives for system's subsystems.
	 * @param [in] operatingTrajectoriesPtr: The operating trajectories of system which will be used for initialization of SLQ.
	 * @param [in] options: Structure containing the settings for the SLQ algorithm.
	 * @param [in] logicRules: The logic rules used for implementing mixed logical dynamical systems.
	 */
	SLQP_MP(const typename controlled_system_base_t::Ptr& subsystemDynamicsPtr,
			const typename derivatives_base_t::Ptr& subsystemDerivativesPtr,
			const typename constraint_base_t::Ptr& subsystemConstraintPtr,
			const typename cost_function_base_t::Ptr& subsystemCostFunctionsPtr,
			const typename operating_trajectories_base_t::Ptr& operatingTrajectoriesPtr,
			const Options_t& options = Options_t::Options(),
			const LOGIC_RULES_T& logicRules = LOGIC_RULES_T())

	: BASE(subsystemDynamicsPtr, subsystemDerivativesPtr, subsystemConstraintPtr, subsystemCostFunctionsPtr, operatingTrajectoriesPtr, options, logicRules),
	  workerTask_(IDLE),
	  subsystemProcessed_(0)
	{
		Eigen::initParallel();

		// initialize threads
		launchWorkerThreads();
	}

	/**
	 * Default destructor.
	 */
	~SLQP_MP();

	/**
	 * Line search on the feedforwrd parts of the controller. It uses the following approach for line search:
	 * The constraint TYPE-1 correction term is directly added through a user defined stepSize (defined in options_.constraintStepSize_).
	 * But the cost minimization term is optimized through a line-search strategy defined in SLQ options.
	 *
	 * @param [in] maxLearningRateStar: The maximum permitted learning rate.
	 */
	void lineSearch(scalar_t maxLearningRateStar=1.0) override;

	/**
	 * Runs the initialization method for single thread SLQ.
	 */
	void runInit() override;

	/**
	 * Solves Riccati equations for all the partitions.
	 *
	 * @param [in] learningRate: The optimal learning rate from line search scheme.
	 * @param [in] SmFinal: The final Sm for Riccati equation.
	 * @param [in] SvFinal: The final Sv for Riccati equation.
	 * @param [in] sFinal: The final s for Riccati equation.
	 */
	void solveSequentialRiccatiEquations(
			const scalar_t& learningRate,
			const state_matrix_t& SmFinal,
			const state_vector_t& SvFinal,
			const eigen_scalar_t& sFinal) override;

	/**
	 * Runs a single iteration of single thread SLQ.
	 *
	 * @param [in] SmFinal: The final Sm for Riccati equation.
	 * @param [in] SvFinal: The final Sv for Riccati equation.
	 * @param [in] sFinal: The final s for Riccati equation.
	 */
	void runIteration(const scalar_t& maxLearningRateStar,
			const state_matrix_t& SmFinal = state_matrix_t::Zero(),
			const state_vector_t& SvFinal = state_vector_t::Zero(),
			const eigen_scalar_t& sFinal = eigen_scalar_t::Zero()) override;

	/**
	 * Runs the exit method single thread SLQ.
	 *
	 * @param [in] SmFinal: The final Sm for Riccati equation.
	 * @param [in] SvFinal: The final Sv for Riccati equation.
	 * @param [in] sFinal: The final s for Riccati equation.
	 */
	void runExit(const state_matrix_t& SmFinal = state_matrix_t::Zero(),
			const state_vector_t& SvFinal = state_vector_t::Zero(),
			const eigen_scalar_t& sFinal = eigen_scalar_t::Zero()) override;

protected:
	/**
	 * Computes the linearized dynamics for a particular time partition.
	 *
	 * @param [in] partitionIndex: Time partition index
	 */
	void approximatePartitionLQ(const size_t& partitionIndex) override;

	/**
	 * Finds the next node's uncompleted LQ approximation and executes approximateLQWorker.
	 *
	 * @param [in] threadId: Thread ID
	 * @param [in] partitionIndex: Time partition index
	 */
	void executeApproximatePartitionLQWorker(size_t threadId, const size_t& partitionIndex);

	/**
	 * Computes the controller for a particular time partition
	 *
	 * @param partitionIndex: Time partition index
	 */
	void calculatePartitionController(const size_t& partitionIndex) override;

	/**
	 * Finds the next node's uncompleted CALCULATE_CONTROLLER task and executes calculateControllerWorker.
	 *
	 * @param [in] threadId: Thread ID
	 * @param [in] partitionIndex: Time partition index
	 */
	void executeCalculatePartitionController(size_t threadId, const size_t& partitionIndex);

	/**
	 * Launches worker threads
	 */
	void launchWorkerThreads();
	/**
	 * Sets number of worker threads
	 * @param [in] threadId: Thread ID
	 */
	void threadWork(size_t threadId);

	/**
	 * Execute line search worker on a thread with various learning rates and accept the result if it satisfies the step
	 * size policy (defined in options_.lsStepsizeGreedy_)
	 *
	 * @param [in] threadId
	 */
	void executeLineSearchWorker(size_t threadId);

	/**
	 * Distributes work
	 */
	void distributeWork();

	/**
	 * Solves Riccati equations for the partitions assigned to the given thread
	 *
	 * @param [in] threadId
	 */
	void executeRiccatiSolver(size_t threadId);

	/**
	 * a heuristic that generates a unique id for a process, such that we can manage the tasks.
	 * Generates a unique identifiers for subsystem, task, iteration:
	 * @param [in] iterateNo
	 * @param [in] workerState
	 * @param [in] subsystemId
	 * @return size_t:
	 */
	size_t generateUniqueProcessID (const size_t& iterateNo, const int workerState, const int subsystemId)
	{
		return (10e9*(workerState +1) + 10e6 * (subsystemId +1) + iterateNo+1);
	}

private:
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

	size_t alphaMax_;
	size_t alphaExpBest_;
	size_t alphaExpMax_;
	scalar_t baselineTotalCost_;
	std::atomic_size_t alphaTaken_;
	std::atomic_bool alphaBestFound_;
	std::atomic_size_t lsWorkerCompleted_;
	std::vector<bool> alphaProcessed_;

	// needed for lineSearch
	std::vector<lagrange_t> initLSlagrangeMultiplierFunctionsStock_;

public:
	template <size_t GSLQP_STATE_DIM, size_t GSLQP_INPUT_DIM>
	friend class GSLQP;
};

} // namespace ocs2

#include "implementation/SLQP_MP.h"


#endif /* SLQP_MP_H_ */
