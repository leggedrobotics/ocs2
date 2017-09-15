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
 * This class implements multi-thread SLQ algorithm.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
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
	 * worker state enum
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
	 *
	 * @param [in] subsystemDynamicsPtr: Array of system dynamics and constraints for system's subsystems.
	 * @param [in] subsystemDerivativesPtr: Array of system dynamics and constraints dervatives for system's subsystems.
	 * @param [in] subsystemCostFunctionsPtr: Array of cost function and its dervatives for system's subsystems.
	 * @param [in] options: Structure containing the settings for the SLQ algorithm.
	 * @param [in] stateOperatingPoints: The state operating points for system's subsystems which will be used for initialization of SLQ.
	 * @param [in] inputOperatingPoints: The input operating points for system's subsystems which will be used for initialization of SLQ.
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

	/**
	 * Default destructor.
	 */
	~SLQP_MP();

	/**
	 * Forward integrate the system dynamics with given controller. It uses the given control policies and initial state,
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

	/**
	 * Forward integrate the system dynamics with given controller. It uses the given control policies and initial state,
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
	void rollout(const scalar_t& initTime,
			const state_vector_t& initState,
			const scalar_t& finalTime,
			const controller_array_t& controllersStock,
			std::vector<scalar_array_t>& timeTrajectoriesStock,
			state_vector_array2_t& stateTrajectoriesStock,
			control_vector_array2_t& inputTrajectoriesStock) override;

	/**
	 * Forward integrate the system dynamics with given controller. It uses the given control policies and initial state,
	 * to integrate the system dyanmics and calculate the costraints in time period [initTime, finalTime].
	 *
	 * @param [in] threadId: Working thread.
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
	 * Forward integrate the system dynamics with given controller. It uses the given control policies and initial state,
	 * to integrate the system dyanmics in time period [initTime, finalTime].
	 *
	 * @param [in] threadId: Working thread.
	 * @param [in] initTime: Initial time.
	 * @param [in] initState: Initial state.
	 * @param [in] finalTime: Final time.
	 * @param [in] controllersStock: Array of control policies.
	 * @param [out] timeTrajectoriesStock: Array of trajectories containing the output time trajectory stamp.
	 * @param [out] stateTrajectoriesStock: Array of trajectories containing the output state trajectory.
	 * @param [out] inputTrajectoriesStock: Array of trajectories containing the output control input trajectory.
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
	 * Forward integrate the system dynamics with given controller.  It uses the given control policies and initial state,
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
	void rollout(const scalar_t& initTime,
			const state_vector_t& initState,
			const scalar_t& finalTime,
			const controller_array_t& controllersStock,
			state_vector_t& finalState,
			control_vector_t& finalInput,
			size_t& finalActiveSubsystemIndex) override;

	/**
	 * Calculates cost of a rollout.
	 *
	 * @param [in] timeTrajectoriesStock: Array of trajectories containing the time trajectory stamp of a rollout.
	 * @param [in] stateTrajectoriesStock: Array of trajectories containing the state trajectory of a rollout.
	 * @param [in] inputTrajectoriesStock: Array of trajectories containing the control input trajectory of a rollout.
	 * @param [out] totalCost: The total cost of the rollout.
	 * @param [in] threadId: Working thread.
	 */
	void calculateCostFunction(
			const std::vector<scalar_array_t>& timeTrajectoriesStock,
			const state_vector_array2_t& stateTrajectoriesStock,
			const control_vector_array2_t& inputTrajectoriesStock,
			scalar_t& totalCost,
			size_t threadId);

	/**
	 * Calculates cost of a rollout.
	 *
	 * @param [in] timeTrajectoriesStock: Array of trajectories containing the time trajectory stamp of a rollout.
	 * @param [in] stateTrajectoriesStock: Array of trajectories containing the state trajectory of a rollout.
	 * @param [in] inputTrajectoriesStock: Array of trajectories containing the control input trajectory of a rollout.
	 * @param [out] totalCost: The total cost of the rollout.
	 */
	void calculateCostFunction(
			const std::vector<scalar_array_t>& timeTrajectoriesStock,
			const state_vector_array2_t& stateTrajectoriesStock,
			const control_vector_array2_t& inputTrajectoriesStock,
			scalar_t& totalCost) override;

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
	void calculateCostFunction(const std::vector<scalar_array_t>& timeTrajectoriesStock,
			const state_vector_array2_t& stateTrajectoriesStock,
			const control_vector_array2_t& inputTrajectoriesStock,
			const std::vector<std::vector<size_t> >& nc2TrajectoriesStock,
			const constraint2_vector_array2_t& HvTrajectoryStock,
			const std::vector<size_t>& nc2FinalStock,
			const constraint2_vector_array_t& HvFinalStock,
			scalar_t& totalCost) override;

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
	 * @param [in] threadId: Working thread.
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
	 * Runs the initialization method for single thread SLQ.
	 */
	void runInit() override;

	/**
	 * Runs a single iteration of single thread SLQ.
	 *
	 * @param [in] SmFinal: The final Sm for Riccati equation.
	 * @param [in] SvFinal: The final Sv for Riccati equation.
	 * @param [in] sFinal: The final s for Riccati equation.
	 */
	void runIteration(const state_matrix_t& SmFinal = state_matrix_t::Zero(),
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

	/**
	 * Gets a pointer to the array of the subsystem dynamics.
	 * @return Pointer to the array of the subsystem dynamics.
	 */
	std::vector<std::shared_ptr<ControlledSystemBase<STATE_DIM, INPUT_DIM>> >& getSubsystemDynamicsPtrStock() override {
		return dynamics_[BASE::options_.nThreads_];
	}

	/**
	 * Gets a nominal state of subsystem cost in the given index.
	 *
	 * @param [in] index: The requested index.
	 * @param [out] timeTrajectory: The time stamp tarjectory for the requested subsystem.
	 * @param [out] stateTrajectory: The state tarjectory for the requested subsystem.
	 */
	void getSingleCostNominalState(size_t index, scalar_array_t& timeTrajectory,
			state_vector_array_t& stateTrajectory) const override;

protected:
	/**
	 * Sets a nominal state of subsystem cost in the given index.
	 *
	 * @param [in] index: The requested index.
	 * @param [in] timeTrajectory: The time stamp tarjectory for the requested subsystem.
	 * @param [in] stateTrajectory: The state tarjectory for the requested subsystem.
	 */
	void setSingleCostNominalState(size_t index, const scalar_array_t& timeTrajectory,
			const state_vector_array_t& stateTrajectory) override;

	/**
	 * Solves Riccati equations for all the susbsystems in the systemStockIndexes.
	 *
	 * @param [in] learningRate: The optimal learning rate from line search scheme.
	 * @param [in] SmFinal: The final Sm for Riccati equation.
	 * @param [in] SvFinal: The final Sv for Riccati equation.
	 * @param [in] sFinal: The final s for Riccati equation.
	 */
	void solveSequentialRiccatiEquations(const scalar_t& learningRate,
			const state_matrix_t& SmFinal,
			const state_vector_t& SvFinal,
			const eigen_scalar_t& sFinal) override;

	/**
	 * Approximates the nonlinear problem as a linear-quadratic problem around the nominal
	 * state and control trajectories. This method updates the following variables:
	 * 	- linearized system model
	 * 	- \f$ dxdt = A_m(t)x + B_m(t)u \f$.
	 * 	- s.t. \f$ C_m(t)x + D_m(t)u + E_v(t) = 0 \f$.
	 * 	- AmTrajectoryStock_: \f$ A_m\f$  matrix.
	 * 	- BmTrajectoryStock_: \f$ B_m\f$  matrix.
	 * 	- CmTrajectoryStock_: \f$ C_m\f$ matrix.
	 * 	- DmTrajectoryStock_: \f$ D_m\f$ matrix.
	 * 	- EvTrajectoryStock_: \f$ E_v\f$ vector.
	 *
	 * 	- quadratized intermediate cost function
	 * 	- intermediate cost: \f$ q(t) + 0.5 xQ_m(t)x + x'Q_v(t) + u'P_m(t)x + 0.5u'R_m(t)u + u'R_v(t) \f$
	 * 	- qTrajectoryStock_:  \f$ q\f$
	 * 	- QvTrajectoryStock_: \f$ Q_v\f$ vector.
	 * 	- QmTrajectoryStock_:\f$  Q_m\f$ matrix.
	 * 	- PmTrajectoryStock_: \f$ P_m\f$ matrix.
	 * 	- RvTrajectoryStock_: \f$ R_v\f$ vector.
	 * 	- RmTrajectoryStock_: \f$ R_m\f$ matrix.
	 * 	- RmInverseTrajectoryStock_: inverse of \f$ R_m\f$ matrix.
	 *
	 * 	- as well as the constrained coefficients of
	 * 		- linearized system model
	 * 		- quadratized intermediate cost function
	 * 		- quadratized final cost
	 *
	 */
	void approximateOptimalControlProblem();

	/**
	 * Calculates the controller and linear function approximation of the type-1 constraint Lagrangian. \n
	 * This method uses the following variables:
	 * - constrained, linearized model
	 * - constrained, quadratized cost \n
	 *
	 * The method outputs:
	 * - BASE::nominalControllersStock_: the controller that stabilizes the system around the new nominal trajectory and
	 * 								improves the constraints as well as the increment to the feedforward control input.
	 */
	void calculateController();

	/**
	 * Line search on the feedforwrd parts of the controller and lagrange multipliers.
	 * Based on the option flag lineSearchByMeritFuntion_ it uses two different approaches for line search:
	 * 	- The constraint correction term is added by a user defined stepSize.
	 * 	- The line search uses the merit function for choosing the best stepSize.
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

	/**
	 * Computes the linearized dynamics for a particular subsystem
	 * @param [in] sysIndex
	 */
	void approximateSubsystemLQ(const size_t sysIndex); // computes the linearized dynamics for a particular subsystem

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
