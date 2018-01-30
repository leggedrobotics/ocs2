/*
 * SLQP_BASE.h
 *
 *  Created on: August 14, 2016
 *      Author: farbodf
 */

#ifndef SLQP_BASE_OCS2_H_
#define SLQP_BASE_OCS2_H_

#include <array>
#include <algorithm>
#include <numeric>
#include <cstddef>
#include <Eigen/StdVector>
#include <vector>
#include <type_traits>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

#include <ocs2_core/Dimensions.h>
#include <ocs2_core/dynamics/ControlledSystemBase.h>
#include <ocs2_core/dynamics/DerivativesBase.h>
#include <ocs2_core/constraint/ConstraintBase.h>
#include <ocs2_core/cost/CostFunctionBase.h>
#include <ocs2_core/integration/Integrator.h>
#include <ocs2_core/integration/KillIntegrationEventHandler.h>
#include <ocs2_core/misc/LinearInterpolation.h>
#include <ocs2_core/misc/LTI_Equations.h>
#include <ocs2_core/misc/FindActiveIntervalIndex.h>
#include <ocs2_core/logic/LogicRulesBase.h>
#include <ocs2_core/logic/LogicRulesMachine.h>
#include <ocs2_core/initialization/SystemOperatingTrajectoriesBase.h>

#include <ocs2_slq/SequentialRiccatiEquations.h>
#include <ocs2_slq/SequentialRiccatiEquationsNormalized.h>
#include <ocs2_slq/SequentialErrorEquation.h>
#include <ocs2_slq/SequentialErrorEquationNormalized.h>

#include <chrono>

//#define BENCHMARK

namespace ocs2{
///**
// * primary template
// */
//template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T> class SLQP_BASE;

/**
 * This class is an interface class for the single-thread and multi-thread SLQ.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 * @tparam LOGIC_RULES_T: Logic Rules type (default NullLogicRules).
 */
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T=NullLogicRules<STATE_DIM,INPUT_DIM> >
class SLQP_BASE
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	static_assert(std::is_base_of<LogicRulesBase<STATE_DIM,INPUT_DIM>, LOGIC_RULES_T>::value, "LOGIC_RULES_T must inherit from LogicRulesBase");

	typedef std::shared_ptr<SLQP_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>>	Ptr;
//	typedef SequentialRiccatiEquations<STATE_DIM, INPUT_DIM> 			riccati_equations_t;
	typedef SequentialRiccatiEquationsNormalized<STATE_DIM, INPUT_DIM>	riccati_equations_t;
//	typedef SequentialErrorEquation<STATE_DIM, INPUT_DIM>			 	error_equation_t;
	typedef SequentialErrorEquationNormalized<STATE_DIM, INPUT_DIM> 	error_equation_t;
//	typedef LTI_Equations<STATE_DIM> LTI_Equation_t;

	typedef Dimensions<STATE_DIM, INPUT_DIM> DIMENSIONS;

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

	typedef ControlledSystemBase<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>	controlled_system_base_t;
	typedef KillIntegrationEventHandler<STATE_DIM>						event_handler_t;
	typedef DerivativesBase<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>		derivatives_base_t;
	typedef ConstraintBase<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>			constraint_base_t;
	typedef CostFunctionBase<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>		cost_function_base_t;
	typedef SystemOperatingTrajectoriesBase<STATE_DIM, INPUT_DIM, LOGIC_RULES_T> operating_trajectories_base_t;

	/**
	 * Default constructor.
	 */
	SLQP_BASE()
	: numPartitionings_(0)
	{}

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
	SLQP_BASE(const typename controlled_system_base_t::Ptr& subsystemDynamicsPtr,
			  const typename derivatives_base_t::Ptr& subsystemDerivativesPtr,
			  const typename constraint_base_t::Ptr& subsystemConstraintPtr,
			  const typename cost_function_base_t::Ptr& subsystemCostFunctionsPtr,
			  const typename operating_trajectories_base_t::Ptr& operatingTrajectoriesPtr,
			  const Options_t& options = Options_t::Options(),
			  const LOGIC_RULES_T& logicRules = LOGIC_RULES_T());

	/**
	 * Default destructor.
	 */
	virtual ~SLQP_BASE() {}

	/**
	 * Forward integrate the system dynamics with given controller. It uses the given control policies and initial state,
	 * to integrate the system dynamics in time period [initTime, finalTime].
	 *
	 * @param [in] initTime: Initial time.
	 * @param [in] initState: Initial state.
	 * @param [in] finalTime: Final time.
	 * @param [in] partitioningTimes: Time partitioning
	 * @param [in] controllersStock: Array of control policies.
	 * @param [out] timeTrajectoriesStock: Array of trajectories containing the output time trajectory stamp.
	 * @param [out] eventsPastTheEndIndecesStock: Array of indices containing past-the-end index of events trigger.
	 * @param [out] stateTrajectoriesStock: Array of trajectories containing the output state trajectory.
	 * @param [out] inputTrajectoriesStock: Array of trajectories containing the output control input trajectory.
	 * @param [in] threadId: Working thread (default is 0).
	 */
	void rolloutTrajectory(const scalar_t& initTime,
			const state_vector_t& initState,
			const scalar_t& finalTime,
			const scalar_array_t& partitioningTimes,
			const controller_array_t& controllersStock,
			std::vector<scalar_array_t>& timeTrajectoriesStock,
			std::vector<size_array_t>& eventsPastTheEndIndecesStock,
			state_vector_array2_t& stateTrajectoriesStock,
			control_vector_array2_t& inputTrajectoriesStock,
			size_t threadId = 0);

	/**
	 * The class for performing roll-out. It uses the given control policies and initial state,
	 * to integrate the system dynamics in time period [initTime, finalTime] and only return the final state.
	 *
	 * @param [in] initTime: Initial time.
	 * @param [in] initState: Initial state.
	 * @param [in] finalTime: Final time.
	 * @param [in] partitioningTimes: Time partitioning
	 * @param [in] controllersStock: Array of control policies.
	 * @param [out] finalState: Final state.
	 * @param [out] finalInput: Final control input.
	 * @param [out] finalActiveSubsystemIndex: The final active subsystem.
	 * @param [in] threadId: Working thread (default is 0).
	 */
	void rolloutFinalState(const scalar_t& initTime,
			const state_vector_t& initState,
			const scalar_t& finalTime,
			const scalar_array_t& partitioningTimes,
			const controller_array_t& controllersStock,
			state_vector_t& finalState,
			control_vector_t& finalInput,
			size_t& finalActiveSubsystemIndex,
			size_t threadId = 0);

	/**
	 * Calculates a rollout constraints. It uses the given rollout trajectories and calculate the constraints.
	 *
	 * @param [in] timeTrajectoriesStock: Array of trajectories containing the output time trajectory stamp.
	 * @param [in] eventsPastTheEndIndecesStock: Array of indices containing past-the-end index of events trigger.
	 * @param [in] stateTrajectoriesStock: Array of trajectories containing the output state trajectory.
	 * @param [in] inputTrajectoriesStock: Array of trajectories containing the output control input trajectory.
	 * @param [out] nc1TrajectoriesStock: Array of trajectories containing the number of the active state-input constraints.
	 * @param [out] EvTrajectoryStock: Array of trajectories containing the value of the state-input constraints (if the roll-out is constrained the value is
	 * always zero otherwise it is nonzero).
	 * @param [out] nc2TrajectoriesStock: Array of trajectories containing the number of the active state-only constraints.
	 * @param [out] HvTrajectoryStock: Array of trajectories containing the value of the state-only constraints.
	 * @param [out] nc2FinalStock: Array containing the number of the active final state-only constraints.
	 * @param [out] HvFinalStock: Array containing the value of the final state-only constraints.
	 * @param [in] threadId: Working thread (default is 0).
	 */
	void calculateRolloutConstraints(
			const std::vector<scalar_array_t>& timeTrajectoriesStock,
			const std::vector<size_array_t>& eventsPastTheEndIndecesStock,
			const state_vector_array2_t& stateTrajectoriesStock,
			const control_vector_array2_t& inputTrajectoriesStock,
			std::vector<size_array_t>& nc1TrajectoriesStock,
			constraint1_vector_array2_t& EvTrajectoryStock,
			std::vector<size_array_t>& nc2TrajectoriesStock,
			constraint2_vector_array2_t& HvTrajectoryStock,
			std::vector<size_array_t>& nc2FinalStock,
			constraint2_vector_array2_t& HvFinalStock,
			size_t threadId = 0);

	/**
	 * Calculates cost of a roll-out.
	 *
	 * @param [in] threadId: Working thread.
	 * @param [in] timeTrajectoriesStock: Array of trajectories containing the time trajectory stamp of a roll-out.
	 * @param [in] eventsPastTheEndIndecesStock: Array of indices containing past-the-end index of events trigger.
	 * @param [in] stateTrajectoriesStock: Array of trajectories containing the state trajectory of a roll-out.
	 * @param [in] inputTrajectoriesStock: Array of trajectories containing the control input trajectory of a roll-out.
	 * @param [out] totalCost: The total cost of the roll-out.
	 * @param [in] threadId: Working thread (default is 0).
	 */
	void calculateRolloutCost(
			const std::vector<scalar_array_t>& timeTrajectoriesStock,
			const std::vector<size_array_t>& eventsPastTheEndIndecesStock,
			const state_vector_array2_t& stateTrajectoriesStock,
			const control_vector_array2_t& inputTrajectoriesStock,
			scalar_t& totalCost,
			size_t threadId = 0);

	/**
	 * Calculates the cost function plus penalty for state-only constraints of a roll-out.
	 *
	 * @param [in] threadId: Working thread.
	 * @param [in] timeTrajectoriesStock: Array of trajectories containing the time trajectory stamp of a roll-out.
	 * @param [in] eventsPastTheEndIndecesStock: Array of indices containing past-the-end index of events trigger.
	 * @param [in] stateTrajectoriesStock: Array of trajectories containing the state trajectory of a roll-out.
	 * @param [in] inputTrajectoriesStock: Array of trajectories containing the control input trajectory of a roll-out.
	 * @param [out] nc2TrajectoriesStock: Array of trajectories containing the number of the active state-only constraints.
	 * @param [out] HvTrajectoryStock: Array of trajectories containing the value of the state-only constraints.
	 * @param [out] nc2FinalStock: Array containing the number of the active final state-only constraints.
	 * @param [out] HvFinalStock: Array containing the value of the final state-only constraints.
	 * @param [out] totalCost: The total cost plus state-only constraints penalty.
	 * @param [in] threadId: Working thread (default is 0).
	 */
	void calculateRolloutCost(
			const std::vector<scalar_array_t>& timeTrajectoriesStock,
			const std::vector<size_array_t>& eventsPastTheEndIndecesStock,
			const state_vector_array2_t& stateTrajectoriesStock,
			const control_vector_array2_t& inputTrajectoriesStock,
			const std::vector<size_array_t>& nc2TrajectoriesStock,
			const constraint2_vector_array2_t& HvTrajectoryStock,
			const std::vector<size_array_t>& nc2FinalStock,
			const constraint2_vector_array2_t& HvFinalStock,
			scalar_t& totalCost,
			size_t threadId = 0);

	/**
	 * Approximates the nonlinear problem as a linear-quadratic problem around the nominal
	 * state and control trajectories. This method updates the following variables:
	 * 	- linearized system model and constraints
	 * 	- \f$ dxdt = A_m(t)x + B_m(t)u \f$.
	 * 	- s.t. \f$ C_m(t)x + D_m(t)u + E_v(t) = 0 \f$ \\
	 * 	-      \f$ F_m(t)x + H_v(t) = 0 \f$ .
	 * 	- AmTrajectoryStock_: \f$ A_m\f$  matrix.
	 * 	- BmTrajectoryStock_: \f$ B_m\f$  matrix.
	 * 	- CmTrajectoryStock_: \f$ C_m\f$ matrix.
	 * 	- DmTrajectoryStock_: \f$ D_m\f$ matrix.
	 * 	- EvTrajectoryStock_: \f$ E_v\f$ vector.
	 * 	- FmTrajectoryStock_: \f$ F_m\f$ vector.
	 * 	- HvTrajectoryStock_: \f$ H_v\f$ vector.
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
	 * Calculates the controller. This method uses the following variables:
	 * - constrained, linearized model
	 * - constrained, quadratized cost
	 *
	 * The method modifies:
	 * - nominalControllersStock_: the controller that stabilizes the system around the new nominal trajectory and
	 * 								improves the constraints as well as the increment to the feed-forward control input.
	 */
	void calculateController();

	/**
	 * Line search on the feedforwrd parts of the controller. It uses the following approach for line search:
	 * The constraint TYPE-1 correction term is directly added through a user defined stepSize (defined in options_.constraintStepSize_).
	 * But the cost minimization term is optimized through a line-search strategy defined in SLQ options.
	 *
	 * @param [in] maxLearningRateStar: The maximum permitted learning rate.
	 */
	virtual void lineSearch(scalar_t maxLearningRateStar=1.0) = 0;

	/**
	 * Solves Riccati equations for all the partitions.
	 *
	 * @param [in] learningRate: The optimal learning rate from line search scheme.
	 * @param [in] SmFinal: The final Sm for Riccati equation.
	 * @param [in] SvFinal: The final Sv for Riccati equation.
	 * @param [in] sFinal: The final s for Riccati equation.
	 */
	virtual void solveSequentialRiccatiEquations(
			const scalar_t& learningRate,
			const state_matrix_t& SmFinal,
			const state_vector_t& SvFinal,
			const eigen_scalar_t& sFinal) = 0;

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
	virtual void runIteration(const scalar_t& maxLearningRateStar,
			const state_matrix_t& SmFinal  = state_matrix_t::Zero(),
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
	 * The main routine of SLQ which runs SLQP for a given state, time, subsystem arrangement, and switching times.
	 *
	 * @param [in] initTime: Initial time.
	 * @param [in] initState: Initial state.
	 * @param [in] finalTime: Final time.
	 * @param [in] partitioningTimes: The partitioning times between subsystems.
	 * @param [in] controllersStock: Array of the initial control policies. If it is provided as a empty array, SLQ calculates internally the initial policies.
	 * @param [in] desiredTimeTrajectoriesStock: The time stamp trajectory for each subsystem's cost.
	 * @param [in] desiredStateTrajectoriesStock: The state trajectory for each subsystem's cost.
	 */
	void run(const double& initTime,
			const state_vector_t& initState,
			const double& finalTime,
			const std::vector<scalar_t>& partitioningTimes,
			const controller_array_t& controllersStock=controller_array_t(),
			const std::vector<scalar_array_t>& desiredTimeTrajectoriesStock = std::vector<scalar_array_t>(),
			const state_vector_array2_t& desiredStateTrajectoriesStock = state_vector_array2_t());

	/**
	 * set logic rules.
	 * @param logicRules: This class will be passed to all of the dynamics and derivatives classes through initializeModel() routine.
	 */
	void setLogicRules(const LOGIC_RULES_T& logicRules);

	/**
	 * Calculates the value function at the given time and state.
	 *
	 * @param [in] time: The inquiry time
	 * @param [in] state: The inquiry state.
	 * @param [out] valueFuntion: value function at the inquiry time and state.
	 */
	virtual void getValueFuntion(const scalar_t& time, const state_vector_t& state, scalar_t& valueFuntion);

	/**
	 * Gets the cost function and ISE of the constraints at the initial time.
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
	 * Uses Disjoint Riccati approach which effective makes parallelization possible for the backward pass of the SLQ.
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
	 * Truncates the internal array of the control policies based on the initTime.
	 *
	 * @param [in] partitioningTimes: Switching times.
	 * @param [in] initTime: Initial time.
	 * @param [out] controllersStock: Truncated array of the control policies.
	 * @param [out] initActiveSubsystemIndex: Initial active subsystems.
	 * @param [out] deletedcontrollersStock: The deleted part of the control policies.
	 */
	void truncateConterller(const scalar_array_t& partitioningTimes,
			const double& initTime,
			controller_array_t& controllersStock,
			size_t& initActiveSubsystemIndex,
			controller_array_t& deletedcontrollersStock);

	/**
	 * Sets switching times.
	 *
	 * @param [in] switchingTimes: Switching times.
	 */
//	void setSwitchingTimes(const scalar_array_t& switchingTimes) {switchingTimes_ = switchingTimes;}

	/**
	 * Gets switching times
	 *
	 * @param [out] switchingTimes: Switching times.
	 */
//	void getSwitchingTimes(scalar_array_t& switchingTimes) const {switchingTimes = switchingTimes_;}

	/**
	 * Gets number of iterations.
	 *
	 * @return Number of iterations.
	 */
	size_t getNumIterations() const {return iteration_;}

	/**
	 * Gets nominal state of subsystem costs.
	 *
	 * @param [out] timeTrajectoryStock: The time stamp trajectory for each subsystem's cost.
	 * @param [out] stateTrajectoryStock: The state trajectory for each subsystem's cost.
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

	/**
	 * Sets up optimizer for different number of partitionings.
	 *
	 * @param [in] numPartitionings: number of partitionings
	 */
	virtual void setupOptimizer(const size_t& numPartitionings);

protected:
	/**
	 * Computes the linearized dynamics for a particular time partition
	 *
	 * @param [in] partitionIndex: Time partition index
	 */
	virtual void approximatePartitionLQ(const size_t& partitionIndex) = 0;

	/**
	 * Computes the controller for a particular time partition
	 *
	 * @param partitionIndex: Time partition index
	 */
	virtual void calculatePartitionController(const size_t& partitionIndex) = 0;

	/**
	 * Forward integrate the system dynamics with given controller. It uses the given control policies and initial state,
	 * to integrate the system dynamics in time period [initTime, finalTime].
	 *
	 * @param [in] workerIndex: Working agent index.
	 * @param [in] partitionIndex: Time partition index.
	 * @param [in] initTime: Initial time.
	 * @param [in] initState: Initial state.
	 * @param [in] finalTime: Final time.
	 * @param [in] controller: control policies.
	 * @param [out] timeTrajectory: The time trajectory stamp.
	 * @param [out] eventsPastTheEndIndeces: Indices containing past-the-end index of events trigger.
	 * @param [out] stateTrajectory: The state trajectory.
	 * @param [out] inputTrajectory: The control input trajectory.
	 */
	void rolloutWorker(size_t workerIndex,
			const size_t& partitionIndex,
			const scalar_t& initTime,
			const state_vector_t& initState,
			const scalar_t& finalTime,
			const controller_t& controller,
			const scalar_array_t& eventTimes,
			scalar_array_t& timeTrajectory,
			size_array_t& eventsPastTheEndIndeces,
			state_vector_array_t& stateTrajectory,
			control_vector_array_t& inputTrajectory);

	/**
	 * Calculates the total cost for the given trajectories.
	 *
	 * @param [in] workerIndex: Working agent index.
	 * @param [in] partitionIndex: Time partition index.
	 * @param [in] timeTrajectory: The time trajectory stamp.
	 * @param [in] eventsPastTheEndIndeces: Indices containing past-the-end index of events trigger.
	 * @param [in] stateTrajectory: The state trajectory.
	 * @param [in] inputTrajectory: The control input trajectory.
	 * @param [out] totalCost: The total cost.
	 */
	void calculateCostWorker(size_t workerIndex,
			const size_t& partitionIndex,
			const scalar_array_t& timeTrajectory,
			const size_array_t& eventsPastTheEndIndeces,
			const state_vector_array_t& stateTrajectory,
			const control_vector_array_t& inputTrajectory,
			scalar_t& totalCost);

	/**
	 * Calculates the constraint trajectories over the given trajectories.
	 *
	 * @param [in] workerIndex: Working agent index.
	 * @param [in] partitionIndex: Time partition index.
	 * @param [in] timeTrajectory: The time trajectory stamp.
	 * @param [in] eventsPastTheEndIndeces: Indices containing past-the-end index of events trigger.
	 * @param [in] stateTrajectory: The state trajectory.
	 * @param [in] inputTrajectory: The control input trajectory.
	 * @param [out] nc1Trajectory: Trajectory containing number of active type-1 constraints.
	 * @param [out] EvTrajectory: Type-1 constraints trajectory.
	 * @param [out] nc2Trajectory: Trajectory containing number of active type-2 constraints.
	 * @param [out] HvTrajectory: Type-2 constraints trajectory.
	 * @param [out] nc2Finals: Number of active final type-2 constraints.
	 * @param [out] HvFinals: Final type-2 constraints.
	 */
	void calculateConstraintsWorker(size_t workerIndex,
			const size_t& partitionIndex,
			const scalar_array_t& timeTrajectory,
			const size_array_t& eventsPastTheEndIndeces,
			const state_vector_array_t& stateTrajectory,
			const control_vector_array_t& inputTrajectory,
			size_array_t& nc1Trajectory,
			constraint1_vector_array_t& EvTrajectory,
			size_array_t& nc2Trajectory,
			constraint2_vector_array_t& HvTrajectory,
			size_array_t& nc2Finals,
			constraint2_vector_array_t& HvFinals);

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
	 * Calculates controller at a given partition and a node.
	 *
	 * @param [in] workerIndex: Working agent index.
	 * @param [in] partitionIndex: Time partition index
	 * @param [in] timeIndex: Time index in the partition
	 * @param [in] constraintStepSize: type-1 constraint step-size
	 */
	void calculateControllerWorker(size_t workerIndex,
			const size_t& partitionIndex,
			const size_t& timeIndex,
			const scalar_t& constraintStepSize);

	/**
	 * Line search with a specific learning rate.
	 *
	 * @param [in] workerIndex
	 * @param [in] learningRate
	 * @param [out] lsTotalCost
	 * @param [out] lsControllersStock
	 * @param [out] lsTimeTrajectoriesStock
	 * @param [out] lsEventsPastTheEndIndecesStock
	 * @param [out] lsStateTrajectoriesStock
	 * @param [out] lsInputTrajectoriesStock
	 * @param [out] lsNc1TrajectoriesStock
	 * @param [out] lsEvTrajectoryStock
	 * @param [out] lsNc2TrajectoriesStock
	 * @param [out] lsHvTrajectoryStock
	 * @param [out] lsNc2FinalStock
	 * @param [out] lsHvFinalStock
	 */
	void lineSearchWorker(size_t workerIndex,
			scalar_t learningRate,
			scalar_t& lsTotalCost,
			controller_array_t& lsControllersStock,
			std::vector<scalar_array_t>& lsTimeTrajectoriesStock,
			std::vector<size_array_t>& lsEventsPastTheEndIndecesStock,
			state_vector_array2_t& lsStateTrajectoriesStock,
			control_vector_array2_t& lsInputTrajectoriesStock,
			std::vector<size_array_t>& lsNc1TrajectoriesStock,
			constraint1_vector_array2_t& lsEvTrajectoryStock,
			std::vector<size_array_t>& lsNc2TrajectoriesStock,
			constraint2_vector_array2_t& lsHvTrajectoryStock,
			std::vector<size_array_t>& lsNc2FinalStock,
			constraint2_vector_array2_t& lsHvFinalStock);

	/**
	 * Solves a set of Riccati equations for the partition in the given index.
	 *
	 * @param [in] workerIndex: Working agent index.
	 * @param [in] partitionIndex: The requested partition index to solve Riccati equations.
	 * @param [in] learningRate: The optimal learning rate from line search scheme.
	 * @param [in] SmFinal: The final Sm for Riccati equation.
	 * @param [in] SvFinal: The final Sv for Riccati equation.
	 * @param [in] sFinal: The final s for Riccati equation.
	 */
	void solveRiccatiEquationsWorker(
			size_t workerIndex,
			const size_t& partitionIndex,
			const scalar_t& learningRate,
			const state_matrix_t& SmFinal,
			const state_vector_t& SvFinal,
			const eigen_scalar_t& sFinal);

	/**
	 * Solves a set of Riccati equations for the partition in the given index with given time trajectory stamp.
	 *
	 * @param [in] workerIndex: Working agent index.
	 * @param [in] partitionIndex: The requested partition index to solve Riccati equations.
	 * @param [in] learningRate: The optimal learning rate from line search scheme.
	 * @param [in] nominalTimeTrajectory: The input array of the time trajectories.
	 * @param [in] nominalEventsPastTheEndIndeces: past_the_end indeces of events on nominalTimeTrajectory time vector
	 * @param [in] SmFinal: The final Sm for the current Riccati equation.
	 * @param [in] SvFinal: The final Sv for the current Riccati equation.
	 * @param [in] sFinal: The final s for the current Riccati equation.
	 */
	void solveRiccatiEquationsWorker(
			size_t workerIndex,
			const size_t& partitionIndex,
			const scalar_t& learningRate,
			const scalar_array_t& nominalTimeTrajectory,
			const size_array_t& nominalEventsPastTheEndIndeces,
			const state_matrix_t& SmFinal,
			const state_vector_t& SvFinal,
			const eigen_scalar_t& sFinal);

	/**
	 * Type_1 constraints error correction compensation which solves a set of erro Riccati equations for the partition in the given index.
	 *
	 * @param [in] workerIndex: Working agent index.
	 * @param [in] partitionIndex: The requested partition index to solve Riccati equations.
	 * @param [in] SveFinal: The final Sve for the current Riccati equation.
	 */
	void solveErrorRiccatiEquationWorker(
			size_t workerIndex,
			const size_t& partitionIndex,
			const state_vector_t& SveFinal);

	/**
	 * Full Backward Sweep method uses exp method instead of ode to solve Riccati equations.
	 *
	 * @param [in] workerIndex: Working agent index.
	 * @param [in] partitionIndex: The requested partition index to solve Riccati equations.
	 * @param [in] SmFinal: The final Sm for the Riccati equation.
	 * @param [in] SvFinal: The final Sv for the Riccati equation.
	 * @param [in] SveFinal: The final Sve for the Riccati equation.
	 * @param [in] sFinal: The final s for the Riccati equation.
	 * @param [in] constraintStepSize: type-1 constraint step-size
	 */
	void fullRiccatiBackwardSweepWorker(
			size_t workerIndex,
			const size_t& partitionIndex,
			const state_matrix_t& SmFinal, const state_vector_t& SvFinal,
			const state_vector_t& SveFinal, const eigen_scalar_t& sFinal,
			const scalar_t& constraintStepSize);

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
	 * Computes the Lagrage multiplier over the given roll-out.
	 *
	 * @param [in] timeTrajectoriesStock: roll-out simulated time steps
	 * @param [in] stateTrajectoriesStock: roll-out outputs
	 * @param [in] lagrangeMultiplierFunctionsStock: the coefficients of the linear function for lagrangeMultiplier
	 * @param [out] lagrangeTrajectoriesStock: lagrangeMultiplier value over the given trajectory
	 */
	void calculateRolloutLagrangeMultiplier(const std::vector<scalar_array_t>& timeTrajectoriesStock,
			const state_vector_array2_t& stateTrajectoriesStock,
			const std::vector<lagrange_t>& lagrangeMultiplierFunctionsStock,
			std::vector<std::vector<Eigen::VectorXd>>& lagrangeTrajectoriesStock);

	/**
	 * Computes the co-state over the given roll-out.
	 *
	 * @param [in] timeTrajectoriesStock: roll-out simulated time steps
	 * @param [in] stateTrajectoriesStock: roll-out outputs
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
	 * compute the merit function for given roll-out
	 *
	 * @param [in] timeTrajectoriesStock: simulation time trajectory
	 * @param [in] nc1TrajectoriesStock: roll-out's number of active constraints in each time step
	 * @param [in] EvTrajectoryStock: roll-out's constraints value
	 * @param [in] lagrangeTrajectoriesStock: constraint Lagrange multiplier for the given roll-out
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
	 * Sets nominal state of subsystem costs.
	 *
	 * @param [in] timeTrajectoryStock: The time stamp trajectory for each subsystem's cost.
	 * @param [in] stateTrajectoryStock: The state trajectory for each subsystem's cost.
	 */
	void setCostNominalStates(const std::vector<scalar_array_t>& timeTrajectoryStock,
			const state_vector_array2_t& stateTrajectoryStock);

	/**
	 * Makes the matrix PSD.
	 * @tparam Derived type.
	 * @param [out] squareMatrix: The matrix to become PSD.
	 * @return boolean:
	 */
	template <typename Derived>
	bool makePSD(Eigen::MatrixBase<Derived>& squareMatrix);

	/**
	 * for nice debug printing
	 * @param [in] text
	 */
	void printString(const std::string& text);

	/****************
	 *** Variables **
	 ****************/
	size_t numPartitionings_;
	controller_array_t  nominalControllersStock_;

	// the original OC problems
	typename controlled_system_base_t::Ptr 		subsystemDynamicsPtr_;
	typename derivatives_base_t::Ptr 			subsystemDerivativesPtr_;
	typename constraint_base_t::Ptr				subsystemConstraintPtr_;
	typename cost_function_base_t::Ptr 			subsystemCostFunctionsPtr_;
	typename operating_trajectories_base_t::Ptr operatingTrajectoriesPtr_;

	Options_t options_;

	LogicRulesMachine<STATE_DIM, INPUT_DIM, LOGIC_RULES_T> logicRulesMachine_;

	// the partitions OC problems
	std::vector<typename controlled_system_base_t::Ptr> subsystemDynamicsPtrStock_;
	std::vector<typename derivatives_base_t::Ptr> 		subsystemDerivativesPtrStock_;
	std::vector<typename constraint_base_t::Ptr> 		subsystemConstraintsPtrStock_;
	std::vector<typename cost_function_base_t::Ptr> 	subsystemCostFunctionsPtrStock_;
	std::vector<typename event_handler_t::Ptr>			eventHandlerPtrStock_;
	std::vector<std::shared_ptr<ODE45<STATE_DIM> > > 	integratorODE45StockPtr_;
	std::vector<typename operating_trajectories_base_t::Ptr> operatingTrajectoriesPtrStock_;

	scalar_t nominalTotalCost_;
	scalar_t nominalConstraint1ISE_;
	scalar_t constraint1MaxNorm_;
	scalar_t nominalConstraint2ISE_;
	scalar_t constraint2MaxNorm_;
	scalar_t learningRateStar_ = 1.0;  // The optimal learning rate.
	scalar_t maxLearningRate_  = 1.0;  // The maximum permitted learning rate (options_.maxLearningRateGSLQP_).
	scalar_array_t constraintStepSizesStock_;  // type-1 constraint's step-sizes for different partitions (it is either 0 or options.constraintStepSize_)

	std::vector<scalar_array_t> nominalTimeTrajectoriesStock_;
	std::vector<size_array_t> 	nominalEventsPastTheEndIndecesStock_;
	state_vector_array2_t		nominalStateTrajectoriesStock_;
	control_vector_array2_t  	nominalInputTrajectoriesStock_;
	state_vector_array2_t   	nominalcostateTrajectoriesStock_;

	std::vector<scalar_array_t> desiredTimeTrajectoriesStock_;
	state_vector_array2_t 		desiredStateTrajectoriesStock_;

	state_matrix_array2_t        AmTrajectoryStock_;
	control_gain_matrix_array2_t BmTrajectoryStock_;

	std::vector<std::vector<size_t>>    nc1TrajectoriesStock_;  	// nc1: Number of the Type-1  active constraints
	constraint1_vector_array2_t			EvTrajectoryStock_;
	constraint1_state_matrix_array2_t   CmTrajectoryStock_;
	constraint1_control_matrix_array2_t DmTrajectoryStock_;

	std::vector<size_array_t> 			nc2TrajectoriesStock_;  // nc2: Number of the Type-2 active constraints
	constraint2_vector_array2_t 		HvTrajectoryStock_;
	constraint2_state_matrix_array2_t 	FmTrajectoryStock_;
	std::vector<size_array_t>			nc2FinalStock_;
	constraint2_vector_array2_t			HvFinalStock_;
	constraint2_state_matrix_array2_t 	FmFinalStock_;

	eigen_scalar_array2_t		qFinalStock_;
	state_vector_array2_t		QvFinalStock_;
	state_matrix_array2_t		QmFinalStock_;

	eigen_scalar_array2_t 		qTrajectoryStock_;
	state_vector_array2_t 		QvTrajectoryStock_;
	state_matrix_array2_t 		QmTrajectoryStock_;
	control_vector_array2_t		RvTrajectoryStock_;
	control_matrix_array2_t		RmTrajectoryStock_;
	control_feedback_array2_t	PmTrajectoryStock_;

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

	std::vector<std::shared_ptr<riccati_equations_t>> 							riccatiEquationsPtrStock_;
	std::vector<std::shared_ptr<IntegratorBase<riccati_equations_t::S_DIM_>>> 	riccatiIntegratorPtrStock_;
	std::vector<std::shared_ptr<error_equation_t>> 			errorEquationPtrStock_;
	std::vector<std::shared_ptr<IntegratorBase<STATE_DIM>>> errorIntegratorPtrStock_;

	std::vector<scalar_array_t>	SsTimeTrajectoryStock_;
	std::vector<scalar_array_t> SsNormalizedTimeTrajectoryStock_;
	std::vector<size_array_t> 	SsNormalizedEventsPastTheEndIndecesStock_;
	eigen_scalar_array2_t 		sTrajectoryStock_;
	state_vector_array2_t 		SvTrajectoryStock_;
	state_vector_array2_t 		SveTrajectoryStock_;
	state_matrix_array2_t 		SmTrajectoryStock_;

	eigen_scalar_array_t sFinalStock_;
	state_vector_array_t SvFinalStock_;
	state_vector_array_t SveFinalStock_;
	state_matrix_array_t SmFinalStock_;
	state_vector_array_t xFinalStock_;

	controller_array_t deletedcontrollersStock_;

	scalar_array_t partitioningTimes_;
	scalar_t initTime_;
	scalar_t finalTime_;
	state_vector_t initState_;
	size_t initActiveSubsystem_;
	size_t finalActiveSubsystem_;
	size_t iteration_;

	eigen_scalar_array_t iterationCost_;
	eigen_scalar_array_t iterationISE1_;

	bool isRewinded_ = false;

	bool useDisjointRiccati_ = false;

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
	controller_array_t initLScontrollersStock_;

public:
	template <size_t GSLQP_STATE_DIM, size_t GSLQP_INPUT_DIM>
	friend class GSLQP;
};

} // namespace ocs2

#include "implementation/SLQP_BASE.h"

#endif /* SLQP_H_ */
