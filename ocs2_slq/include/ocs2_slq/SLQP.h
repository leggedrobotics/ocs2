/*
 * SLQP.h
 *
 *  Created on: Jun 16, 2016
 *      Author: farbod
 */

#ifndef SLQP_OCS2_H_
#define SLQP_OCS2_H_


#include "ocs2_slq/SLQP_BASE.h"


namespace ocs2{

/**
 * This class implements single thread SLQ algorithm.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 */
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T=NullLogicRules<STATE_DIM,INPUT_DIM>>
class SLQP : public SLQP_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef SLQP_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>	BASE;
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
	SLQP(const typename controlled_system_base_t::Ptr& subsystemDynamicsPtr,
			const typename derivatives_base_t::Ptr& subsystemDerivativesPtr,
			const typename constraint_base_t::Ptr& subsystemConstraintPtr,
			const typename cost_function_base_t::Ptr& subsystemCostFunctionsPtr,
			const typename operating_trajectories_base_t::Ptr& operatingTrajectoriesPtr,
			const Options_t& options = Options_t::Options(),
			const LOGIC_RULES_T& logicRules = LOGIC_RULES_T())

	: BASE(subsystemDynamicsPtr, subsystemDerivativesPtr, subsystemConstraintPtr, subsystemCostFunctionsPtr, operatingTrajectoriesPtr, options, logicRules)
	{}

	/**
	 * Default destructor.
	 */
	virtual ~SLQP();

	/**
	 * Forward integrate the system dynamics with given controller. It uses the given control policies and initial state,
	 * to integrate the system dynamics in time period [initTime, finalTime].
	 *
	 * @param [in] initTime: Initial time.
	 * @param [in] initState: Initial state.
	 * @param [in] finalTime: Final time.
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
			const controller_array_t& controllersStock,
			std::vector<scalar_array_t>& timeTrajectoriesStock,
			std::vector<size_array_t>& eventsPastTheEndIndecesStock,
			state_vector_array2_t& stateTrajectoriesStock,
			control_vector_array2_t& inputTrajectoriesStock,
			size_t threadId = 0) override;

	/**
	 * The interface class for constraints. It uses the given rollout trajectories and calculate the constraints.
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
	 */
	virtual void calculateRolloutConstraints(const std::vector<scalar_array_t>& timeTrajectoriesStock,
			const std::vector<size_array_t>& eventsPastTheEndIndecesStock,
			const state_vector_array2_t& stateTrajectoriesStock,
			const control_vector_array2_t& inputTrajectoriesStock,
			std::vector<size_array_t>& nc1TrajectoriesStock,
			constraint1_vector_array2_t& EvTrajectoryStock,
			std::vector<size_array_t>& nc2TrajectoriesStock,
			constraint2_vector_array2_t& HvTrajectoryStock,
			std::vector<size_array_t>& nc2FinalValuesStock,
			constraint2_vector_array2_t& HvFinalStock) override;

	/**
	 * Calculates cost of a roll-out.
	 *
	 * @param [in] timeTrajectoriesStock: Array of trajectories containing the time trajectory stamp of a roll-out.
	 * @param [in] eventsPastTheEndIndecesStock: Array of indices containing past-the-end index of events trigger.
	 * @param [in] stateTrajectoriesStock: Array of trajectories containing the state trajectory of a roll-out.
	 * @param [in] inputTrajectoriesStock: Array of trajectories containing the control input trajectory of a roll-out.
	 * @param [out] totalCost: The total cost of the roll-out.
	 */
	void calculateRolloutCost(const std::vector<scalar_array_t>& timeTrajectoriesStock,
			const std::vector<size_array_t>& eventsPastTheEndIndecesStock,
			const state_vector_array2_t& stateTrajectoriesStock,
			const control_vector_array2_t& inputTrajectoriesStock,
			scalar_t& totalCost) override;

	/**
	 * Calculates the cost function plus penalty for state-only constraints of a roll-out.
	 *
	 * @param [in] timeTrajectoriesStock: Array of trajectories containing the time trajectory stamp of a roll-out.
	 * @param [in] eventsPastTheEndIndecesStock: Array of indices containing past-the-end index of events trigger.
	 * @param [in] stateTrajectoriesStock: Array of trajectories containing the state trajectory of a roll-out.
	 * @param [in] inputTrajectoriesStock: Array of trajectories containing the control input trajectory of a roll-out.
	 * @param [out] nc2TrajectoriesStock: Array of trajectories containing the number of the active state-only constraints.
	 * @param [out] HvTrajectoryStock: Array of trajectories containing the value of the state-only constraints.
	 * @param [out] nc2FinalStock: Array containing the number of the active final state-only constraints.
	 * @param [out] HvFinalStock: Array containing the value of the final state-only constraints.
	 * @param [out] totalCost: The total cost plus state-only constraints penalty.
	 */
	void calculateRolloutCost(const std::vector<scalar_array_t>& timeTrajectoriesStock,
			const std::vector<size_array_t>& eventsPastTheEndIndecesStock,
			const state_vector_array2_t& stateTrajectoriesStock,
			const control_vector_array2_t& inputTrajectoriesStock,
			const std::vector<size_array_t>& nc2TrajectoriesStock,
			const constraint2_vector_array2_t& HvTrajectoryStock,
			const std::vector<size_array_t>& nc2FinalStock,
			const constraint2_vector_array2_t& HvFinalStock,
			scalar_t& totalCost) override;

	/**
	 * Sets up the optimizer
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
	 * Solves Riccati equations for all the subsystems.
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
	 * Computes the linearized dynamics for a particular subsystem
	 * @param [in] sysIndex
	 */
	void approximatePartitionLQ(const size_t& sysIndex) override;

	/**
	 * Calculates the controller. This method uses the following variables:
	 * - constrained, linearized model
	 * - constrained, quadratized cost \n
	 *
	 * The method modifies:
	 * - nominalControllersStock_: the controller that stabilizes the system around the new nominal trajectory and
	 * 								improves the constraints as well as the increment to the feedforward control input.
	 */
	void calculateController();

	/**
	 * Line search on the feedforwrd parts of the controller and lagrange multipliers.
	 * Based on the option flag lineSearchByMeritFuntion_ it uses two different approaches for line search:
	 * 	- The constraint correction term is added by a user defined stepSize.
	 * 	- The line search uses the merit function for choosing the best stepSize.
	 * @param [out] learningRateStar: The optimal learning rate.
	 * @param [in] maxLearningRateStar: The maximum permitted learning rate.
	 */
	void lineSearch(scalar_t& learningRateStar,
			scalar_t maxLearningRateStar=1.0);

private:


public:
	template <size_t GSLQP_STATE_DIM, size_t GSLQP_INPUT_DIM>
	friend class GSLQP;
};

} // namespace ocs2

#include "implementation/SLQP.h"


#endif /* SLQP_H_ */
