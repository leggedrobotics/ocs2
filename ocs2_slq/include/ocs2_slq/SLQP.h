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
 * @tparam LOGIC_RULES_T: Logic Rules type (default NullLogicRules).
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
	 * Line search on the feedforwrd parts of the controller. It uses the following approach for line search:
	 * The constraint TYPE-1 correction term is directly added through a user defined stepSize (defined in options_.constraintStepSize_).
	 * But the cost minimization term is optimized through a line-search strategy defined in SLQ options.
	 *
	 * @param [in] maxLearningRateStar: The maximum permitted learning rate.
	 */
	void lineSearch(scalar_t maxLearningRateStar=1.0) override;

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
	 * Computes the linearized dynamics for a particular time partition
	 * @param [in] sysIndex
	 */
	void approximatePartitionLQ(const size_t& partitionIndex) override;

	/**
	 * Computes the controller for a particular time partition
	 *
	 * @param partitionIndex: Time partition index
	 */
	void calculatePartitionController(const size_t& partitionIndex) override;


private:


public:
	template <size_t GSLQP_STATE_DIM, size_t GSLQP_INPUT_DIM>
	friend class GSLQP;
};

} // namespace ocs2

#include "implementation/SLQP.h"


#endif /* SLQP_H_ */
