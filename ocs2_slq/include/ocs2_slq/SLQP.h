/*
 * SLQP.h
 *
 *  Created on: Jun 16, 2016
 *      Author: farbod
 */

#ifndef SLQP_OCS2_H_
#define SLQP_OCS2_H_


#include <ocs2_core/Dimensions.h>
#include <ocs2_core/integration/KillIntegrationEventHandler.h>

#include "ocs2_slq/SLQP_BASE.h"


namespace ocs2{

/**
 * SLQP class
 * @tparam STATE_DIM
 * @tparam INPUT_DIM
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class SLQP : public SLQP_BASE<STATE_DIM, INPUT_DIM>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef SLQP_BASE<STATE_DIM, INPUT_DIM>	BASE;
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
	 *
	 */
	SLQP(const std::vector<std::shared_ptr<ControlledSystemBase<STATE_DIM, INPUT_DIM> > >& subsystemDynamicsPtr,
			const std::vector<std::shared_ptr<DerivativesBase<STATE_DIM, INPUT_DIM> > >& subsystemDerivativesPtr,
			const std::vector<std::shared_ptr<CostFunctionBaseOCS2<STATE_DIM, INPUT_DIM> > >& subsystemCostFunctionsPtr,
			const Options_t& options = Options_t::Options(),
			const state_vector_array_t& stateOperatingPoints = state_vector_array_t(),
			const control_vector_array_t& inputOperatingPoints = control_vector_array_t() )
	: BASE(subsystemDynamicsPtr, subsystemDerivativesPtr, subsystemCostFunctionsPtr, options, stateOperatingPoints, inputOperatingPoints)
	{}

	virtual ~SLQP();

	/**
	 * Rollout Function
	 * @param initTime
	 * @param initState
	 * @param finalTime
	 * @param controllersStock
	 * @param timeTrajectoriesStock
	 * @param stateTrajectoriesStock
	 * @param inputTrajectoriesStock
	 * @param nc1TrajectoriesStock
	 * @param EvTrajectoryStock
	 * @param nc2TrajectoriesStock
	 * @param HvTrajectoryStock
	 * @param nc2FinalStock
	 * @param HvFinalStock
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
			constraint2_vector_array_t& HvFinalStock) override;

	/**
	 * Rollout Function
	 * @param initTime
	 * @param initState
	 * @param finalTime
	 * @param controllersStock
	 * @param timeTrajectoriesStock
	 * @param stateTrajectoriesStock
	 * @param inputTrajectoriesStock
	 */
	virtual void rollout(const double& initTime,
			const state_vector_t& initState,
			const scalar_t& finalTime,
			const controller_array_t& controllersStock,
			std::vector<scalar_array_t>& timeTrajectoriesStock,
			state_vector_array2_t& stateTrajectoriesStock,
			control_vector_array2_t& inputTrajectoriesStock) override;

	/**
	 * Rollout Function
	 * @param initTime
	 * @param initState
	 * @param finalTime
	 * @param controllersStock
	 * @param finalState
	 * @param finalInput
	 * @param finalActiveSubsystemIndex
	 */
	virtual void rollout(const scalar_t& initTime,
			const state_vector_t& initState,
			const scalar_t& finalTime,
			const controller_array_t& controllersStock,
			state_vector_t& finalState,
			control_vector_t& finalInput,
			size_t& finalActiveSubsystemIndex) override;

	/**
	 * Calculates the cost function
	 * @param timeTrajectoriesStock
	 * @param stateTrajectoriesStock
	 * @param inputTrajectoriesStock
	 * @param totalCost
	 */
	void calculateCostFunction(const std::vector<scalar_array_t>& timeTrajectoriesStock,
			const state_vector_array2_t& stateTrajectoriesStock,
			const control_vector_array2_t& inputTrajectoriesStock,
			scalar_t& totalCost) override;

	/**
	 * Calculates the cost function
	 * @param timeTrajectoriesStock
	 * @param stateTrajectoriesStock
	 * @param inputTrajectoriesStock
	 * @param nc2TrajectoriesStock
	 * @param HvTrajectoryStock
	 * @param nc2FinalStock
	 * @param HvFinalStock
	 * @param totalCost
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
	 * Sets up the optimizer
	 */
	void setupOptimizer() override;

	/**
	 * Runs initialization
	 */
	void runInit() override;

	/**
	 * Runs iteration
	 * @param SmFinal
	 * @param SvFinal
	 * @param sFinal
	 */
	void runIteration(const state_matrix_t& SmFinal = state_matrix_t::Zero(),
			const state_vector_t& SvFinal = state_vector_t::Zero(),
			const eigen_scalar_t& sFinal = eigen_scalar_t::Zero()) override;

	/**
	 * Runs exit
	 * @param SmFinal
	 * @param SvFinal
	 * @param sFinal
	 */
	void runExit(const state_matrix_t& SmFinal = state_matrix_t::Zero(),
			const state_vector_t& SvFinal = state_vector_t::Zero(),
			const eigen_scalar_t& sFinal = eigen_scalar_t::Zero()) override;

	/**
	 * Gets subsystem dynamics pointer
	 */
	virtual std::vector<std::shared_ptr<ControlledSystemBase<STATE_DIM, INPUT_DIM>> >& getSubsystemDynamicsPtrStock() override{
		return subsystemDynamicsPtrStock_;
	}

	/**
	 * Gets single cost nominal state
	 * @param index
	 * @param timeTrajectory
	 * @param stateTrajectory
	 */
	void getSingleCostNominalState(size_t index, scalar_array_t& timeTrajectory,
			state_vector_array_t& stateTrajectory) const override;

protected:
	/**
	 * Sets single cost nominal State
	 * @param index
	 * @param timeTrajectory
	 * @param stateTrajectory
	 */
	void setSingleCostNominalState(size_t index, const scalar_array_t& timeTrajectory,
			const state_vector_array_t& stateTrajectory) override;

	/**
	 * Solves sequential Riccati equations
	 * @param learningRate
	 * @param SmFinal
	 * @param SvFinal
	 * @param sFinal
	 */
	void solveSequentialRiccatiEquations(const scalar_t& learningRate,
			const state_matrix_t& SmFinal,
			const state_vector_t& SvFinal,
			const eigen_scalar_t& sFinal) override;

	/**
	 * Approximates optimal control problem
	 */
	void approximateOptimalControlProblem();

	/**
	 * Calculates controller
	 */
	void calculateController();

	/**
	 * Line search function
	 * @param learningRateStar
	 * @param maxLearningRateStar
	 */
	void lineSearch(scalar_t& learningRateStar,
			scalar_t maxLearningRateStar=1.0);

private:
	std::vector<std::shared_ptr<ControlledSystemBase<STATE_DIM, INPUT_DIM>>> subsystemDynamicsPtrStock_;

	std::vector<std::shared_ptr<DerivativesBase<STATE_DIM, INPUT_DIM> > > subsystemDerivativesPtrStock_;
	std::vector<std::shared_ptr<CostFunctionBaseOCS2<STATE_DIM, INPUT_DIM> > > subsystemCostFunctionsPtrStock_;

	std::vector<std::shared_ptr<ODE45<STATE_DIM> > > subsystemSimulatorsStockPtr_;

public:
	template <size_t GSLQP_STATE_DIM, size_t GSLQP_INPUT_DIM>
	friend class GSLQP;
};

} // namespace ocs2

#include "implementation/SLQP.h"


#endif /* SLQP_H_ */
