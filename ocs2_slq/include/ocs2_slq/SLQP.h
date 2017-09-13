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
	 * Constructor
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
	 * Forward integrate the system dynamics with given controller:
	 * 		inputs:
	 * 			+ initTime: intial time
	 * 			+ initState: initial state at time initTime
	 * 			+ controllersStock: controller for each subsystem
	 * 		outputs:
	 * 			+ timeTrajectoriesStock:  rollout simulated time steps
	 * 			+ stateTrajectoriesStock: rollout states
	 * 			+ inputTrajectoriesStock: rollout control inputs
	 * 			+ (optional) stateTrajectoriesStock_: rollout outputs
	 * 			+ (optional) nc1TrajectoriesStock: number of active constraints at each time step
	 * 			+ (optional) EvTrajectoryStock: value of the constraint (if the rollout is constrained the value is
	 * 											always zero otherwise it is nonzero)
	 * @param [in] initTime
	 * @param [in] initState
	 * @param [in] finalTime
	 * @param [out] controllersStock
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
			constraint2_vector_array_t& HvFinalStock) override;

	/**
	 * Forward integrate the system dynamics with given controller:
	 * 		inputs:
	 * 			+ initTime: intial time
	 * 			+ initState: initial state at time initTime
	 * 			+ controllersStock: controller for each subsystem
	 * 		outputs:
	 * 			+ timeTrajectoriesStock:  rollout simulated time steps
	 * 			+ stateTrajectoriesStock: rollout states
	 * 			+ inputTrajectoriesStock: rollout control inputs
	 * @param [in] initTime
	 * @param [in] initState
	 * @param [in] finalTime
	 * @param [out] controllersStock
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
			control_vector_array2_t& inputTrajectoriesStock) override;

	/**
	 * Forward integrate the system dynamics with given controller:
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
	 * compute the cost for a given rollout
	 * 		inputs:
	 * 			+ timeTrajectoriesStock:  rollout simulated time steps
	 * 			+ stateTrajectoriesStock: rollout outputs
	 * 			+ inputTrajectoriesStock: rollout control inputs
	 *
	 * 		outputs:
	 * 			+ totalCost: the total cost of the trajectory
	 * @param [in] timeTrajectoriesStock
	 * @param [in] stateTrajectoriesStock
	 * @param [in] inputTrajectoriesStock
	 * @param [out] totalCost
	 */
	void calculateCostFunction(const std::vector<scalar_array_t>& timeTrajectoriesStock,
			const state_vector_array2_t& stateTrajectoriesStock,
			const control_vector_array2_t& inputTrajectoriesStock,
			scalar_t& totalCost) override;

	/**
	 * compute the cost for a given rollout
	 * @param [in] timeTrajectoriesStock
	 * @param [in] stateTrajectoriesStock
	 * @param [in] inputTrajectoriesStock
	 * @param [out] nc2TrajectoriesStock
	 * @param [out] HvTrajectoryStock
	 * @param [out] nc2FinalStock
	 * @param [out] HvFinalStock
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
	 * Sets up the optimizer
	 */
	void setupOptimizer() override;

	/**
	 * Runs initialization
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
	 * Runs exit
	 * @param [in] SmFinal
	 * @param [in] SvFinal
	 * @param [in] sFinal
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
	 * @param [in] index
	 * @param [out] timeTrajectory
	 * @param [out] stateTrajectory
	 */
	void getSingleCostNominalState(size_t index, scalar_array_t& timeTrajectory,
			state_vector_array_t& stateTrajectory) const override;

protected:
	/**
	 * Sets single cost nominal State
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
	 * 		+ as well as the constrained coefficients of
	 * 			linearized system model
	 * 			quadratized intermediate cost function
	 * 			quadratized final cost
	 * Approximates optimal control problem
	 */
	void approximateOptimalControlProblem();

	/**
	 * calculates the controller and linear function approximation of the type-1 constraint Lagrangian:
	 * 		This method uses the following variables:
	 * 			+ constrained, linearized model
	 * 			+ constrained, quadratized cost
	 *
	 * 		The method outputs:
	 * 			+ BASE::nominalControllersStock_: the controller that stabilizes the system around the new nominal trajectory and
	 * 								improves the constraints as well as the increment to the feedforward control input.
	 */
	void calculateController();

	/**
	 * line search on the feedforwrd parts of the controller and lagrange multipliers.
	 * Based on the option flag lineSearchByMeritFuntion_ it uses two different approaches for line search:
	 * 		+ the constraint correction term is added by a user defined stepSize.
	 * 		The line search uses the pure cost function for choosing the best stepSize.
	 * @param [out] learningRateStar
	 * @param [in] maxLearningRateStar
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
