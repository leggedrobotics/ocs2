/**
 * GSLQ.h
 *
 *  Created on: Dec 18, 2015
 *      Author: farbod
 */

#ifndef GSLQ_OCS2_H_
#define GSLQ_OCS2_H_

#include <array>
#include <algorithm>
#include <cstddef>
#include <omp.h>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <vector>

#include <ocs2_core/Dimensions.h>
#include <ocs2_core/dynamics/ControlledSystemBase.h>
#include <ocs2_core/dynamics/DerivativesBase.h>
#include <ocs2_core/cost/CostFunctionBase.h>
#include <ocs2_core/integration/Integrator.h>
#include <ocs2_core/misc/LinearInterpolation.h>

#include <ocs2_slq/SLQ_BASE.h>
#include <ocs2_slq/SLQ_DataCollector.h>

#include "ocs2_ocs2/EventTimeIndexer.h"
#include "ocs2_ocs2/sensitivity_equations/SensitivitySequentialRiccatiEquations.h"
#include "ocs2_ocs2/sensitivity_equations/BvpSensitivityEquations.h"
#include "ocs2_ocs2/sensitivity_equations/RolloutSensitivityEquations.h"

namespace ocs2 {

/**
 * GSLQ Class for computing gradient of the cost function w.r.t. switching times.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 * @tparam LOGIC_RULES_T: Logic Rules type (default NullLogicRules).
 */
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T=NullLogicRules<STATE_DIM,INPUT_DIM>>
class GSLQ
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	static_assert(std::is_base_of<LogicRulesBase<STATE_DIM, INPUT_DIM>, LOGIC_RULES_T>::value,
			"LOGIC_RULES_T must inherit from LogicRulesBase");

	typedef std::shared_ptr<GSLQ<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>> Ptr;

	typedef SLQ_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T> slq_t;

	typedef SLQ_DataCollector<STATE_DIM, INPUT_DIM, LOGIC_RULES_T> slq_data_collector_t;

	typedef BvpSensitivityEquations<STATE_DIM, INPUT_DIM> bvp_sensitivity_equations_t;
	typedef RolloutSensitivityEquations<STATE_DIM, INPUT_DIM> rollout_sensitivity_equations_t;
	typedef SensitivitySequentialRiccatiEquations<STATE_DIM, INPUT_DIM> riccati_sensitivity_equations_t;

	typedef LogicRulesMachine<STATE_DIM, INPUT_DIM, LOGIC_RULES_T> logic_rules_machine_t;
	typedef typename logic_rules_machine_t::Ptr                    logic_rules_machine_ptr_t;

	typedef Dimensions<STATE_DIM, INPUT_DIM> DIMENSIONS;

	typedef typename DIMENSIONS::controller_t controller_t;
	typedef typename DIMENSIONS::controller_array_t controller_array_t;
	typedef typename DIMENSIONS::lagrange_t lagrange_t;
	typedef typename DIMENSIONS::lagrange_array_t lagrange_array_t;
	typedef typename DIMENSIONS::size_array_t size_array_t;
	typedef typename DIMENSIONS::scalar_t scalar_t;
	typedef typename DIMENSIONS::scalar_array_t scalar_array_t;
	typedef typename DIMENSIONS::eigen_scalar_t eigen_scalar_t;
	typedef typename DIMENSIONS::eigen_scalar_array_t eigen_scalar_array_t;
	typedef typename DIMENSIONS::eigen_scalar_array2_t eigen_scalar_array2_t;
	typedef typename DIMENSIONS::state_vector_t state_vector_t;
	typedef typename DIMENSIONS::state_vector_array_t state_vector_array_t;
	typedef typename DIMENSIONS::state_vector_array2_t state_vector_array2_t;
	typedef typename DIMENSIONS::input_vector_t input_vector_t;
	typedef typename DIMENSIONS::input_vector_array_t input_vector_array_t;
	typedef typename DIMENSIONS::input_vector_array2_t input_vector_array2_t;
	typedef typename DIMENSIONS::input_state_matrix_t input_state_matrix_t;
	typedef typename DIMENSIONS::input_state_matrix_array_t input_state_matrix_array_t;
	typedef typename DIMENSIONS::input_state_matrix_array2_t input_state_matrix_array2_t;
	typedef typename DIMENSIONS::state_matrix_t state_matrix_t;
	typedef typename DIMENSIONS::state_matrix_array_t state_matrix_array_t;
	typedef typename DIMENSIONS::state_matrix_array2_t state_matrix_array2_t;
	typedef typename DIMENSIONS::input_matrix_t input_matrix_t;
	typedef typename DIMENSIONS::input_matrix_array_t input_matrix_array_t;
	typedef typename DIMENSIONS::input_matrix_array2_t input_matrix_array2_t;
	typedef typename DIMENSIONS::state_input_matrix_t state_input_matrix_t;
	typedef typename DIMENSIONS::state_input_matrix_array_t state_input_matrix_array_t;
	typedef typename DIMENSIONS::state_input_matrix_array2_t state_input_matrix_array2_t;
	typedef typename DIMENSIONS::constraint1_vector_t constraint1_vector_t;
	typedef typename DIMENSIONS::constraint1_vector_array_t constraint1_vector_array_t;
	typedef typename DIMENSIONS::constraint1_vector_array2_t constraint1_vector_array2_t;
	typedef typename DIMENSIONS::constraint1_state_matrix_t constraint1_state_matrix_t;
	typedef typename DIMENSIONS::constraint1_state_matrix_array_t constraint1_state_matrix_array_t;
	typedef typename DIMENSIONS::constraint1_state_matrix_array2_t constraint1_state_matrix_array2_t;
	typedef typename DIMENSIONS::constraint1_input_matrix_t constraint1_input_matrix_t;
	typedef typename DIMENSIONS::constraint1_input_matrix_array_t constraint1_input_matrix_array_t;
	typedef typename DIMENSIONS::constraint1_input_matrix_array2_t constraint1_input_matrix_array2_t;
	typedef typename DIMENSIONS::control_constraint1_matrix_t control_constraint1_matrix_t;
	typedef typename DIMENSIONS::control_constraint1_matrix_array_t control_constraint1_matrix_array_t;
	typedef typename DIMENSIONS::control_constraint1_matrix_array2_t control_constraint1_matrix_array2_t;
	typedef typename DIMENSIONS::constraint2_vector_t       constraint2_vector_t;
	typedef typename DIMENSIONS::constraint2_vector_array_t constraint2_vector_array_t;
	typedef typename DIMENSIONS::constraint2_vector_array2_t constraint2_vector_array2_t;
	typedef typename DIMENSIONS::constraint2_state_matrix_t       constraint2_state_matrix_t;
	typedef typename DIMENSIONS::constraint2_state_matrix_array_t constraint2_state_matrix_array_t;
	typedef typename DIMENSIONS::constraint2_state_matrix_array2_t constraint2_state_matrix_array2_t;
	typedef typename DIMENSIONS::dynamic_vector_t dynamic_vector_t;
	typedef typename DIMENSIONS::dynamic_input_matrix_t dynamic_input_matrix_t;

    typedef std::vector<eigen_scalar_array2_t, Eigen::aligned_allocator<eigen_scalar_array2_t>> eigen_scalar_array3_t;
    typedef std::vector<state_vector_array2_t, Eigen::aligned_allocator<state_vector_array2_t>> state_vector_array3_t;
    typedef std::vector<input_vector_array2_t, Eigen::aligned_allocator<input_vector_array2_t>> input_vector_array3_t;
    typedef std::vector<state_matrix_array2_t, Eigen::aligned_allocator<state_matrix_array2_t>> state_matrix_array3_t;
    typedef std::vector<input_matrix_array2_t, Eigen::aligned_allocator<input_matrix_array2_t>> input_matrix_array3_t;

    /**
     * Constructor.
     *
     * @param [in] slq: A reference to the SLQ instance for which the cost function gradient
     * would be calculated.
     */
	GSLQ(slq_t& slq);

	/**
	 * Destructor.
	 */
	virtual ~GSLQ() = default;

    /**
     * Gets the calculated rollout's sensitivity to an event time.
     *
     * @param [in] eventTimeIndex: Event time index.
     * @param [out] sensitivityTimeTrajectoriesStock: time stamps of the sensitivity values.
     * @param [out] sensitivityStateTrajectoriesStock: state trajectory sensitivity to the switching times.
     * @param [out] sensitivityInputTrajectoriesStock: control input trajectory sensitivity to the switching times.
     */
	void getRolloutSensitivity2SwitchingTime(
			const size_t& eventTimeIndex,
			std::vector<scalar_array_t>& sensitivityTimeTrajectoriesStock,
			state_matrix_array2_t& sensitivityStateTrajectoriesStock,
			input_matrix_array2_t& sensitivityInputTrajectoriesStock);

    /**
     * Gets the calculated optimal controller structure.
     *
     * @param [out] controllersStock
     */
	void getController(controller_array_t& controllersStock);

    /**
     * Calculates the value function for the given time and state vector.
     *
     * @param [in] time: The inquiry time.
     * @param [in] state: The inquiry state.
     * @param [out] valueFuntion: value function at the inquiry time and state.
     */
	void getValueFuntion(
			const scalar_t& time,
			const state_vector_t& state,
			scalar_t& valueFuntion);

    /**
     * Calculates the cost function at the initial time.
     *
     * @param [in] costFunction
     * @param [out] constriantISE
     */
	void getCostFuntion(
			scalar_t& costFunction,
			scalar_t& constriantISE);

    /**
     * Calculates the cost function's derivatives w.r.t. event times.
     *
     * @param [out] costFunctionDerivative: cost function's derivatives w.r.t. event times.
     */
	template <typename Derived>
	void getCostFuntionDerivative(
			Eigen::MatrixBase<Derived> const& costFunctionDerivative) const;

    /**
     * Gets the optimal state and input trajectories.
     *
     * @param [out] nominalTimeTrajectoriesStock
     * @param [out] nominalStateTrajectoriesStock
     * @param [out] nominalInputTrajectoriesStock
     */
	void getNominalTrajectories(
			std::vector<scalar_array_t>& nominalTimeTrajectoriesStock,
			state_vector_array2_t& nominalStateTrajectoriesStock,
			input_vector_array2_t& nominalInputTrajectoriesStock);

	/**
	 * Gets a constant reference to the event time vector.
	 *
	 * @return A constant reference to the event time vector.
	 */
	const scalar_array_t& eventTimes() const;

    /**
     * Runs the GSLQ to compute the gradient of the cost function w.r.t. the event times.
     */
	void run();

protected:
    /**
     * Runs the LQ-based algorithm to compute the value function derivatives wr.t. event times.
     */
	void runLQBasedMethod();

    /**
     * Runs the Sweeping-BVP algorithm to compute the cost function derivatives wr.t. event times.
     */
	void runSweepingBVPMethod();

	/**
	 * Sets up optimizer for different number of partitions.
	 *
	 * @param [in] numPartitions: number of partitions.
	 */
	virtual void setupOptimizer(const size_t& numPartitions);

	/**
	 * Computes the required data which are not computed in the SLQ.
	 */
	void computeMissingSlqData();

	/**
	 * Computes the costate over the given rollout.
	 *
	 * @param [in] timeTrajectoriesStock: the inquiry rollout time
	 * @param [in] stateTrajectoriesStock: the inquiry rollout state
	 * @param [out] costateTrajectoriesStock: costate vector for the given trajectory
	 * @param [in] learningRate: the learning rate.
	 */
	void calculateRolloutCostate(
			const std::vector<scalar_array_t>& timeTrajectoriesStock,
			const state_vector_array2_t& stateTrajectoriesStock,
			state_vector_array2_t& costateTrajectoriesStock,
			scalar_t learningRate = 0.0);

	/**
	 * Computes the nominal costate over the given time.
	 *
	 * @param [in] timeTrajectoriesStock: the inquiry rollout time
	 * @param [out] costateTrajectoriesStock: costate vector for the given trajectory
	 * @param [in] learningRate: the learning rate.
	 */
	void calculateRolloutCostate(
			const std::vector<scalar_array_t>& timeTrajectoriesStock,
			state_vector_array2_t& costateTrajectoriesStock);

	/**
	 * Calculates the linear function approximation of the state-input constraint Lagrangian.
	 * This method uses the following variables:
	 *
	 * @param [out] lagrangeMultiplierFunctionsStock: the linear function approximation of the type-1 constraint Lagrangian.
	 * @param [in] learningRate: the learning rate.
	 */
	void calculateInputConstraintLagrangian(
			lagrange_array_t& lagrangeMultiplierFunctionsStock,
			scalar_t learningRate = 0.0);

	/**
	 * Computes the Lagrange multiplier of the state-input constraint over the given rollout.
	 *
	 * @param [in] timeTrajectoriesStock: the inquiry rollout time
	 * @param [in] stateTrajectoriesStock: the inquiry rollout state
	 * @param [in] lagrangeMultiplierFunctionsStock: the coefficients of the linear function for lagrangeMultiplier
	 * @param [out] lagrangeTrajectoriesStock: lagrangeMultiplier value over the given trajectory
	 */
	void calculateRolloutLagrangeMultiplier(
			const std::vector<scalar_array_t>& timeTrajectoriesStock,
			const state_vector_array2_t& stateTrajectoriesStock,
			const lagrange_array_t& lagrangeMultiplierFunctionsStock,
			constraint1_vector_array2_t& lagrangeTrajectoriesStock);

	/**
	 * Computes the Lagrange multiplier of the state-input constraint over the given time trajectory.
	 *
	 * @param [in] timeTrajectoriesStock: the inquiry rollout time
	 * @param [out] lagrangeTrajectoriesStock: lagrangeMultiplier value over the given trajectory
	 */
	void calculateNominalRolloutLagrangeMultiplier(
			const std::vector<scalar_array_t>& timeTrajectoriesStock,
			constraint1_vector_array2_t& lagrangeTrajectoriesStock);

	/**
	 * Finds the active subsystem. It output is is in the set: {0, 1, ..., #eventTimes+1}.
	 * Thus if no event takes place it returns zero, otherwise the i'th subsystem is active
	 * in the time period [te_{i-1}, te_{i}].
	 *
	 * @param [in] partitioningTimes: a sorted event times sequence.
	 * @param [in] time: inquiry time.
	 * @param [in] ceilingFunction: Use the ceiling function settings ().
	 * @return Active subsystem index.
	 */
	size_t findActiveSubsystemIndex(
			const scalar_array_t& eventTimes,
			const scalar_t& time,
			bool ceilingFunction = true) const;

	/**
	 * Finds the interval of partitioningTimes to which the input time belongs to it.
	 * time is in interval i if: partitioningTimes[i] < t <= partitioningTimes[i+1]
	 * Exception: if time=partitioningTimes[0] then time is interval 0
	 *
	 * @param [in] partitioningTimes: a sorted time sequence.
	 * @param [in] time: Enquiry time.
	 * @param [in] ceilingFunction: Use the ceiling function settings ().
	 * @return Active subsystem index.
	 */
	size_t findActivePartitionIndex(
			const scalar_array_t& partitioningTimes,
			const scalar_t& time,
			bool ceilingFunction = true) const;

	/**
	 * Computes the equivalent system formulation multiplier. which is
	 * \f$ \frac{\delta_{i,j}-\delta_{i-1,j}}{t_{i}-t_{i-1}} \f$
	 * where i is activeSubsystem and j is eventTimeIndex.
	 *
	 * @param [in] eventTimeIndex: Event time index.
	 * @param [in] activeSubsystem: Current active subsystem index.
	 * @param [out] multiplier: Equivalent system formulation multiplier
	 */
	void computeEquivalentSystemMultiplier(
			const size_t& eventTimeIndex,
			const size_t& activeSubsystem,
			scalar_t& multiplier) const;

	/**
	 * Integrates the sensitivity equation of the rollout w.r.t. event times.
	 *
	 * @param [in] workerIndex: Working agent index.
	 * @param [in] eventTimeIndex: Event time index.
	 * @param [in] controllersStock: Nominal controller.
	 * @param [in] LvTrajectoriesStock: Controller's feedforward sensitivity
	 * @param [in] sensitivityTimeTrajectoriesStock: Integration time trajectory.
	 * @param [in] eventsPastTheEndIndecesStock: Indices containing past-the-end index of events trigger.
	 * @param [out] sensitivityStateTrajectoriesStock: Array of state sensitivity trajectory.
	 * @param [out] sensitivityInputTrajectoriesStock: Array of input sensitivity trajectory.
	 */
	void propagateRolloutSensitivity(
			size_t workerIndex,
			const size_t& eventTimeIndex,
			const controller_array_t& controllersStock,
			const input_vector_array2_t& LvTrajectoriesStock,
			const std::vector<scalar_array_t>& sensitivityTimeTrajectoriesStock,
			const std::vector<size_array_t>& eventsPastTheEndIndecesStock,
			state_vector_array2_t& sensitivityStateTrajectoriesStock,
			input_vector_array2_t& sensitivityInputTrajectoriesStock);

	/**
	 * Approximates nominal LQ problem sensitivity to event times.
	 *
	 * @param [in] sensitivityStateTrajectoriesStock: Array of state sensitivity trajectory.
	 * @param [in] sensitivityInputTrajectoriesStock: Array of input sensitivity trajectory.
	 * @param [out] nablaqTrajectoriesStock: Sensitivity of the cost's zero order variation.
	 * @param [out] nablaQvTrajectoriesStock: Sensitivity of the cost's first order state variation.
	 * @param [out] nablaRvTrajectoriesStock: Sensitivity of the cost's first order input variation.
	 * @param [out] nablaqFinalStock: Sensitivity of the final cost's zero order variation.
	 * @param [out] nablaQvFinalStock: Sensitivity of the final cost's first order state variation.
	 */
	void approximateNominalLQPSensitivity2SwitchingTime(
			const state_vector_array2_t& sensitivityStateTrajectoriesStock,
			const input_vector_array2_t& sensitivityInputTrajectoriesStock,
			eigen_scalar_array2_t& nablaqTrajectoriesStock,
			state_vector_array2_t& nablaQvTrajectoriesStock,
			input_vector_array2_t& nablaRvTrajectoriesStock,
			eigen_scalar_array2_t& nablaqFinalStock,
			state_vector_array2_t& nablaQvFinalStock) const;

	/**
	 * Approximates nominal LQ problem sensitivity to event times.
	 *
	 * @param [in] sensitivityFinalState: Final state sensitivity.
	 * @param [out] nablasHeuristics: Sensitivity of the Heuristics zero order variation.
	 * @param [out] nablaSvHeuristics: Sensitivity of the Heuristics first order state variation.
	 */
	void approximateNominalHeuristicsSensitivity2SwitchingTime(
			const state_vector_t& sensitivityFinalState,
			eigen_scalar_t& nablasHeuristics,
			state_vector_t& nablaSvHeuristics) const;

	/**
	 * Solves the SLQ Riccati equations sensitivity differential equations.
	 *
	 * @param [in] workerIndex: Working agent index.
	 * @param [in] eventTimeIndex: Event time index.
	 * @param [in] learningRate: learning rate typically should be zero.
	 * @param [in] nablasHeuristics: Sensitivity of the Heuristics zero order variation.
	 * @param [in] nablaSvHeuristics: Sensitivity of the Heuristics first order state variation.
	 * @param [in] nablaSmHeuristics: Sensitivity of the Heuristics second order state variation.
	 * @param [out] nablasTrajectoriesStock: Sensitivity of the Riccati equations's zero order variation.
	 * @param [out] nablaSvTrajectoriesStock: Sensitivity of the Riccati equations's first order variation.
	 * @param [out] nablaSmTrajectoriesStock: Sensitivity of the Riccati equations's second order variation.
	 */
	void solveSensitivityRiccatiEquations(
			size_t workerIndex,
			const size_t& eventTimeIndex,
			const scalar_t& learningRate,
			const eigen_scalar_t& nablasHeuristics,
			const state_vector_t& nablaSvHeuristics,
			const state_matrix_t& nablaSmHeuristics,
			eigen_scalar_array2_t& nablasTrajectoriesStock,
			state_vector_array2_t& nablaSvTrajectoriesStock,
			state_matrix_array2_t& nablaSmTrajectoriesStock);

	/**
	 * Solves a boundary value problem using a Riccati approach which later be used to
	 * compute controller's feedforward sensitivity w.r.t. event times.
	 *
	 * @param [in] workerIndex: Working agent index.
	 * @param [in] eventTimeIndex: Event time index.
	 * @param [in] MvFinal: The final Heuristic value.
	 * @param [out] MvTrajectoriesStock: Boundary value problem solution.
	 */
	void solveSensitivityBVP(
			size_t workerIndex,
			const size_t& eventTimeIndex,
			const state_vector_t& MvFinal,
			state_vector_array2_t& MvTrajectoriesStock);

    /**
     * Calculates controller's feedforward part sensitivity for the LQ method.
     *
     * @param [in] workerIndex: Working agent index.
	 * @param [in] eventTimeIndex: Event time index.
     * @param [in] timeTrajectoriesStock: Time stamp of the riccati solution
     * @param [in] nablaSvTrajectoriesStock: Sensitivity of the Riccati equations's first order variation.
     * @param [out] nablaLvTrajectoriesStock: Sensitivity of the control input increment to event times.
     */
	void calculateLQSensitivityControllerForward(
			size_t workerIndex,
			const size_t& eventTimeIndex,
			const std::vector<scalar_array_t>& timeTrajectoriesStock,
			const state_vector_array2_t& nablaSvTrajectoriesStock,
			input_vector_array2_t& nablaLvTrajectoriesStock);

    /**
     * calculate the sensitivity of the control input increment to event times
     * based on the BVP method.
     *
     * @param [in] workerIndex: Working agent index.
	 * @param [in] eventTimeIndex: Event time index.
     * @param [in] timeTrajectoriesStock: Time stamp of the BVP solution
     * @param [in] MvTrajectoriesStock: BVP solution.
     * @param [out] LvTrajectoriesStock: Sensitivity of the control input increment to event times.
     */
	void calculateBVPSensitivityControllerForward(
			size_t workerIndex,
			const size_t& eventTimeIndex,
			const std::vector<scalar_array_t>& timeTrajectoriesStock,
			const state_vector_array2_t& MvTrajectoriesStock,
			input_vector_array2_t& LvTrajectoriesStock);

    /**
     * Calculates the value function's derivative w.r.t. an event time for a given time and state.
     *
     * @param [in] eventTimeIndex: Event time index.
     * @param [in] time: The inquired time.
     * @param [in] state: The inquired state.
     * @param [out] valueFunctionDerivative: The value function's derivative w.r.t. an event time.
     */
	void getValueFuntionDerivative(
			const size_t& eventTimeIndex,
			const scalar_t& time,
			const state_vector_t& state,
			scalar_t& valueFunctionDerivative);

	/**
	 * calculates cost function derivative based on BVP solution
	 *
	 * @param [in] workerIndex: Working agent index.
	 * @param [in] eventTimeIndex: Event time index.
	 * @param [in] sensitivityStateTrajectoriesStock: Array of state sensitivity trajectory.
	 * @param [in] sensitivityInputTrajectoriesStock: Array of input sensitivity trajectory.
	 * @param [out] costDerivative: The cost function's derivative w.r.t. an event time.
	 */
	void calculateCostDerivative(
			size_t workerIndex,
			const size_t& eventTimeIndex,
			const state_vector_array2_t& sensitivityStateTrajectoriesStock,
			const input_vector_array2_t& sensitivityInputTrajectoriesStock,
			scalar_t& costDerivative) const;

private:
	const slq_t* slqPtr_;
	const SLQ_Settings* settingsPtr_;
	logic_rules_machine_t* logicRulesMachinePtr_;

	size_t numSubsystems_;
	size_t numEventTimes_;

	size_t activeEventTimeBeginIndex_;
	size_t activeEventTimeEndIndex_;

	/******************
	 * SLQ data collector
	 ******************/
	std::shared_ptr<slq_data_collector_t> dcPtr_;

	/******************
	 * SLQ missing variables
	 ******************/
	state_vector_array2_t       nominalCostateTrajectoriesStock_;
	constraint1_vector_array2_t nominalLagrangianTrajectoriesStock_;

	std::vector<std::shared_ptr<bvp_sensitivity_equations_t>> bvpSensitivityEquationsPtrStock_;
	std::vector<std::shared_ptr<IntegratorBase<STATE_DIM>>>   bvpSensitivityIntegratorsPtrStock_;
	std::vector<std::shared_ptr<rollout_sensitivity_equations_t>> rolloutSensitivityEquationsPtrStock_;
	std::vector<std::shared_ptr<IntegratorBase<STATE_DIM>>>       rolloutSensitivityIntegratorsPtrStock_;
	std::vector<std::shared_ptr<riccati_sensitivity_equations_t>> riccatiSensitivityEquationsPtrStock_;
	std::vector<std::shared_ptr<IntegratorBase<riccati_sensitivity_equations_t::S_DIM_>>> riccatiSensitivityIntegratorsPtrStock_;
	//
	eigen_scalar_array3_t nablaqTrajectoriesStockSet_;
	state_vector_array3_t nablaQvTrajectoriesStockSet_;
	input_vector_array3_t nablaRvTrajectoriesStockSet_;
	eigen_scalar_array3_t nablaqFinalStockSet_;
	state_vector_array3_t nablaQvFinalStockSet_;
	eigen_scalar_array_t  nablasHeuristics_;
	state_vector_array_t  nablaSvHeuristics_;

	eigen_scalar_array3_t nablasTrajectoriesStockSet_;
	state_vector_array3_t nablaSvTrajectoriesStockSet_;
	state_matrix_array3_t nablaSmTrajectoriesStockSet_;
	input_vector_array3_t nablaLvTrajectoriesStockSet_;

	state_vector_array3_t MvTrajectoriesStockSet_;  // Riccati solution for sensitivity controller feedforward
	input_vector_array3_t LvTrajectoriesStockSet_;  // sensitivity controller feedforward

	state_vector_array3_t sensitivityStateTrajectoriesStockSet_;
	input_vector_array3_t sensitivityInputTrajectoriesStockSet_;

	dynamic_vector_t nominalCostFuntionDerivative_;


	// calculateBVPSensitivityControllerForward & calculateLQSensitivityControllerForward
	std::vector<EigenLinearInterpolation<state_input_matrix_t>> BmFuncStock_;
	std::vector<EigenLinearInterpolation<input_matrix_t>> RmInverseFuncStock_;
	std::vector<EigenLinearInterpolation<input_matrix_t>> DmProjectedFuncStock_;
	std::vector<EigenLinearInterpolation<input_vector_t>> nablaRvFuncStock_;

	EigenLinearInterpolation<state_matrix_t> SmFunc_;
	EigenLinearInterpolation<state_vector_t> SvFunc_;
	EigenLinearInterpolation<state_vector_t> SveFunc_;
	EigenLinearInterpolation<state_vector_t> nominalStateFunc_;

};

} // namespace ocs2

#include "implementation/GSLQ.h"

#endif /* GSLQ_OCS2_H_ */
