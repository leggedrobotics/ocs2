/**
 * OCS2Projected.h
 *
 *  Created on: July 21, 2016
 *      Author: farbod
 */

#ifndef OCS2_OCS2PROJECTED_H_
#define OCS2_OCS2PROJECTED_H_

#include <array>
#include <memory>
#include <iterator>
#include <algorithm>

#include <ocs2_frank_wolfe/GradientDescent.h>

#include <ocs2_slq/SLQ_BASE.h>
#include <ocs2_slq/SLQ.h>
#include <ocs2_slq/SLQ_MP.h>

#include "ocs2_ocs2/GSLQ.h"

namespace ocs2{

/**
 * OCS2Projected
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 * @tparam LOGIC_RULES_T: Logic Rules type (default NullLogicRules).
 */
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T=NullLogicRules<STATE_DIM,INPUT_DIM>>
class OCS2Projected : private nlp::GradientDescent<double>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	static_assert(std::is_base_of<LogicRulesBase<STATE_DIM, INPUT_DIM>, LOGIC_RULES_T>::value,
			"LOGIC_RULES_T must inherit from LogicRulesBase");

	typedef std::shared_ptr<OCS2Projected<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>> Ptr;

	typedef nlp::GradientDescent<double> BASE;

	typedef SLQ_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T> slq_base_t;
	typedef SLQ<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>      slq_t;
	typedef SLQ_MP<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>   slq_mp_t;
	typedef GSLQ<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>     gslq_t;
	typedef typename slq_base_t::Ptr 	slq_base_ptr_t;
	typedef typename slq_t::Ptr  		slq_ptr_t;
	typedef typename slq_mp_t::Ptr  	slq_mp_ptr_t;
	typedef typename gslq_t::Ptr 		gslq_ptr_t;

	typedef typename slq_base_t::controlled_system_base_t      controlled_system_base_t;
	typedef typename slq_base_t::derivatives_base_t            derivatives_base_t;
	typedef typename slq_base_t::constraint_base_t             constraint_base_t;
	typedef typename slq_base_t::cost_function_base_t          cost_function_base_t;
	typedef typename slq_base_t::cost_desired_trajectories_t   cost_desired_trajectories_t;
	typedef typename slq_base_t::operating_trajectories_base_t operating_trajectories_base_t;


	typedef Dimensions<STATE_DIM, INPUT_DIM> DIMENSIONS;

	typedef typename DIMENSIONS::controller_t          controller_t;
	typedef typename DIMENSIONS::controller_array_t    controller_array_t;
	typedef typename DIMENSIONS::controller_array2_t   controller_array2_t;
	typedef typename DIMENSIONS::scalar_t              scalar_t;
	typedef typename DIMENSIONS::scalar_array_t        scalar_array_t;
	typedef typename DIMENSIONS::eigen_scalar_t        eigen_scalar_t;
	typedef typename DIMENSIONS::eigen_scalar_array_t  eigen_scalar_array_t;
	typedef typename DIMENSIONS::state_vector_t        state_vector_t;
	typedef typename DIMENSIONS::state_vector_array_t  state_vector_array_t;
	typedef typename DIMENSIONS::state_vector_array2_t state_vector_array2_t;
	typedef typename DIMENSIONS::input_vector_t        input_vector_t;
	typedef typename DIMENSIONS::input_vector_array_t  input_vector_array_t;
	typedef typename DIMENSIONS::input_vector_array2_t input_vector_array2_t;
	typedef typename DIMENSIONS::dynamic_vector_t      dynamic_vector_t;
	typedef typename DIMENSIONS::dynamic_matrix_t      dynamic_matrix_t;

	OCS2Projected()
	: numSubsystems_(0)
	{}

	/**
	 * Constructor
	 * @param [in] subsystemDynamicsPtr
	 * @param [in] subsystemDerivativesPtr
	 * @param [in] subsystemCostFunctionsPtr
	 * @param [in] slqSettings
	 * @param [in] stateOperatingPoints
	 * @param [in] inputOperatingPoints
	 */
	OCS2Projected(
			const controlled_system_base_t* systemDynamicsPtr,
			const derivatives_base_t* systemDerivativesPtr,
			const constraint_base_t* systemConstraintsPtr,
			const cost_function_base_t* costFunctionPtr,
			const operating_trajectories_base_t* operatingTrajectoriesPtr,
			const SLQ_Settings& slqSettings = SLQ_Settings(),
			const LOGIC_RULES_T* logicRulesPtr = nullptr,
			const cost_function_base_t* heuristicsFunctionPtr = nullptr);

	/**
	 *
	 */
	virtual ~OCS2Projected() = default;

	/**
	 * Gets cost function
	 * @param [out] costFunction
	 */
	void getCostFunction(scalar_t& costFunction) const;

	/**
	 * Gets the function derivatives
	 * @param [out] costFuntionDerivative
	 */
	void getCostFunctionDerivative(dynamic_vector_t& costFuntionDerivative) const;

	/**
	 * Gets the controller stocks
	 * @param [out] controllersStock
	 */
	void getControllerPtr(std::shared_ptr<controller_array_t>& controllersStock) const;

	void getController(controller_array_t& controllersStock) const;

	const controller_t& controller(size_t index) const;

	/**
	 * Gets event times
	 *
	 * @param [out] eventTimes
	 */
	void getEventTimes(scalar_array_t& eventTimes) const;

	/**
	 * Gets trajectories
	 * @param [out] nominalTimeTrajectoriesStock
	 * @param [out] nominalStateTrajectoriesStock
	 * @param [out] nominalInputTrajectoriesStock
	 */
	void getNominalTrajectories(
			std::vector<scalar_array_t>& nominalTimeTrajectoriesStock,
			state_vector_array2_t& nominalStateTrajectoriesStock,
			input_vector_array2_t& nominalInputTrajectoriesStock) const;

	void getNominalTrajectoriesPtr(
			std::shared_ptr<std::vector<scalar_array_t>>& optimizedTimeTrajectoriesStockPtr,
			std::shared_ptr<state_vector_array2_t>& optimizedStateTrajectoriesStockPtr,
			std::shared_ptr<input_vector_array2_t>& optimizedInputTrajectoriesStockPtr) const;

	/**
	 * Gets slq Iteration COst
	 * @param [out] slqIterationCost
	 * @param [out] slqIterationISE1
	 */
	void getSLQIterationsLog(
			eigen_scalar_array_t& slqIterationCost,
			eigen_scalar_array_t& slqIterationISE1) const {

		slqIterationCost = slqIterationCost_;
		slqIterationISE1 = slqIterationISE1_;
	}

	/**
	 * Gets OCS2 iteration log
	 * @param [out] iterationCost
	 */
	void getOCS2IterationsLog(eigen_scalar_array_t& iterationCost) const { iterationCost = iterationCost_;}

	SLQ_Settings& slqSettings();

	/**
	 * Rollout
	 * @param [in] initTime
	 * @param [in] initState
	 * @param [in] finalTime
	 * @param [in] switchingTimes
	 * @param [in] controllersStock
	 * @param [out] timeTrajectoriesStock
	 * @param [out] stateTrajectoriesStock
	 * @param [out] inputTrajectoriesStock
	 */
	void rollout(
			const scalar_t& initTime,
			const state_vector_t& initState,
			const scalar_t& finalTime,
			const scalar_array_t& switchingTimes,
			const controller_array_t& controllersStock,
			std::vector<scalar_array_t>& timeTrajectoriesStock,
			state_vector_array2_t& stateTrajectoriesStock,
			input_vector_array2_t& inputTrajectoriesStock);

	/**
	 * Calculates cost function
	 * @param [in] timeTrajectoriesStock
	 * @param [in] stateTrajectoriesStock
	 * @param [in] inputTrajectoriesStock
	 * @param [out] totalCost
	 */
	void calculateCostFunction(
			const std::vector<scalar_array_t>& timeTrajectoriesStock,
			const state_vector_array2_t& stateTrajectoriesStock,
			const input_vector_array2_t& inputTrajectoriesStock,
			scalar_t& totalCost);

	/**
	 * Main run function
	 *
	 * @param initTime
	 * @param initState
	 * @param finalTime
	 * @param initEventTimes
	 * @param costDesiredTrajectories
	 */
	void run(
			const scalar_t& initTime,
			const state_vector_t& initState,
			const scalar_t& finalTime,
			const scalar_array_t& partitioningTimes,
			const scalar_array_t& initEventTimes = std::vector<scalar_t>(),
			const cost_desired_trajectories_t& costDesiredTrajectories = cost_desired_trajectories_t());

private:
	/**
	 * Finds nearest neighbor
	 * @param [in] enquiryParameter
	 * @return size_t:
	 */
	size_t findNearestController(const dynamic_vector_t& enquiryParameter) const;

	/**
	 * Calculates linear equality constraint
	 * @param [out] Am
	 * @param [out] Bv
	 */
	void calculateLinearEqualityConstraint(
			dynamic_matrix_t& Am,
			dynamic_vector_t& Bv) override;

	/**
	 * Calculates gradient
	 * @param [in] id
	 * @param [in] parameters
	 * @param [out] gradient
	 * @return boolean:
	 */
	virtual bool calculateGradient(const size_t& id,
			const dynamic_vector_t& parameters,
			dynamic_vector_t& gradient) override;

	/**
	 * Calculates cost
	 * @param [in] id
	 * @param [in] parameters
	 * @param [out] cost
	 * @return boolean:
	 */
	virtual bool calculateCost(
			const size_t& id,
			const dynamic_vector_t& parameters,
			scalar_t& cost) override;

	/**
	 * Get solution
	 * @param [out] idStar
	 */
	virtual void getSolution(size_t idStar) override;

	/**
	 * rewind optimizer
	 * @param [in] firstIndex
	 * @param [in] initRun
	 */
	void rewindOptimizer(
			const size_t& firstIndex,
			bool initRun = false);

	/**
	 * Saves to bag
	 * @param [in] id
	 * @param [in] parameters
	 */
	void saveToBag(
			size_t id,
			const dynamic_vector_t& parameters);

	/**
	 * Calculates initial controller
	 * @param [in] initState
	 * @param [in] switchingTimes
	 * @param [out] controllersStock
	 */
	void calculateInitialController(
			const state_vector_t& initState,
			const scalar_array_t& eventTimes,
			controller_array_t& controllersStock);

	/**
	 * Set up optimizer
	 */
	void setupOptimizer();

private:
	SLQ_Settings slqSettings_;

	std::vector<std::shared_ptr<ControlledSystemBase<STATE_DIM, INPUT_DIM> > >  subsystemDynamicsPtr_;
	std::vector<std::shared_ptr<DerivativesBase<STATE_DIM, INPUT_DIM> > > 	subsystemDerivativesPtr_;
	std::vector<std::shared_ptr<CostFunctionBase<STATE_DIM, INPUT_DIM> > > 	subsystemCostFunctionsPtr_;
	controller_array_t lqpControllersStock_;

	scalar_t       initTime_;
	state_vector_t initState_;
	scalar_t       finalTime_;
	size_t         numSubsystems_;
	scalar_array_t partitioningTimes_;
	size_t         numPartitions_;
	scalar_array_t initEventTimes_;

	// for rollout function
	std::vector<std::shared_ptr<ControlledSystemBase<STATE_DIM, INPUT_DIM> > >  subsystemDynamicsPtrStock_;
	std::vector<std::shared_ptr<CostFunctionBase<STATE_DIM, INPUT_DIM> > > subsystemCostFunctionsPtrStock_;
	std::vector<std::shared_ptr<ODE45<STATE_DIM> > > subsystemSimulatorsStockPtr_;

	// optimized solution variables
	scalar_t optimizedTotalCost_;
	scalar_t optimizedConstraint1ISE_;
	scalar_t optimizedConstraint2ISE_;
	scalar_array_t optimizedEventTimes_;
	controller_array_t optimizedControllersStock_;
	std::vector<scalar_array_t> optimizedTimeTrajectoriesStock_;
	state_vector_array2_t optimizedStateTrajectoriesStock_;
	input_vector_array2_t optimizedInputTrajectoriesStock_;
	dynamic_vector_t costFuntionDerivative_;

	scalar_t currentTotalCost_;
	dynamic_vector_t currentCostFuntionDerivative_;

	std::vector<dynamic_vector_t> parameterBag_;
	controller_array2_t controllersStockBag_;

	std::vector<gslq_ptr_t> gslqSolverPtrs_;
	std::vector<slq_base_ptr_t> slqSolverPtrs_;

	eigen_scalar_array_t slqIterationCost_;
	eigen_scalar_array_t slqIterationISE1_;
	eigen_scalar_array_t slqIterationISE2_;
};

}  // end of ocs2 namespace

#include "implementation/OCS2Projected.h"

#endif /* OCS2_OCS2PROJECTED_H_ */
