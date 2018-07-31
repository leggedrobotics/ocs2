/**
 * OCS2Projected.h
 *
 *  Created on: Jul 21, 2016
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
class OCS2Projected : private nlp::GradientDescent
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	static_assert(std::is_base_of<LogicRulesBase<STATE_DIM, INPUT_DIM>, LOGIC_RULES_T>::value,
			"LOGIC_RULES_T must inherit from LogicRulesBase");

	typedef std::shared_ptr<OCS2Projected<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>> Ptr;

	typedef SLQ_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T> slq_base_t;
	typedef SLQ<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>      slq_t;
	typedef SLQ_MP<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>   slq_mp_t;
	typedef GSLQ<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>     gslq_t;
	typedef typename slq_base_t::Ptr 	slq_base_ptr_t;
	typedef typename slq_t::Ptr  		slq_ptr_t;
	typedef typename slq_mp_t::Ptr  	slq_mp_ptr_t;
	typedef typename gslq_t::Ptr 		gslq_ptr_t;

	typedef Dimensions<STATE_DIM, INPUT_DIM> DIMENSIONS;

	typedef typename DIMENSIONS::controller_t 		controller_t;
	typedef typename DIMENSIONS::controller_array_t controller_array_t;
	typedef typename DIMENSIONS::scalar_t 		scalar_t;
	typedef typename DIMENSIONS::scalar_array_t scalar_array_t;
	typedef typename DIMENSIONS::eigen_scalar_t       eigen_scalar_t;
	typedef typename DIMENSIONS::eigen_scalar_array_t eigen_scalar_array_t;
	typedef typename DIMENSIONS::state_vector_t 	  state_vector_t;
	typedef typename DIMENSIONS::state_vector_array_t state_vector_array_t;
	typedef typename DIMENSIONS::state_vector_array2_t state_vector_array2_t;
	typedef typename DIMENSIONS::input_vector_t 		input_vector_t;
	typedef typename DIMENSIONS::input_vector_array_t input_vector_array_t;
	typedef typename DIMENSIONS::input_vector_array2_t input_vector_array2_t;
	typedef typename DIMENSIONS::input_state_t 	  	 input_state_t;
	typedef typename DIMENSIONS::input_state_array_t input_state_array_t;
	typedef typename DIMENSIONS::state_matrix_t 	  state_matrix_t;
	typedef typename DIMENSIONS::state_matrix_array_t state_matrix_array_t;
	typedef typename DIMENSIONS::input_matrix_t 		input_matrix_t;
	typedef typename DIMENSIONS::input_matrix_array_t 	input_matrix_array_t;
	typedef typename DIMENSIONS::state_input_matrix_t 		state_input_matrix_t;
	typedef typename DIMENSIONS::state_input_matrix_array_t state_input_matrix_array_t;

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
			const std::vector<std::shared_ptr<ControlledSystemBase<STATE_DIM, INPUT_DIM> > >& subsystemDynamicsPtr,
			const std::vector<std::shared_ptr<DerivativesBase<STATE_DIM, INPUT_DIM> > >& subsystemDerivativesPtr,
			const std::vector<std::shared_ptr<CostFunctionBase<STATE_DIM, INPUT_DIM> > >& subsystemCostFunctionsPtr,
			const SLQ_Settings& slqSettings,
			const state_vector_array_t& stateOperatingPoints,
			const input_vector_array_t& inputOperatingPoints);

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
	 * Gets the fucntion derivatives
	 * @param [out] costFuntionDerivative
	 */
	void getCostFunctionDerivative(Eigen::VectorXd& costFuntionDerivative) const;

	/**
	 * Gets the controller stocks
	 * @param [out] controllersStock
	 */
	void getControllerPtr(std::shared_ptr<controller_array_t>& controllersStock) const;

	void getController(controller_array_t& controllersStock) const;

	const controller_t& controller(size_t index) const;

	/**
	 * Gets switching times
	 * @param [out] switchingTimes
	 */
	void getSwitchingTimes(scalar_array_t& switchingTimes) const;

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
	 * Gets OCS2 iteraltion log
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
	 * @param [in] initTime
	 * @param [in] initState
	 * @param [in] finalTime
	 * @param [in] systemStockIndexes
	 * @param [in] switchingTimes
	 * @param [in] controllersStock
	 * @param [in] desiredTimeTrajectoriesStock
	 * @param [in] desiredStateTrajectoriesStock
	 */
	void run(
			const scalar_t& initTime,
			const state_vector_t& initState,
			const scalar_t& finalTime,
			const std::vector<size_t>& systemStockIndexes = std::vector<size_t>(),
			const std::vector<scalar_t>& switchingTimes = std::vector<scalar_t>(),
			const controller_array_t& controllersStock = controller_array_t(),
			const std::vector<scalar_array_t>& desiredTimeTrajectoriesStock = std::vector<scalar_array_t>(),
			const state_vector_array2_t& desiredStateTrajectoriesStock = state_vector_array2_t());

private:
	/**
	 * Finds nearest neighbour
	 * @param [in] enquiryParameter
	 * @return size_t:
	 */
	size_t findNearestController(const Eigen::VectorXd& enquiryParameter) const;

	/**
	 * Calculates linear equality constraint
	 * @param [out] Am
	 * @param [out] Bv
	 */
	void calculateLinearEqualityConstraint(
			Eigen::MatrixXd& Am,
			Eigen::VectorXd& Bv) override;

	/**
	 * Calculates gradient
	 * @param [in] id
	 * @param [in] parameters
	 * @param [out] gradient
	 * @return boolean:
	 */
	bool calculateGradient(const size_t& id,
			const Eigen::VectorXd& parameters,
			Eigen::VectorXd& gradient) override;

	/**
	 * Calculates cost
	 * @param [in] id
	 * @param [in] parameters
	 * @param [out] cost
	 * @return boolean:
	 */
	bool calculateCost(
			const size_t& id,
			const Eigen::VectorXd& parameters,
			scalar_t& cost) override;

	/**
	 * Get solution
	 * @param [out] idStar
	 */
	void getSolution(size_t idStar) override;

	/**
	 * rewind optimizer
	 * @param [in] firstIndex
	 * @param [in] initRun
	 */
	void rewindOptimizer(
			const size_t& firstIndex,
			bool initRun = false);

	/**
	 * use disjoint Riccati approach
	 * @param [in] useDisjointRiccati
	 */
	void setUseDisjointRiccati(bool useDisjointRiccati);

	/**
	 * Saves to bag
	 * @param [in] id
	 * @param [in] parameters
	 */
	void saveToBag(
			size_t id,
			const Eigen::VectorXd& parameters);

	/**
	 * Calculates initial controller
	 * @param [in] initState
	 * @param [in] switchingTimes
	 * @param [out] controllersStock
	 */
	void calculateInitialController(
			const state_vector_t& initState,
			const scalar_array_t& switchingTimes,
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

	scalar_t 		initTime_;
	state_vector_t 	initState_;
	scalar_t 		finalTime_;
	size_t 			numSubsystems_;
	std::vector<size_t> systemStockIndexes_;
	scalar_array_t 		initSwitchingTimes_;
	std::vector<scalar_array_t> desiredTimeTrajectoriesStock_;
	state_vector_array2_t 		desiredStateTrajectoriesStock_;

	// for rollout function
	std::vector<std::shared_ptr<ControlledSystemBase<STATE_DIM, INPUT_DIM> > >  subsystemDynamicsPtrStock_;
	std::vector<std::shared_ptr<CostFunctionBaseOCS2<STATE_DIM, INPUT_DIM> > > subsystemCostFunctionsPtrStock_;
	std::vector<std::shared_ptr<ODE45<STATE_DIM> > > subsystemSimulatorsStockPtr_;

	// optimized solution variables
	scalar_t       optimizedTotalCost_;
	scalar_t       optimizedConstraintISE_;
	scalar_array_t optimizedSwitchingTimes_;
	controller_array_t optimizedControllersStock_;
	std::vector<scalar_array_t> optimizedTimeTrajectoriesStock_;
	state_vector_array2_t   optimizedStateTrajectoriesStock_;
	input_vector_array2_t optimizedInputTrajectoriesStock_;
	Eigen::VectorXd 		costFuntionDerivative_;

	scalar_t currentTotalCost_;
	Eigen::VectorXd currentCostFuntionDerivative_;

	std::vector<Eigen::VectorXd> parameterBag_;
	std::vector<controller_array_t, Eigen::aligned_allocator<controller_array_t> > controllersStockBag_;

	gslq_ptr_t gslqSolver_;
	std::vector<slq_base_ptr_t> slqSolverPtrs_;

	eigen_scalar_array_t slqIterationCost_;
	eigen_scalar_array_t slqIterationISE1_;
};

}  // end of ocs2 namespace

#include "implementation/OCS2Projected.h"

#endif /* OCS2_OCS2PROJECTED_H_ */
