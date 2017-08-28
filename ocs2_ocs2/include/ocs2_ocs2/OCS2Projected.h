/**
 * OCS2Projected.h
 *
 *  Created on: Jul 21, 2016
 *      Author: farbod
 */

#ifndef OCS2_OCS2PROJECTED_H_
#define OCS2_OCS2PROJECTED_H_

#include <vector>
#include <array>
#include <memory>
#include <iterator>
#include <algorithm>

#include <c_gradient_descent/GradientDescent.h>

#include <ocs2_slq/GLQP.h>

#include "ocs2_ocs2/GSLQP.h"

namespace ocs2{

/**
 * OCS2Projected
 * @tparam STATE_DIM
 * @tparam INPUT_DIM
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class OCS2Projected : private nlp::GradientDescent
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef std::shared_ptr<OCS2Projected<STATE_DIM, INPUT_DIM> > Ptr;

	typedef GLQP<STATE_DIM, INPUT_DIM> 		lqp_t;
	typedef SLQP_BASE<STATE_DIM, INPUT_DIM> slqp_base_t;
	typedef SLQP<STATE_DIM, INPUT_DIM>  	slqp_t;
	typedef SLQP_MP<STATE_DIM, INPUT_DIM>  	slqp_mp_t;
	typedef GSLQP<STATE_DIM, INPUT_DIM> 	gslqp_t;
	typedef typename lqp_t::Ptr			lqp_ptr_t;
	typedef typename slqp_base_t::Ptr 	slqp_base_ptr_t;
	typedef typename slqp_t::Ptr  		slqp_ptr_t;
	typedef typename slqp_mp_t::Ptr  	slqp_mp_ptr_t;
	typedef typename gslqp_t::Ptr 		gslqp_ptr_t;

	typedef Dimensions<STATE_DIM, INPUT_DIM> DIMENSIONS;
	typedef typename DIMENSIONS::controller_t 		controller_t;
	typedef typename DIMENSIONS::controller_array_t controller_array_t;
	typedef typename DIMENSIONS::Options Options_t;
	typedef typename DIMENSIONS::scalar_t 		scalar_t;
	typedef typename DIMENSIONS::scalar_array_t scalar_array_t;
	typedef typename DIMENSIONS::eigen_scalar_t       eigen_scalar_t;
	typedef typename DIMENSIONS::eigen_scalar_array_t eigen_scalar_array_t;
	typedef typename DIMENSIONS::state_vector_t 	  state_vector_t;
	typedef typename DIMENSIONS::state_vector_array_t state_vector_array_t;
	typedef typename DIMENSIONS::state_vector_array2_t state_vector_array2_t;
	typedef typename DIMENSIONS::control_vector_t 		control_vector_t;
	typedef typename DIMENSIONS::control_vector_array_t control_vector_array_t;
	typedef typename DIMENSIONS::control_vector_array2_t control_vector_array2_t;
	typedef typename DIMENSIONS::control_feedback_t 	  control_feedback_t;
	typedef typename DIMENSIONS::control_feedback_array_t control_feedback_array_t;
	typedef typename DIMENSIONS::state_matrix_t 	  state_matrix_t;
	typedef typename DIMENSIONS::state_matrix_array_t state_matrix_array_t;
	typedef typename DIMENSIONS::control_matrix_t 		control_matrix_t;
	typedef typename DIMENSIONS::control_matrix_array_t control_matrix_array_t;
	typedef typename DIMENSIONS::control_gain_matrix_t 		 control_gain_matrix_t;
	typedef typename DIMENSIONS::control_gain_matrix_array_t control_gain_matrix_array_t;

	OCS2Projected()
	: numSubsystems_(0)
	{}

	/**
	 * Constructor
	 * @param [in] subsystemDynamicsPtr
	 * @param [in] subsystemDerivativesPtr
	 * @param [in] subsystemCostFunctionsPtr
	 * @param [in] options
	 * @param [in] stateOperatingPoints
	 * @param [in] inputOperatingPoints
	 */
	OCS2Projected(const std::vector<std::shared_ptr<ControlledSystemBase<STATE_DIM, INPUT_DIM> > >& subsystemDynamicsPtr,
			const std::vector<std::shared_ptr<DerivativesBase<STATE_DIM, INPUT_DIM> > >& subsystemDerivativesPtr,
			const std::vector<std::shared_ptr<CostFunctionBaseOCS2<STATE_DIM, INPUT_DIM> > >& subsystemCostFunctionsPtr,
			const Options_t& options,
			const state_vector_array_t&   stateOperatingPoints,
			const control_vector_array_t& inputOperatingPoints)

	: numSubsystems_(0),
	  subsystemDynamicsPtr_(subsystemDynamicsPtr),
	  subsystemDerivativesPtr_(subsystemDerivativesPtr),
	  subsystemCostFunctionsPtr_(subsystemCostFunctionsPtr),
	  options_(options)
	{
		// NLP optimizer options
		nlpOptions_.displayGradientDescent_ = options_.displayGradientDescent_;
		nlpOptions_.maxIterations_ 	  = options_.maxIterationGradientDescent_;
		nlpOptions_.minRelCost_    	  = options_.acceptableTolGradientDescent_;
		nlpOptions_.maxLearningRate_  = options_.maxLearningRateNLP_;
		nlpOptions_.minLearningRate_  = options_.minLearningRateNLP_;
		nlpOptions_.minDisToBoundary_ = options_.minAcceptedSwitchingTimeDifference_;
		nlpOptions_.useAscendingLineSearchNLP_ = options_.useAscendingLineSearchNLP_;
		adjustOptions();

		// setting up subsystemSimulatorsStockPtr
		if (subsystemDynamicsPtr.size() != subsystemCostFunctionsPtr.size())
			throw std::runtime_error("Number of cost functions is not equal to the number of subsystems.");

		// SLQP solvers
		slqpSolverPtrs_.resize(numLineSearch_+1);
		for (size_t i=0; i<slqpSolverPtrs_.size(); i++)
			if (options_.useMultiThreading_==true)
				slqpSolverPtrs_[i] = slqp_base_ptr_t( new slqp_mp_t(subsystemDynamicsPtr, subsystemDerivativesPtr, subsystemCostFunctionsPtr,
						options, stateOperatingPoints, inputOperatingPoints) );
			else
				slqpSolverPtrs_[i] = slqp_base_ptr_t( new slqp_t(subsystemDynamicsPtr, subsystemDerivativesPtr, subsystemCostFunctionsPtr,
						options, stateOperatingPoints, inputOperatingPoints) );

		// GSLQP solver
		gslqpSolver_ = gslqp_ptr_t( new gslqp_t(options) );

		// LQP solver
		lqp_ptr_t lqpPtr_( new lqp_t(subsystemDynamicsPtr, subsystemDerivativesPtr, subsystemCostFunctionsPtr, stateOperatingPoints, inputOperatingPoints) );
		const size_t N = subsystemDynamicsPtr.size();
		std::vector<size_t>   systemStockIndexes(1);
		std::vector<scalar_t> switchingTimes(2);
		lqpControllersStock_.resize(N);
		for (size_t i=0; i<N; i++) {

			systemStockIndexes[0] = i;
			switchingTimes[0] = 0.0;
			switchingTimes[1] = 1.0;

			lqpPtr_->run(systemStockIndexes, switchingTimes, 0.0/*=1.0*/);
			controller_array_t local_controller(1);
			lqpPtr_->getController(local_controller);
			lqpControllersStock_[i].swap(local_controller[0]);
		} // end of i loop

	}

	~OCS2Projected() {}

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
	void getNominalTrajectories(std::vector<scalar_array_t>& nominalTimeTrajectoriesStock,
			state_vector_array2_t& nominalStateTrajectoriesStock,
			control_vector_array2_t& nominalInputTrajectoriesStock) const;

	void getNominalTrajectoriesPtr(std::shared_ptr<std::vector<scalar_array_t>>& optimizedTimeTrajectoriesStockPtr,
			std::shared_ptr<state_vector_array2_t>& optimizedStateTrajectoriesStockPtr,
			std::shared_ptr<control_vector_array2_t>& optimizedInputTrajectoriesStockPtr) const;

	/**
	 * Gets slq Iteration COst
	 * @param [out] slqIterationCost
	 * @param [out] slqIterationISE1
	 */
	void getSLQIterationsLog(eigen_scalar_array_t& slqIterationCost, eigen_scalar_array_t& slqIterationISE1) const {
		slqIterationCost = slqIterationCost_;
		slqIterationISE1 = slqIterationISE1_;
	}

	/**
	 * Gets OCS2 iteraltion log
	 * @param [out] iterationCost
	 */
	void getOCS2IterationsLog(eigen_scalar_array_t& iterationCost) const { iterationCost = iterationCost_;}

	Options_t& options();

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
	void rollout(const double& initTime,
			const state_vector_t& initState,
			const scalar_t& finalTime,
			const scalar_array_t& switchingTimes,
			const controller_array_t& controllersStock,
			std::vector<scalar_array_t>& timeTrajectoriesStock,
			state_vector_array2_t& stateTrajectoriesStock,
			control_vector_array2_t& inputTrajectoriesStock);

	/**
	 * Calculates cost function
	 * @param [in] timeTrajectoriesStock
	 * @param [in] stateTrajectoriesStock
	 * @param [in] inputTrajectoriesStock
	 * @param [out] totalCost
	 */
	void calculateCostFunction(const std::vector<scalar_array_t>& timeTrajectoriesStock,
			const state_vector_array2_t& stateTrajectoriesStock,
			const control_vector_array2_t& inputTrajectoriesStock,
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
	void run(const scalar_t& initTime, const state_vector_t& initState, const scalar_t& finalTime,
			const std::vector<size_t>& systemStockIndexes = std::vector<size_t>(),
			const std::vector<scalar_t>& switchingTimes = std::vector<scalar_t>(),
			const controller_array_t& controllersStock = controller_array_t(),
			const std::vector<scalar_array_t>& desiredTimeTrajectoriesStock = std::vector<scalar_array_t>(),
			const state_vector_array2_t& desiredStateTrajectoriesStock = state_vector_array2_t());

private:
	/**
	 * Finds nearest neighbour
	 * @param [in] enquiryParameter
	 * @return
	 */
	size_t findNearestController(const Eigen::VectorXd& enquiryParameter) const;

	/**
	 * Calculates linear equality constraint
	 * @param [out] Am
	 * @param [out] Bv
	 */
	void calculateLinearEqualityConstraint(Eigen::MatrixXd& Am, Eigen::VectorXd& Bv) override;

	/**
	 * Calculates gradient
	 * @param [in] id
	 * @param [in] parameters
	 * @param [out] gradient
	 * @return
	 */
	bool calculateGradient(const size_t& id, const Eigen::VectorXd& parameters, Eigen::VectorXd& gradient) override;

	/**
	 * Calculates cost
	 * @param [in] id
	 * @param [in] parameters
	 * @param [out] cost
	 * @return
	 */
	bool calculateCost(const size_t& id, const Eigen::VectorXd& parameters, double& cost) override;

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
	void rewindOptimizer(const size_t& firstIndex, bool initRun = false);

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
	void saveToBag(size_t id, const Eigen::VectorXd& parameters);

	/**
	 * Calculates initial controller
	 * @param [in] initState
	 * @param [in] switchingTimes
	 * @param [out] controllersStock
	 */
	void calculateInitialController(const state_vector_t& initState, const scalar_array_t& switchingTimes,
			controller_array_t& controllersStock);

	/**
	 * Set up optimizer
	 */
	void setupOptimizer();

private:
	Options_t options_;

	std::vector<std::shared_ptr<ControlledSystemBase<STATE_DIM, INPUT_DIM> > >  subsystemDynamicsPtr_;
	std::vector<std::shared_ptr<DerivativesBase<STATE_DIM, INPUT_DIM> > > 		subsystemDerivativesPtr_;
	std::vector<std::shared_ptr<CostFunctionBaseOCS2<STATE_DIM, INPUT_DIM> > > 	subsystemCostFunctionsPtr_;
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
	control_vector_array2_t optimizedInputTrajectoriesStock_;
	Eigen::VectorXd 		costFuntionDerivative_;

	scalar_t currentTotalCost_;
	Eigen::VectorXd currentCostFuntionDerivative_;

	std::vector<Eigen::VectorXd> parameterBag_;
	std::vector<controller_array_t, Eigen::aligned_allocator<controller_array_t> > controllersStockBag_;

	gslqp_ptr_t gslqpSolver_;
	std::vector<slqp_base_ptr_t> slqpSolverPtrs_;

	eigen_scalar_array_t slqIterationCost_;
	eigen_scalar_array_t slqIterationISE1_;
};

}  // end of ocs2 namespace

#include "implementation/OCS2Projected.h"

#endif /* OCS2_OCS2PROJECTED_H_ */
