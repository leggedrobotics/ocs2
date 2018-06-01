/*
 * Implementation of SLQ.h
 *
 *  Created on: Jan 5, 2016
 *      Author: farbod
 */


namespace ocs2{

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
SLQ<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::SLQ(
		const controlled_system_base_t* systemDynamicsPtr,
		const derivatives_base_t* systemDerivativesPtr,
		const constraint_base_t* systemConstraintsPtr,
		const cost_function_base_t* costFunctionPtr,
		const operating_trajectories_base_t* operatingTrajectoriesPtr,
		const SLQ_Settings& settings /*= SLQ_Settings()*/,
		const LOGIC_RULES_T* logicRulesPtr /*= nullptr*/,
		const cost_function_base_t* heuristicsFunctionPtr /*= nullptr*/)

	: BASE(systemDynamicsPtr, systemDerivativesPtr, systemConstraintsPtr, costFunctionPtr, operatingTrajectoriesPtr,
			settings, logicRulesPtr, heuristicsFunctionPtr)
{}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
SLQ<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::~SLQ()
{}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQ<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::approximatePartitionLQ(const size_t& partitionIndex)  {

	const size_t threadId = 0;
	size_t N = BASE::nominalTimeTrajectoriesStock_[partitionIndex].size();

	if (N > 0) {
		for (size_t k=0; k<N; k++) {
			// execute approximateLQWorker for the given partition and time node index (k)
			BASE::approximateLQWorker(threadId, partitionIndex, k);
		} // end of k loop
	}
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQ<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::calculatePartitionController(const size_t& partitionIndex)  {

	const size_t threadId = 0;
	size_t N = BASE::SsTimeTrajectoryStock_[partitionIndex].size();

	if (N > 0) {

		for (size_t k=0; k<N; k++) {
			// execute calculateControllerWorker for the given partition and time node index (k)
			BASE::calculateControllerWorker(threadId, partitionIndex, k);
		} // end of k loop

	}
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQ<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::lineSearch(bool computeISEs)  {

	const size_t workerIndex = 0;

	BASE::lsComputeISEs_ = computeISEs;

	// display
	if (BASE::settings_.displayInfo_)  {
		scalar_t maxDeltaUffNorm, maxDeltaUeeNorm;
		BASE::calculateControllerUpdateMaxNorm(maxDeltaUffNorm, maxDeltaUeeNorm);

		std::cerr << "max feedforward update norm:  " << maxDeltaUffNorm << std::endl;
		std::cerr << "max type-1 error update norm: " << maxDeltaUeeNorm << std::endl;
	}

	// perform one rollout while the input correction for the type-1 constraint is considered.
	BASE::rolloutTrajectory(BASE::initTime_, BASE::initState_, BASE::finalTime_,
			BASE::partitioningTimes_, BASE::nominalControllersStock_,
			BASE::nominalTimeTrajectoriesStock_, BASE::nominalEventsPastTheEndIndecesStock_,
			BASE::nominalStateTrajectoriesStock_, BASE::nominalInputTrajectoriesStock_);

	if (BASE::lsComputeISEs_==true) {
		// calculate constraint
		BASE::calculateRolloutConstraints(BASE::nominalTimeTrajectoriesStock_, BASE::nominalEventsPastTheEndIndecesStock_,
				BASE::nominalStateTrajectoriesStock_, BASE::nominalInputTrajectoriesStock_,
				BASE::nc1TrajectoriesStock_, BASE::EvTrajectoryStock_,BASE::nc2TrajectoriesStock_,
				BASE::HvTrajectoryStock_, BASE::nc2FinalStock_, BASE::HvFinalStock_);
		// calculate constraint type-1 ISE and maximum norm
		BASE::nominalConstraint1MaxNorm_ = BASE::calculateConstraintISE(
				BASE::nominalTimeTrajectoriesStock_, BASE::nc1TrajectoriesStock_, BASE::EvTrajectoryStock_,
				BASE::nominalConstraint1ISE_);
		// calculates type-2 constraint ISE and maximum norm
		BASE::nominalConstraint2MaxNorm_ = BASE::calculateConstraintISE(
				BASE::nominalTimeTrajectoriesStock_, BASE::nc2TrajectoriesStock_, BASE::HvTrajectoryStock_,
				BASE::nominalConstraint2ISE_);
		// calculates cost
		BASE::calculateRolloutCost(BASE::nominalTimeTrajectoriesStock_, BASE::nominalEventsPastTheEndIndecesStock_,
				BASE::nominalStateTrajectoriesStock_, BASE::nominalInputTrajectoriesStock_,
				BASE::nominalConstraint2ISE_, BASE::nc2FinalStock_, BASE::HvFinalStock_,
				BASE::nominalTotalCost_);
	} else {
		// calculate constraint type-1 ISE and maximum norm
		BASE::nominalConstraint1ISE_ = BASE::nominalConstraint1MaxNorm_ = 0.0;
		// calculates type-2 constraint ISE and maximum norm
		BASE::nominalConstraint2ISE_ = BASE::nominalConstraint2MaxNorm_ = 0.0;
		// calculates cost
		BASE::calculateRolloutCost(BASE::nominalTimeTrajectoriesStock_, BASE::nominalEventsPastTheEndIndecesStock_,
				BASE::nominalStateTrajectoriesStock_, BASE::nominalInputTrajectoriesStock_,
				BASE::nominalTotalCost_);
	}

	// display
	if (BASE::settings_.displayInfo_)  {
		std::cerr << "\t learningRate 0.0 \t cost: " << BASE::nominalTotalCost_ << " \t constraint ISE: " << BASE::nominalConstraint1ISE_ << std::endl;
		std::cerr << "\t final constraint type-2:  ";
		size_t itr = 0;
		for(size_t i=BASE::initActivePartition_; i<=BASE::finalActivePartition_; i++)
			for (size_t k=0; k<BASE::nc2FinalStock_[i].size(); k++) {
				std::cerr << "[" << itr  << "]: " << BASE::HvFinalStock_[i][k].head(BASE::nc2FinalStock_[i][k]).transpose() << ",  ";
				itr++;
			}
		std::cerr << std::endl;
	}

	scalar_t learningRate = BASE::maxLearningRate_;
	BASE::initLScontrollersStock_ = BASE::nominalControllersStock_;

	// local search forward simulation's variables
	scalar_t lsTotalCost;
	scalar_t lsConstraint1ISE, lsConstraint2ISE;
	scalar_t lsConstraint1MaxNorm, lsConstraint2MaxNorm;
	controller_array_t			lsControllersStock(BASE::numPartitionings_);
	std::vector<scalar_array_t>	lsTimeTrajectoriesStock(BASE::numPartitionings_);
	std::vector<size_array_t>	lsEventsPastTheEndIndecesStock(BASE::numPartitionings_);
	state_vector_array2_t   	lsStateTrajectoriesStock(BASE::numPartitionings_);
	input_vector_array2_t 		lsInputTrajectoriesStock(BASE::numPartitionings_);

	while (learningRate >= BASE::settings_.minLearningRateGSLQP_)  {

		// do a line search
		lsControllersStock = BASE::initLScontrollersStock_;
		BASE::lineSearchWorker(workerIndex, learningRate,
				lsTotalCost,
				lsConstraint1ISE, lsConstraint1MaxNorm,
				lsConstraint2ISE, lsConstraint2MaxNorm,
				lsControllersStock,
				lsTimeTrajectoriesStock, lsEventsPastTheEndIndecesStock,
				lsStateTrajectoriesStock, lsInputTrajectoriesStock);

		// break condition 1: it exits with largest learningRate that its cost is smaller than nominal cost.
		if (lsTotalCost < BASE::nominalTotalCost_*(1-1e-3*learningRate))
			break;  // exit while loop
		else
			learningRate = BASE::settings_.lineSearchContractionRate_*learningRate;

	}  // end of while


	if (learningRate >= BASE::settings_.minLearningRateGSLQP_)  {
		BASE::learningRateStar_ = learningRate;
		BASE::nominalTotalCost_ = lsTotalCost;
		BASE::nominalConstraint1ISE_ 	 = lsConstraint1ISE;
		BASE::nominalConstraint1MaxNorm_ = lsConstraint1MaxNorm;
		BASE::nominalConstraint2ISE_ 	 = lsConstraint2ISE;
		BASE::nominalConstraint2MaxNorm_ = lsConstraint2MaxNorm;

		BASE::nominalControllersStock_.swap(lsControllersStock);
		BASE::nominalTimeTrajectoriesStock_.swap(lsTimeTrajectoriesStock);
		BASE::nominalEventsPastTheEndIndecesStock_.swap(lsEventsPastTheEndIndecesStock);
		BASE::nominalStateTrajectoriesStock_.swap(lsStateTrajectoriesStock);
		BASE::nominalInputTrajectoriesStock_.swap(lsInputTrajectoriesStock);

	} else // since the open loop input is not change, the nominal trajectories will be unchanged
		BASE::learningRateStar_ = 0.0;

	// clear the feedforward increments
	for (size_t i=0; i<BASE::numPartitionings_; i++)
		BASE::nominalControllersStock_[i].deltaUff_.clear();

	// display
	if (BASE::settings_.displayInfo_)
		std::cerr << "The chosen learningRate is: " << BASE::learningRateStar_ << std::endl;
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQ<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::solveSequentialRiccatiEquations(
		const state_matrix_t& SmFinal,
		const state_vector_t& SvFinal,
		const eigen_scalar_t& sFinal)  {

	// for all partition, there is only one worker
	const size_t workerIndex = 0;

	BASE::SmFinalStock_[BASE::finalActivePartition_]  = SmFinal;
	BASE::SvFinalStock_[BASE::finalActivePartition_]  = SvFinal;
	BASE::SveFinalStock_[BASE::finalActivePartition_] = state_vector_t::Zero();
	BASE::sFinalStock_[BASE::finalActivePartition_]   = sFinal;

	for (int i=BASE::numPartitionings_-1; i>=0; i--) {

		if (i< (signed)BASE::initActivePartition_ || i > (signed)BASE::finalActivePartition_) {

			BASE::SsTimeTrajectoryStock_[i].clear();
			BASE::SsNormalizedTimeTrajectoryStock_[i].clear();
			BASE::SsNormalizedEventsPastTheEndIndecesStock_[i].clear();
			BASE::SmTrajectoryStock_[i].clear();
			BASE::SvTrajectoryStock_[i].clear();
			BASE::SveTrajectoryStock_[i].clear();
			BASE::sTrajectoryStock_[i].clear();

			BASE::SmFinalStock_[i].setZero();
			BASE::SvFinalStock_[i].setZero();
			BASE::SveFinalStock_[i].setZero();
			BASE::sFinalStock_[i].setZero();
			BASE::xFinalStock_[i].setZero();

			continue;
		}

		if (BASE::settings_.useRiccatiSolver_==true) {
			BASE::solveSlqRiccatiEquationsWorker(workerIndex, i,
					BASE::SmFinalStock_[i], BASE::SvFinalStock_[i], BASE::sFinalStock_[i], BASE::SveFinalStock_[i]);
		} else {
			scalar_t constraintStepSize = BASE::updateFeedForwardPoliciesStock_[i] ? BASE::settings_.constraintStepSize_ : 0.0;
			BASE::fullRiccatiBackwardSweepWorker(workerIndex, i,
					BASE::SmFinalStock_[i], BASE::SvFinalStock_[i], BASE::SveFinalStock_[i], BASE::sFinalStock_[i],
					constraintStepSize);
		}

		// set the final value for next Riccati equation
		if (i>BASE::initActivePartition_) {
			BASE::SmFinalStock_[i-1]  = BASE::SmTrajectoryStock_[i].front();
			BASE::SvFinalStock_[i-1]  = BASE::SvTrajectoryStock_[i].front();
			BASE::SveFinalStock_[i-1] = BASE::SveTrajectoryStock_[i].front();
			BASE::sFinalStock_[i-1]   = BASE::sTrajectoryStock_[i].front();
			BASE::xFinalStock_[i-1]	  = BASE::nominalStateTrajectoriesStock_[i].front();
		}
	}  // end of i loop

}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQ<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::runInit() {

	// run BASE routine
	BASE::runInit();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQ<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::runIteration()  {

	// run BASE routine
	BASE::runIteration();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQ<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::runExit()  {

	// linearizing the dynamics and quadratizing the cost function along nominal trajectories
	BASE::approximateOptimalControlProblem();

	// solve Riccati equations
	solveSequentialRiccatiEquations(BASE::SmHeuristics_, BASE::SvHeuristics_, BASE::sHeuristics_);

	// calculate the nominal co-state
	BASE::calculateRolloutCostate(BASE::nominalTimeTrajectoriesStock_, BASE::nominalStateTrajectoriesStock_, BASE::nominalcostateTrajectoriesStock_);

}


} // namespace ocs2
