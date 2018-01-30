/*
 * Implementation of SLQP.h
 *
 *  Created on: Jan 5, 2016
 *      Author: farbod, markus
 */


namespace ocs2{

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
SLQP<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::~SLQP()
{
#ifdef BENCHMARK
	std::cout<<"Avg time for approximateOptimalControlProblem :  "<< tAvg1<<" ms "<< std::endl;
	std::cout<<"Avg time for riccati :  "<< tAvg2<<" ms "<< std::endl;
	std::cout<<"Avg time for linearsearch :  "<< tAvg3<<" ms "<< std::endl;
#endif
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQP<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::approximatePartitionLQ(const size_t& partitionIndex)  {

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
void SLQP<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::calculatePartitionController(const size_t& partitionIndex)  {

	const size_t threadId = 0;
	size_t N = BASE::SsTimeTrajectoryStock_[partitionIndex].size();

	if (N > 0) {

		for (size_t k=0; k<N; k++) {
			// execute calculateControllerWorker for the given partition and time node index (k)
			BASE::calculateControllerWorker(threadId, partitionIndex, k,
					BASE::constraintStepSizesStock_[partitionIndex]);
		} // end of k loop

	}
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQP<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::lineSearch(scalar_t maxLearningRateStar/*=1.0*/)  {

	const size_t workerIndex = 0;

	BASE::maxLearningRate_ = maxLearningRateStar;

	// display
	if (BASE::options_.dispayGSLQP_)  {
		// less-equal operator for eigen vectors
		auto eigenVectorLessEqual = [] (const control_vector_t& u1, const control_vector_t& u2){ return u1.norm() < u2.norm(); };

		control_vector_array_t maxDeltaUffStock(BASE::finalActiveSubsystem_-BASE::initActiveSubsystem_+1);
		for (size_t i=BASE::initActiveSubsystem_; i<=BASE::finalActiveSubsystem_; i++)  {
			maxDeltaUffStock[i-BASE::initActiveSubsystem_]  = *std::max_element(BASE::nominalControllersStock_[i].deltaUff_.begin(), BASE::nominalControllersStock_[i].deltaUff_.template end(), eigenVectorLessEqual);
		}
		control_vector_t maxDeltaUff  = *std::max_element(maxDeltaUffStock.begin(), maxDeltaUffStock.end(), eigenVectorLessEqual);
		std::cerr << "max delta_uff norm: " << maxDeltaUff.norm()  << std::endl;
	}

	// perform one rollout while the input correction for the type-1 constraint is considered.
	BASE::rolloutTrajectory(BASE::initTime_, BASE::initState_, BASE::finalTime_,
			BASE::partitioningTimes_, BASE::nominalControllersStock_,
			BASE::nominalTimeTrajectoriesStock_, BASE::nominalEventsPastTheEndIndecesStock_,
			BASE::nominalStateTrajectoriesStock_, BASE::nominalInputTrajectoriesStock_);
	BASE::calculateRolloutConstraints(BASE::nominalTimeTrajectoriesStock_, BASE::nominalEventsPastTheEndIndecesStock_,
			BASE::nominalStateTrajectoriesStock_, BASE::nominalInputTrajectoriesStock_,
			BASE::nc1TrajectoriesStock_, BASE::EvTrajectoryStock_, BASE::nc2TrajectoriesStock_,
			BASE::HvTrajectoryStock_, BASE::nc2FinalStock_, BASE::HvFinalStock_);
	BASE::calculateRolloutCost(BASE::nominalTimeTrajectoriesStock_, BASE::nominalEventsPastTheEndIndecesStock_,
			BASE::nominalStateTrajectoriesStock_, BASE::nominalInputTrajectoriesStock_,
			BASE::nc2TrajectoriesStock_, BASE::HvTrajectoryStock_, BASE::nc2FinalStock_, BASE::HvFinalStock_,
			BASE::nominalTotalCost_);

	// display
	if (BASE::options_.dispayGSLQP_)  {
		this->calculateConstraintISE(BASE::nominalTimeTrajectoriesStock_, BASE::nc1TrajectoriesStock_, BASE::EvTrajectoryStock_, BASE::nominalConstraint1ISE_);

		std::cerr << "\t learningRate 0.0 \t cost: " << BASE::nominalTotalCost_ << " \t constraint ISE: " << BASE::nominalConstraint1ISE_ << std::endl;
		std::cerr << "\t final constraint type-2:  ";
		size_t itr = 0;
		for(size_t i=BASE::initActiveSubsystem_; i<=BASE::finalActiveSubsystem_; i++)
			for (size_t k=0; k<BASE::nc2FinalStock_[i].size(); k++) {
				std::cerr << "[" << itr  << "]: " << BASE::HvFinalStock_[i][k].head(BASE::nc2FinalStock_[i][k]).transpose() << ",  ";
				itr++;
			}
		std::cerr << std::endl;

	}
	scalar_t learningRate = BASE::maxLearningRate_;
	BASE::initLScontrollersStock_ = BASE::nominalControllersStock_;

	// local search forward simulation's variables
	scalar_t 					lsTotalCost;
	controller_array_t			lsControllersStock(BASE::numPartitionings_);
	std::vector<scalar_array_t>	lsTimeTrajectoriesStock(BASE::numPartitionings_);
	std::vector<size_array_t>	lsEventsPastTheEndIndecesStock(BASE::numPartitionings_);
	state_vector_array2_t   	lsStateTrajectoriesStock(BASE::numPartitionings_);
	control_vector_array2_t 	lsInputTrajectoriesStock(BASE::numPartitionings_);
	std::vector<size_array_t>   lsNc1TrajectoriesStock(BASE::numPartitionings_);
	constraint1_vector_array2_t lsEvTrajectoryStock(BASE::numPartitionings_);
	std::vector<size_array_t>   lsNc2TrajectoriesStock(BASE::numPartitionings_);
	constraint2_vector_array2_t lsHvTrajectoryStock(BASE::numPartitionings_);
	std::vector<size_array_t>	lsNc2FinalStock(BASE::numPartitionings_);
	constraint2_vector_array2_t	lsHvFinalStock(BASE::numPartitionings_);

	while (learningRate >= BASE::options_.minLearningRateGSLQP_)  {

		// do a line search
		lsControllersStock = BASE::initLScontrollersStock_;
		BASE::lineSearchWorker(workerIndex, learningRate,
				lsTotalCost, lsControllersStock,
				lsTimeTrajectoriesStock, lsEventsPastTheEndIndecesStock,
				lsStateTrajectoriesStock, lsInputTrajectoriesStock,
				lsNc1TrajectoriesStock, lsEvTrajectoryStock,
				lsNc2TrajectoriesStock, lsHvTrajectoryStock,
				lsNc2FinalStock, lsHvFinalStock);

		// break condition 1: it exits with largest learningRate that its cost is smaller than nominal cost.
		if (lsTotalCost < BASE::nominalTotalCost_*(1-1e-3*learningRate))
			break;  // exit while loop
		else
			learningRate = BASE::options_.lineSearchContractionRate_*learningRate;

	}  // end of while


	if (learningRate >= BASE::options_.minLearningRateGSLQP_)  {
		BASE::nominalTotalCost_ = lsTotalCost;
		BASE::learningRateStar_ = learningRate;

		BASE::nominalControllersStock_.swap(lsControllersStock);
		BASE::nominalTimeTrajectoriesStock_.swap(lsTimeTrajectoriesStock);
		BASE::nominalEventsPastTheEndIndecesStock_.swap(lsEventsPastTheEndIndecesStock);
		BASE::nominalStateTrajectoriesStock_.swap(lsStateTrajectoriesStock);
		BASE::nominalInputTrajectoriesStock_.swap(lsInputTrajectoriesStock);
		BASE::nc1TrajectoriesStock_.swap(lsNc1TrajectoriesStock);
		BASE::EvTrajectoryStock_.swap(lsEvTrajectoryStock);
		BASE::nc2TrajectoriesStock_.swap(lsNc2TrajectoriesStock);
		BASE::HvTrajectoryStock_.swap(lsHvTrajectoryStock);
		BASE::nc2FinalStock_.swap(lsNc2FinalStock);
		BASE::HvFinalStock_.swap(lsHvFinalStock);

	} else // since the open loop input is not change, the nominal trajectories will be unchanged
		BASE::learningRateStar_ = 0.0;

	// clear the feedforward increments
	for (size_t i=0; i<BASE::numPartitionings_; i++)
		BASE::nominalControllersStock_[i].deltaUff_.clear();

	// display
	if (BASE::options_.dispayGSLQP_)  std::cerr << "The chosen learningRate is: " << BASE::learningRateStar_ << std::endl;
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQP<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::solveSequentialRiccatiEquations(const scalar_t& learningRate,
		const state_matrix_t& SmFinal, const state_vector_t& SvFinal, const eigen_scalar_t& sFinal)  {

	// for all partition, there is only one worker
	const size_t workerIndex = 0;

	BASE::SsTimeTrajectoryStock_.resize(BASE::numPartitionings_);
	BASE::SsNormalizedTimeTrajectoryStock_.resize(BASE::numPartitionings_);
	BASE::SsNormalizedEventsPastTheEndIndecesStock_.resize(BASE::numPartitionings_);
	BASE::sTrajectoryStock_.resize(BASE::numPartitionings_);
	BASE::SvTrajectoryStock_.resize(BASE::numPartitionings_);
	BASE::SveTrajectoryStock_.resize(BASE::numPartitionings_);
	BASE::SmTrajectoryStock_.resize(BASE::numPartitionings_);

	BASE::SmFinalStock_[BASE::finalActiveSubsystem_+1]  = SmFinal;
	BASE::SvFinalStock_[BASE::finalActiveSubsystem_+1]  = SvFinal;
	BASE::SveFinalStock_[BASE::finalActiveSubsystem_+1] = state_vector_t::Zero();
	BASE::sFinalStock_[BASE::finalActiveSubsystem_+1]   = sFinal;

	for (int i=BASE::numPartitionings_-1; i>=0; i--) {

		if (i< (signed)BASE::initActiveSubsystem_ || i > (signed)BASE::finalActiveSubsystem_) {

			BASE::SsTimeTrajectoryStock_[i].clear();
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

		if (BASE::options_.useRiccatiSolver_==true) {
			BASE::solveRiccatiEquationsWorker(workerIndex, i,
					learningRate, BASE::SmFinalStock_[i+1], BASE::SvFinalStock_[i+1], BASE::sFinalStock_[i+1]);
			BASE::solveErrorRiccatiEquationWorker(workerIndex, i,
					BASE::SveFinalStock_[i+1]);
		} else {
			BASE::fullRiccatiBackwardSweepWorker(workerIndex, i,
					BASE::SmFinalStock_[i+1], BASE::SvFinalStock_[i+1], BASE::SveFinalStock_[i+1], BASE::sFinalStock_[i+1],
					BASE::constraintStepSizesStock_[i]);
		}

		// set the final value for next Riccati equation
		BASE::sFinalStock_[i]   = BASE::sTrajectoryStock_[i].front();
		BASE::SvFinalStock_[i]  = BASE::SvTrajectoryStock_[i].front();
		BASE::SveFinalStock_[i] = BASE::SveTrajectoryStock_[i].front();
		BASE::SmFinalStock_[i]  = BASE::SmTrajectoryStock_[i].front();

	}

	// state at the switching times
	BASE::xFinalStock_[BASE::finalActiveSubsystem_+1] = BASE::nominalStateTrajectoriesStock_[BASE::finalActiveSubsystem_].back();
	for (size_t i=BASE::initActiveSubsystem_; i<=BASE::finalActiveSubsystem_; i++)
		BASE::xFinalStock_[i] = BASE::nominalStateTrajectoriesStock_[i].front();

}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQP<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::runInit() {

	// set the start and final time for costFuntions
	for(size_t i=0; i<BASE::numPartitionings_; i++)
		BASE::subsystemCostFunctionsPtrStock_[i]->setTimePeriod(BASE::partitioningTimes_[i], BASE::partitioningTimes_[i+1]);

	// initial controller rollout
	BASE::rolloutTrajectory(BASE::initTime_, BASE::initState_, BASE::finalTime_,
			BASE::partitioningTimes_, BASE::nominalControllersStock_,
			BASE::nominalTimeTrajectoriesStock_, BASE::nominalEventsPastTheEndIndecesStock_,
			BASE::nominalStateTrajectoriesStock_, BASE::nominalInputTrajectoriesStock_);

	// calculate constraints
	BASE::calculateRolloutConstraints(BASE::nominalTimeTrajectoriesStock_, BASE::nominalEventsPastTheEndIndecesStock_,
			BASE::nominalStateTrajectoriesStock_, BASE::nominalInputTrajectoriesStock_,
			BASE::nc1TrajectoriesStock_, BASE::EvTrajectoryStock_,
			BASE::nc2TrajectoriesStock_, BASE::HvTrajectoryStock_, BASE::nc2FinalStock_, BASE::HvFinalStock_);

	// initial controller cost
	BASE::calculateRolloutCost(BASE::nominalTimeTrajectoriesStock_, BASE::nominalEventsPastTheEndIndecesStock_,
			BASE::nominalStateTrajectoriesStock_, BASE::nominalInputTrajectoriesStock_,
			BASE::nc2TrajectoriesStock_, BASE::HvTrajectoryStock_, BASE::nc2FinalStock_, BASE::HvFinalStock_,
			BASE::nominalTotalCost_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQP<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::runIteration(
		const scalar_t& maxLearningRateStar,
		const state_matrix_t& SmFinal /*= state_matrix_t::Zero()*/,
		const state_vector_t& SvFinal /*= state_vector_t::Zero()*/,
		const eigen_scalar_t& sFinal /*= eigen_scalar_t::Zero()*/)  {

#ifdef BENCHMARK
	// Benchmarking
	static size_t nIterations = 0;
	static scalar_t tAvg1, tAvg2, tAvg3 = 0.0;
	nIterations++;
	auto start = std::chrono::steady_clock::now();
#endif

	// linearizing the dynamics and quadratizing the cost function along nominal trajectories
	BASE::approximateOptimalControlProblem();

#ifdef BENCHMARK
	auto end = std::chrono::steady_clock::now();
	auto diff = end - start;
	tAvg1 = ((1.0 - 1.0/nIterations)* tAvg1) + (1.0/nIterations)*std::chrono::duration_cast<std::chrono::milliseconds>(diff).count();
	start = std::chrono::steady_clock::now();
#endif

	// solve Riccati equations
	solveSequentialRiccatiEquations(1.0 /*learningRate*/, SmFinal, SvFinal, sFinal);
	// calculate controller
	if (BASE::options_.useRiccatiSolver_==true)
		BASE::calculateController();

#ifdef BENCHMARK
	end = std::chrono::steady_clock::now();
	diff = end - start;
	tAvg2 = ((1.0 - 1.0/nIterations)* tAvg2) + (1.0/nIterations)*std::chrono::duration_cast<std::chrono::milliseconds>(diff).count();
	start = std::chrono::steady_clock::now();
#endif

	// finding the optimal learningRate
	lineSearch(maxLearningRateStar);

#ifdef BENCHMARK
	end = std::chrono::steady_clock::now();
	diff = end - start;
	tAvg3 = ((1.0 - 1.0/nIterations)* tAvg3) + (1.0/nIterations)*std::chrono::duration_cast<std::chrono::milliseconds>(diff).count();
#endif
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQP<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::runExit(const state_matrix_t& SmFinal /*= state_matrix_t::Zero()*/,
		const state_vector_t& SvFinal /*= state_vector_t::Zero()*/,
		const eigen_scalar_t& sFinal /*= eigen_scalar_t::Zero()*/)  {

	// linearizing the dynamics and quadratizing the cost function along nominal trajectories
	BASE::approximateOptimalControlProblem();

	// solve Riccati equations
	solveSequentialRiccatiEquations(0.0 /*learningRate*/, SmFinal, SvFinal, sFinal);

	// calculate the nominal co-state
	BASE::calculateRolloutCostate(BASE::nominalTimeTrajectoriesStock_, BASE::nominalStateTrajectoriesStock_, BASE::nominalcostateTrajectoriesStock_);

}


} // namespace ocs2
