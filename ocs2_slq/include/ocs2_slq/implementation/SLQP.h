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
void SLQP<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::calculateRolloutConstraints(
		const std::vector<scalar_array_t>& timeTrajectoriesStock,
		const std::vector<size_array_t>& eventsPastTheEndIndecesStock,
		const state_vector_array2_t& stateTrajectoriesStock,
		const control_vector_array2_t& inputTrajectoriesStock,
		std::vector<size_array_t>& nc1TrajectoriesStock,
		constraint1_vector_array2_t& EvTrajectoryStock,
		std::vector<size_array_t>& nc2TrajectoriesStock,
		constraint2_vector_array2_t& HvTrajectoryStock,
		std::vector<size_array_t>& nc2FinalValuesStock,
		constraint2_vector_array2_t& HvFinalStock) {

	// calculate constraint violations
	// constraint type 1 computations which consists of number of active constraints at each time point
	// and the value of the constraint (if the rollout is constrained the value is always zero otherwise
	// it is nonzero)
	nc1TrajectoriesStock.resize(BASE::options_.numPartitionings_);
	EvTrajectoryStock.resize(BASE::options_.numPartitionings_);

	// constraint type 2 computations which consists of number of active constraints at each time point
	// and the value of the constraint
	nc2TrajectoriesStock.resize(BASE::options_.numPartitionings_);
	HvTrajectoryStock.resize(BASE::options_.numPartitionings_);
	nc2FinalStock.resize(BASE::options_.numPartitionings_);
	HvFinalStock.resize(BASE::options_.numPartitionings_);

	for (size_t i=0; i<BASE::options_.numPartitionings_; i++) {

		BASE::calculateConstraintsWorker(i,
				timeTrajectoriesStock[i], eventsPastTheEndIndecesStock[i],
				stateTrajectoriesStock[i], inputTrajectoriesStock[i],
				nc1TrajectoriesStock[i], EvTrajectoryStock[i],
				nc2TrajectoriesStock[i], HvTrajectoryStock[i],
				nc2FinalValuesStock[i], HvFinalStock[i]);
	}  // end of i loop

}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQP<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::calculateRolloutCost(
		const std::vector<scalar_array_t>& timeTrajectoriesStock,
		const std::vector<size_array_t>& eventsPastTheEndIndecesStock,
		const state_vector_array2_t& stateTrajectoriesStock,
		const control_vector_array2_t& inputTrajectoriesStock,
		const std::vector<size_array_t>& nc2TrajectoriesStock,
		const constraint2_vector_array2_t& HvTrajectoryStock,
		const std::vector<size_array_t>& nc2FinalStock,
		const constraint2_vector_array2_t& HvFinalStock,
		scalar_t& totalCost) {

	calculateRolloutCost(timeTrajectoriesStock, eventsPastTheEndIndecesStock, stateTrajectoriesStock, inputTrajectoriesStock, totalCost);

	double stateConstraintPenalty = BASE::options_.stateConstraintPenaltyCoeff_ * pow(BASE::options_.stateConstraintPenaltyBase_, BASE::iteration_);

	size_t eventItr = 0;

	for (size_t i=0; i<BASE::options_.numPartitionings_; i++) {

		for (size_t k=0; k+1<timeTrajectoriesStock[i].size(); k++) {

			// integrates constraint type 2
			size_t nc2 = nc2TrajectoriesStock[i][k];
			if (nc2 > 0) {
				double dt = timeTrajectoriesStock[i][k+1]-timeTrajectoriesStock[i][k];
				totalCost += 0.5 * dt * stateConstraintPenalty * HvTrajectoryStock[i][k].head(nc2).squaredNorm();
			}

			// final constraint type 2
			if (eventItr!=eventsPastTheEndIndecesStock[i].size() && k+1==eventsPastTheEndIndecesStock[i][eventItr]) {
				size_t nc2Final = nc2FinalStock[i][eventItr];
				totalCost += 0.5 * stateConstraintPenalty * HvFinalStock[i][eventItr].head(nc2Final).squaredNorm();
				eventItr++;
			}

		}  // end of k loop

//		// final constraint type 2
//		size_t nc2Final = nc2FinalStock[i];
//		if (nc2Final>0)
//			totalCost += 0.5 * stateConstraintPenalty * HvFinalStock[i].head(nc2Final).squaredNorm();

	}  // end of i loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQP<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::calculateRolloutCost(
		const std::vector<scalar_array_t>& timeTrajectoriesStock,
		const std::vector<size_array_t>& eventsPastTheEndIndecesStock,
		const state_vector_array2_t& stateTrajectoriesStock,
		const control_vector_array2_t& inputTrajectoriesStock,
		scalar_t& totalCost)  {

	totalCost = 0.0;
	for (size_t i=0; i<BASE::options_.numPartitionings_; i++) {

		scalar_t cost;

		BASE::calculateCostWorker(i,
				timeTrajectoriesStock[i], eventsPastTheEndIndecesStock[i],
				stateTrajectoriesStock[i], inputTrajectoriesStock[i],
				cost);

		totalCost += cost;

	}  // end of i loop
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQP<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::approximatePartitionLQ(const size_t& i)  {

	size_t N = BASE::nominalTimeTrajectoriesStock_[i].size();

	if (N > 0) {

		// initialize subsystem i dynamics derivatives
		subsystemDerivativesPtrStock_[i]->initializeModel(BASE::logicRulesMachine_.getLogicRules(), BASE::partitioningTimes_,
				BASE::nominalStateTrajectoriesStock_[i].front(), i, "GSLPQ");
		// initialize subsystem i constraint
		subsystemConstraintsPtrStock_[i]->initializeModel(BASE::logicRulesMachine_.getLogicRules(), BASE::partitioningTimes_,
				BASE::nominalStateTrajectoriesStock_[i].front(), i, "GSLPQ");
		// initialize subsystem i cost
		subsystemCostFunctionsPtrStock_[i]->initializeModel(BASE::logicRulesMachine_.getLogicRules(), BASE::partitioningTimes_,
				BASE::nominalStateTrajectoriesStock_[i].front(), i, "GSLPQ");

		for (size_t k=0; k<N; k++) {
			BASE::approximateLQIntermediateTimeWorker(i, k, i);
		} // end of k loop

	}

	// if a switch took place calculate switch related variables
	size_t NE = BASE::nominalEventsPastTheEndIndecesStock_[i].size();

	for (size_t k=0; k<NE; k++) {
		BASE::approximateLQFinalTimeWorker(i, k, i);
	} // end of k loop

}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQP<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::calculateController() {

	// functions for controller and lagrane multiplier
	LinearInterpolation<state_vector_t,Eigen::aligned_allocator<state_vector_t> >     nominalStateFunc;
	LinearInterpolation<control_vector_t,Eigen::aligned_allocator<control_vector_t> > nominalInputFunc;

	LinearInterpolation<control_gain_matrix_t,Eigen::aligned_allocator<control_gain_matrix_t> > BmFunc;
	LinearInterpolation<control_feedback_t,Eigen::aligned_allocator<control_feedback_t> > PmFunc;
	LinearInterpolation<control_matrix_t,Eigen::aligned_allocator<control_matrix_t> >     RmInverseFunc;
	LinearInterpolation<control_vector_t,Eigen::aligned_allocator<control_vector_t> >     RvFunc;
	LinearInterpolation<control_vector_t,Eigen::aligned_allocator<control_vector_t> >     EvProjectedFunc;
	LinearInterpolation<control_feedback_t,Eigen::aligned_allocator<control_feedback_t> > CmProjectedFunc;
	LinearInterpolation<control_matrix_t,Eigen::aligned_allocator<control_matrix_t> >     DmProjectedFunc;

	for (size_t i=0; i<BASE::options_.numPartitionings_; i++) {

		if (i<BASE::initActiveSubsystem_ || i>BASE::finalActiveSubsystem_) {
			BASE::nominalControllersStock_[i].clear();
			continue;
		}

		// functions for controller and lagrane multiplier
		nominalStateFunc.reset();
		nominalStateFunc.setTimeStamp( &(BASE::nominalTimeTrajectoriesStock_[i]) );
		nominalStateFunc.setData( &(BASE::nominalStateTrajectoriesStock_[i]) );

		nominalInputFunc.reset();
		nominalInputFunc.setTimeStamp( &(BASE::nominalTimeTrajectoriesStock_[i]) );
		nominalInputFunc.setData( &(BASE::nominalInputTrajectoriesStock_[i]) );

		BmFunc.reset();
		BmFunc.setTimeStamp( &(BASE::nominalTimeTrajectoriesStock_[i]) );
		BmFunc.setData( &(BASE::BmTrajectoryStock_[i]) );

		PmFunc.reset();
		PmFunc.setTimeStamp( &(BASE::nominalTimeTrajectoriesStock_[i]) );
		PmFunc.setData( &(BASE::PmTrajectoryStock_[i]) );

		RmInverseFunc.reset();
		RmInverseFunc.setTimeStamp( &(BASE::nominalTimeTrajectoriesStock_[i]) );
		RmInverseFunc.setData( &(BASE::RmInverseTrajectoryStock_[i]) );

		RvFunc.reset();
		RvFunc.setTimeStamp( &(BASE::nominalTimeTrajectoriesStock_[i]) );
		RvFunc.setData( &(BASE::RvTrajectoryStock_[i]) );

		EvProjectedFunc.reset();
		EvProjectedFunc.setTimeStamp( &(BASE::nominalTimeTrajectoriesStock_[i]) );
		EvProjectedFunc.setData( &(BASE::EvProjectedTrajectoryStock_[i]) );

		CmProjectedFunc.reset();
		CmProjectedFunc.setTimeStamp( &(BASE::nominalTimeTrajectoriesStock_[i]) );
		CmProjectedFunc.setData( &(BASE::CmProjectedTrajectoryStock_[i]) );

		DmProjectedFunc.reset();
		DmProjectedFunc.setTimeStamp( &(BASE::nominalTimeTrajectoriesStock_[i]) );
		DmProjectedFunc.setData( &(BASE::DmProjectedTrajectoryStock_[i]) );

		size_t N = BASE::SsTimeTrajectoryStock_[i].size();

		BASE::nominalControllersStock_[i].time_ = BASE::SsTimeTrajectoryStock_[i];
		BASE::nominalControllersStock_[i].k_.resize(N);
		BASE::nominalControllersStock_[i].uff_.resize(N);
		BASE::nominalControllersStock_[i].deltaUff_.resize(N);

		for (size_t k=0; k<N; k++) {

			const double& time = BASE::SsTimeTrajectoryStock_[i][k];
			size_t greatestLessTimeStampIndex;

			state_vector_t nominalState;
			nominalStateFunc.interpolate(time, nominalState);
			greatestLessTimeStampIndex = nominalStateFunc.getGreatestLessTimeStampIndex();
			control_vector_t nominalInput;
			nominalInputFunc.interpolate(time, nominalInput, greatestLessTimeStampIndex);

			control_gain_matrix_t Bm;
			BmFunc.interpolate(time, Bm, greatestLessTimeStampIndex);
			control_feedback_t Pm;
			PmFunc.interpolate(time, Pm, greatestLessTimeStampIndex);
			control_vector_t Rv;
			RvFunc.interpolate(time, Rv, greatestLessTimeStampIndex);
			control_matrix_t RmInverse;
			RmInverseFunc.interpolate(time, RmInverse, greatestLessTimeStampIndex);
			control_vector_t EvProjected;
			EvProjectedFunc.interpolate(time, EvProjected, greatestLessTimeStampIndex);
			control_feedback_t CmProjected;
			CmProjectedFunc.interpolate(time, CmProjected, greatestLessTimeStampIndex);
			control_matrix_t DmProjected;
			DmProjectedFunc.interpolate(time, DmProjected, greatestLessTimeStampIndex);

			control_feedback_t Lm  = RmInverse * (Pm + Bm.transpose()*BASE::SmTrajectoryStock_[i][k]);
			control_vector_t   Lv  = RmInverse * (Rv + Bm.transpose()*BASE::SvTrajectoryStock_[i][k]);
			control_vector_t   Lve = RmInverse * (Bm.transpose()*BASE::SveTrajectoryStock_[i][k]);

			control_matrix_t DmNullProjection = control_matrix_t::Identity()-DmProjected;
			BASE::nominalControllersStock_[i].k_[k]   = -DmNullProjection*Lm - CmProjected;
			BASE::nominalControllersStock_[i].uff_[k] = nominalInput - BASE::nominalControllersStock_[i].k_[k]*nominalState
					- BASE::options_.constraintStepSize_ * (DmNullProjection*Lve + EvProjected);
			BASE::nominalControllersStock_[i].deltaUff_[k] = -DmNullProjection*Lv;

			// checking the numerical stability of the controller parameters
			if (BASE::options_.checkNumericalStability_==true) {
				try {
					if (BASE::nominalControllersStock_[i].k_[k] != BASE::nominalControllersStock_[i].k_[k])
						throw std::runtime_error("Feedback gains are unstable.");
					if (BASE::nominalControllersStock_[i].deltaUff_[k] != BASE::nominalControllersStock_[i].deltaUff_[k])
						throw std::runtime_error("feedForwardControl is unstable.");
				}
				catch(const std::exception& error)  {
					std::cerr << "what(): " << error.what() << " at time " << BASE::nominalControllersStock_[i].time_[k] << " [sec]." << std::endl;
				}
			}

		}  // end of k loop
	}  // end of i loop
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQP<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::lineSearch(scalar_t& learningRateStar, scalar_t maxLearningRateStar/*=1.0*/)  {

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
	rolloutTrajectory(BASE::initTime_, BASE::initState_, BASE::finalTime_, BASE::nominalControllersStock_,
			BASE::nominalTimeTrajectoriesStock_, BASE::nominalEventsPastTheEndIndecesStock_,
			BASE::nominalStateTrajectoriesStock_, BASE::nominalInputTrajectoriesStock_);
	calculateRolloutConstraints(BASE::nominalTimeTrajectoriesStock_, BASE::nominalEventsPastTheEndIndecesStock_,
			BASE::nominalStateTrajectoriesStock_, BASE::nominalInputTrajectoriesStock_,
			BASE::nc1TrajectoriesStock_, BASE::EvTrajectoryStock_, BASE::nc2TrajectoriesStock_,
			BASE::HvTrajectoryStock_, BASE::nc2FinalStock_, BASE::HvFinalStock_);
	calculateRolloutCost(BASE::nominalTimeTrajectoriesStock_, BASE::nominalEventsPastTheEndIndecesStock_,
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
	scalar_t learningRate = maxLearningRateStar;
	const controller_array_t controllersStock = BASE::nominalControllersStock_;

	// local search forward simulation's variables
	scalar_t lsTotalCost;
	controller_array_t			lsControllersStock(BASE::options_.numPartitionings_);
	std::vector<scalar_array_t>	lsTimeTrajectoriesStock(BASE::options_.numPartitionings_);
	std::vector<size_array_t>	lsEventsPastTheEndIndecesStock(BASE::options_.numPartitionings_);
	state_vector_array2_t   	lsStateTrajectoriesStock(BASE::options_.numPartitionings_);
	control_vector_array2_t 	lsInputTrajectoriesStock(BASE::options_.numPartitionings_);
	std::vector<size_array_t>   lsNc1TrajectoriesStock(BASE::options_.numPartitionings_);
	constraint1_vector_array2_t lsEvTrajectoryStock(BASE::options_.numPartitionings_);
	std::vector<size_array_t>   lsNc2TrajectoriesStock(BASE::options_.numPartitionings_);
	constraint2_vector_array2_t lsHvTrajectoryStock(BASE::options_.numPartitionings_);
	std::vector<size_array_t>	lsNc2FinalStock(BASE::options_.numPartitionings_);
	constraint2_vector_array2_t	lsHvFinalStock(BASE::options_.numPartitionings_);

	while (learningRate >= BASE::options_.minLearningRateGSLQP_)  {
		// modifying uff by the local increments
		lsControllersStock = controllersStock;
		for (size_t i=0; i<BASE::options_.numPartitionings_; i++)
			for (size_t k=0; k<lsControllersStock[i].time_.size(); k++)
				lsControllersStock[i].uff_[k] += learningRate*lsControllersStock[i].deltaUff_[k];

		// perform rollout
		try {
			rolloutTrajectory(BASE::initTime_, BASE::initState_, BASE::finalTime_, lsControllersStock,
					lsTimeTrajectoriesStock, lsEventsPastTheEndIndecesStock, lsStateTrajectoriesStock, lsInputTrajectoriesStock);
			calculateRolloutConstraints(lsTimeTrajectoriesStock, lsEventsPastTheEndIndecesStock, lsStateTrajectoriesStock, lsInputTrajectoriesStock,
					lsNc1TrajectoriesStock, lsEvTrajectoryStock,  lsNc2TrajectoriesStock, lsHvTrajectoryStock, lsNc2FinalStock, lsHvFinalStock);
			// calculate rollout cost
			calculateRolloutCost(lsTimeTrajectoriesStock, lsEventsPastTheEndIndecesStock, lsStateTrajectoriesStock, lsInputTrajectoriesStock,
					lsNc2TrajectoriesStock, lsHvTrajectoryStock, lsNc2FinalStock, lsHvFinalStock, lsTotalCost);

			// display
			if (BASE::options_.dispayGSLQP_) {
				scalar_t lsConstraint1ISE;
				this->calculateConstraintISE(lsTimeTrajectoriesStock, lsNc1TrajectoriesStock, lsEvTrajectoryStock, lsConstraint1ISE);
				std::cerr << "\t learningRate " << learningRate << " \t cost: " << lsTotalCost << " \t constraint ISE: " << lsConstraint1ISE << std::endl;
			}
		}
		catch(const std::exception& error)
		{
			std::cerr << "\t rollout with learningRate " << learningRate << " is terminated due to the slow simulation!" << std::endl;
			lsTotalCost  = std::numeric_limits<scalar_t>::max();
		}

		// break condition 1: it exits with largest learningRate that its cost is smaller than nominal cost.
		if (lsTotalCost < BASE::nominalTotalCost_*(1-1e-3*learningRate))
			break;  // exit while loop
		else
			learningRate = 0.5*learningRate;

	}  // end of while


	if (learningRate >= BASE::options_.minLearningRateGSLQP_)  {
		BASE::nominalTotalCost_ = lsTotalCost;
		learningRateStar = learningRate;

		// FIXME: swap one layer higher
		for (size_t i = 0; i<BASE::options_.numPartitionings_; i++)	// swapping where possible for efficiency
		{
			BASE::nominalControllersStock_[i].swap(lsControllersStock[i]);
			BASE::nominalTimeTrajectoriesStock_[i].swap(lsTimeTrajectoriesStock[i]);
			BASE::nominalEventsPastTheEndIndecesStock_[i].swap(lsEventsPastTheEndIndecesStock[i]);
			BASE::nominalStateTrajectoriesStock_[i].swap(lsStateTrajectoriesStock[i]);
			BASE::nominalInputTrajectoriesStock_[i].swap(lsInputTrajectoriesStock[i]);
			BASE::nc1TrajectoriesStock_[i].swap(lsNc1TrajectoriesStock[i]);
			BASE::EvTrajectoryStock_[i].swap(lsEvTrajectoryStock[i]);
			BASE::nc2TrajectoriesStock_[i].swap(lsNc2TrajectoriesStock[i]);
			BASE::HvTrajectoryStock_[i].swap(lsHvTrajectoryStock[i]);
			BASE::nc2FinalStock_[i].swap(lsNc2FinalStock[i]);
			BASE::HvFinalStock_[i].swap(lsHvFinalStock[i]);
		}

	} else // since the open loop input is not change, the nominal trajectories will be unchanged
		learningRateStar = 0.0;

	// clear the feedforward increments
	for (size_t i=0; i<BASE::options_.numPartitionings_; i++)
		BASE::nominalControllersStock_[i].deltaUff_.clear();

	// display
	if (BASE::options_.dispayGSLQP_)  std::cerr << "The chosen learningRate is: " << learningRateStar << std::endl;
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQP<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::solveSequentialRiccatiEquations(const scalar_t& learningRate,
		const state_matrix_t& SmFinal, const state_vector_t& SvFinal, const eigen_scalar_t& sFinal)  {

	BASE::SsTimeTrajectoryStock_.resize(BASE::options_.numPartitionings_);
	BASE::SsNormalizedTimeTrajectoryStock_.resize(BASE::options_.numPartitionings_);
	BASE::SsNormalizedEventsPastTheEndIndecesStock_.resize(BASE::options_.numPartitionings_);
	BASE::sTrajectoryStock_.resize(BASE::options_.numPartitionings_);
	BASE::SvTrajectoryStock_.resize(BASE::options_.numPartitionings_);
	BASE::SveTrajectoryStock_.resize(BASE::options_.numPartitionings_);
	BASE::SmTrajectoryStock_.resize(BASE::options_.numPartitionings_);

	BASE::SmFinalStock_[BASE::finalActiveSubsystem_+1]  = SmFinal;
	BASE::SvFinalStock_[BASE::finalActiveSubsystem_+1]  = SvFinal;
	BASE::SveFinalStock_[BASE::finalActiveSubsystem_+1] = state_vector_t::Zero();
	BASE::sFinalStock_[BASE::finalActiveSubsystem_+1]   = sFinal;

	for (int i=BASE::options_.numPartitionings_-1; i>=0; i--) {

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

		// for each partition, there is one worker
		const int& workerIndex = i;

		if (BASE::options_.useRiccatiSolver_==true) {
			BASE::solveRiccatiEquationsWorker(workerIndex, i,
					learningRate, BASE::SmFinalStock_[i+1], BASE::SvFinalStock_[i+1], BASE::sFinalStock_[i+1]);
			BASE::solveErrorRiccatiEquationWorker(workerIndex, i,
					BASE::SveFinalStock_[i+1]);
		} else {
			BASE::fullRiccatiBackwardSweepWorker(workerIndex, i,
					BASE::SmFinalStock_[i+1], BASE::SvFinalStock_[i+1], BASE::SveFinalStock_[i+1], BASE::sFinalStock_[i+1]);
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
void SLQP<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::setupOptimizer() {

	// call Base routine
	BASE::setupOptimizer();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQP<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::runInit() {

	// set the start and final time for costFuntions
	for(size_t i=0; i<BASE::options_.numPartitionings_; i++)
		subsystemCostFunctionsPtrStock_[i]->setTimePeriod(BASE::partitioningTimes_[i], BASE::partitioningTimes_[i+1]);

	// initial controller rollout
	rolloutTrajectory(BASE::initTime_, BASE::initState_, BASE::finalTime_, BASE::nominalControllersStock_,
			BASE::nominalTimeTrajectoriesStock_, BASE::nominalEventsPastTheEndIndecesStock_,
			BASE::nominalStateTrajectoriesStock_, BASE::nominalInputTrajectoriesStock_);

	// calculate constraints
	calculateRolloutConstraints(BASE::nominalTimeTrajectoriesStock_, BASE::nominalEventsPastTheEndIndecesStock_,
			BASE::nominalStateTrajectoriesStock_, BASE::nominalInputTrajectoriesStock_,
			BASE::nc1TrajectoriesStock_, BASE::EvTrajectoryStock_,
			BASE::nc2TrajectoriesStock_, BASE::HvTrajectoryStock_, BASE::nc2FinalStock_, BASE::HvFinalStock_);

	// run the an itration of the SLQ algorithm and update the member variables
	runIteration(0.0 /*maxLearningRateGSLQP*/);

	// initial controller cost
	calculateRolloutCost(BASE::nominalTimeTrajectoriesStock_, BASE::nominalEventsPastTheEndIndecesStock_,
			BASE::nominalStateTrajectoriesStock_, BASE::nominalInputTrajectoriesStock_,
			BASE::nc2TrajectoriesStock_, BASE::HvTrajectoryStock_, BASE::nc2FinalStock_, BASE::HvFinalStock_,
			BASE::nominalTotalCost_);

}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQP<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::runIteration(const scalar_t& maxLearningRateStar,
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
		calculateController();

#ifdef BENCHMARK
	end = std::chrono::steady_clock::now();
	diff = end - start;
	tAvg2 = ((1.0 - 1.0/nIterations)* tAvg2) + (1.0/nIterations)*std::chrono::duration_cast<std::chrono::milliseconds>(diff).count();
	start = std::chrono::steady_clock::now();
#endif

	// finding the optimal learningRate
	lineSearch(BASE::learningRateStar_, maxLearningRateStar);

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
