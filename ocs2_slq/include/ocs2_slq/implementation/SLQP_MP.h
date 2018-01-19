/*
 * Implementation of SLQP_MP.h
 *
 *  Created on: Jun 20, 2016
 *      Author: markus, farbod
 */


namespace ocs2{

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
SLQP_MP<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::~SLQP_MP()
{
	workersActive_ = false;
	workerTask_ = SHUTDOWN;

	workerWakeUpCondition_.notify_all();

	if(BASE::options_.debugPrintMP_)
		printString("Shutting down workers");

	for (size_t i=0; i<workerThreads_.size(); i++)
		workerThreads_[i].join();

	if(BASE::options_.debugPrintMP_)
		printString("All workers shut down");

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
void SLQP_MP<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::rolloutTrajectory(
		const scalar_t& initTime,
		const state_vector_t& initState,
		const scalar_t& finalTime,
		const controller_array_t& controllersStock,
		std::vector<scalar_array_t>& timeTrajectoriesStock,
		std::vector<size_array_t>& eventsPastTheEndIndecesStock,
		state_vector_array2_t& stateTrajectoriesStock,
		control_vector_array2_t& inputTrajectoriesStock)  {

	if (controllersStock.size() != BASE::options_.numPartitionings_)
		throw std::runtime_error("controllersStock has less controllers then the number of subsystems");

	timeTrajectoriesStock.resize(BASE::options_.numPartitionings_);
	eventsPastTheEndIndecesStock.resize(BASE::options_.numPartitionings_);
	stateTrajectoriesStock.resize(BASE::options_.numPartitionings_);
	inputTrajectoriesStock.resize(BASE::options_.numPartitionings_);

	// finding the active subsystem index at initTime
	size_t initActiveSubsystemIndex = BASE::findActiveSubsystemIndex(BASE::partitioningTimes_, initTime);
	// finding the active subsystem index at initTime
	size_t finalActiveSubsystemIndex = BASE::findActiveSubsystemIndex(BASE::partitioningTimes_, finalTime);

	scalar_t t0 = initTime;
	state_vector_t x0 = initState;
	scalar_t tf;
	for (size_t i=0; i<BASE::options_.numPartitionings_; i++)  {

		// for subsystems before the initial time
		if (i<initActiveSubsystemIndex  ||  i>finalActiveSubsystemIndex) {
			timeTrajectoriesStock[i].clear();
			eventsPastTheEndIndecesStock[i].clear();
			stateTrajectoriesStock[i].clear();
			inputTrajectoriesStock[i].clear();
			continue;
		}

		// final time
		tf = (i != finalActiveSubsystemIndex) ? BASE::partitioningTimes_[i+1] : finalTime;

		// use Base rolloutWorker to use the worker i
		BASE::rolloutWorker(i,
				t0, x0, tf, controllersStock[i], BASE::logicRulesMachine_.getSwitchingTimes(i),
				timeTrajectoriesStock[i], eventsPastTheEndIndecesStock[i], stateTrajectoriesStock[i], inputTrajectoriesStock[i]);

		// reset the initial time and state
		t0 = timeTrajectoriesStock[i].back();
		x0 = stateTrajectoriesStock[i].back();

		if (x0 != x0)  throw std::runtime_error("System became unstable during the SLQP rollout.");

	}  // end of i loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQP_MP<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::rolloutTrajectory(
		size_t threadId,
		const scalar_t& initTime,
		const state_vector_t& initState,
		const scalar_t& finalTime,
		const controller_array_t& controllersStock,
		std::vector<scalar_array_t>& timeTrajectoriesStock,
		std::vector<size_array_t>& eventsPastTheEndIndecesStock,
		state_vector_array2_t& stateTrajectoriesStock,
		control_vector_array2_t& inputTrajectoriesStock){

	if (controllersStock.size() != BASE::options_.numPartitionings_)
		throw std::runtime_error("controllersStock has less controllers then the number of subsystems");

	timeTrajectoriesStock.resize(BASE::options_.numPartitionings_);
	eventsPastTheEndIndecesStock.resize(BASE::options_.numPartitionings_);
	stateTrajectoriesStock.resize(BASE::options_.numPartitionings_);
	inputTrajectoriesStock.resize(BASE::options_.numPartitionings_);

	// finding the active subsystem index at initTime
	size_t initActiveSubsystemIndex = BASE::findActiveSubsystemIndex(BASE::partitioningTimes_, initTime);
	// finding the active subsystem index at initTime
	size_t finalActiveSubsystemIndex = BASE::findActiveSubsystemIndex(BASE::partitioningTimes_, finalTime);

	scalar_t t0 = initTime;
	state_vector_t x0 = initState;
	scalar_t tf;
	for (size_t i=0; i<BASE::options_.numPartitionings_; i++)
	{
		// for subsystems before the initial time
		if (i<initActiveSubsystemIndex  ||  i>finalActiveSubsystemIndex) {
			timeTrajectoriesStock[i].clear();
			eventsPastTheEndIndecesStock[i].clear();
			stateTrajectoriesStock[i].clear();
			inputTrajectoriesStock[i].clear();
			continue;
		}

		// final time
		tf = (i != finalActiveSubsystemIndex) ? BASE::partitioningTimes_[i+1] : finalTime;

		// use Base rolloutWorker to use the worker threadId
		BASE::rolloutWorker(threadId,
				t0, x0, tf, controllersStock[i], BASE::logicRulesMachine_.getSwitchingTimes(i),
				timeTrajectoriesStock[i], eventsPastTheEndIndecesStock[i], stateTrajectoriesStock[i], inputTrajectoriesStock[i]);

		// reset the initial state
		t0 = timeTrajectoriesStock[i].back();
		x0 = stateTrajectoriesStock[i].back();

		if (x0 != x0)  throw std::runtime_error("System became unstable during the SLQP_MP rollout.");
	}
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQP_MP<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::calculateRolloutConstraints(
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

	calculateRolloutConstraints(BASE::options_.nThreads_,
			timeTrajectoriesStock, eventsPastTheEndIndecesStock,
			stateTrajectoriesStock, inputTrajectoriesStock,
			nc1TrajectoriesStock, EvTrajectoryStock,
			nc2TrajectoriesStock, HvTrajectoryStock,
			nc2FinalValuesStock, HvFinalStock);
}

/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQP_MP<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::calculateRolloutConstraints(
		size_t threadId,
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

		BASE::calculateConstraintsWorker(threadId,
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
void SLQP_MP<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::calculateRolloutCost(
		const std::vector<scalar_array_t>& timeTrajectoriesStock,
		const std::vector<size_array_t>& eventsPastTheEndIndecesStock,
		const state_vector_array2_t& stateTrajectoriesStock,
		const control_vector_array2_t& inputTrajectoriesStock,
		scalar_t& totalCost) {

	calculateRolloutCost(BASE::options_.nThreads_,
			timeTrajectoriesStock, eventsPastTheEndIndecesStock,
			stateTrajectoriesStock, inputTrajectoriesStock,
			totalCost);
}

/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQP_MP<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::calculateRolloutCost(
		size_t threadId,
		const std::vector<scalar_array_t>& timeTrajectoriesStock,
		const std::vector<size_array_t>& eventsPastTheEndIndecesStock,
		const state_vector_array2_t& stateTrajectoriesStock,
		const control_vector_array2_t& inputTrajectoriesStock,
		scalar_t& totalCost)  {

	totalCost = 0.0;

	for (size_t i=0; i<BASE::options_.numPartitionings_; i++) {

		scalar_t cost;

		BASE::calculateCostWorker(threadId,
				timeTrajectoriesStock[i], eventsPastTheEndIndecesStock[i],
				stateTrajectoriesStock[i], inputTrajectoriesStock[i],
				cost);

		totalCost += cost;

	}  // end of i loop
}

/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQP_MP<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::calculateRolloutCost(
		const std::vector<scalar_array_t>& timeTrajectoriesStock,
		const std::vector<size_array_t>& eventsPastTheEndIndecesStock,
		const state_vector_array2_t& stateTrajectoriesStock,
		const control_vector_array2_t& inputTrajectoriesStock,
		const std::vector<size_array_t>& nc2TrajectoriesStock,
		const constraint2_vector_array2_t& HvTrajectoryStock,
		const std::vector<size_array_t>& nc2FinalStock,
		const constraint2_vector_array2_t& HvFinalStock,
		scalar_t& totalCost){

	calculateRolloutCost(BASE::options_.nThreads_,
			timeTrajectoriesStock, eventsPastTheEndIndecesStock,
			stateTrajectoriesStock, inputTrajectoriesStock,
			nc2TrajectoriesStock, HvTrajectoryStock, nc2FinalStock, HvFinalStock,
			totalCost);
}

/*****************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQP_MP<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::calculateRolloutCost(
		size_t threadId,
		const std::vector<scalar_array_t>& timeTrajectoriesStock,
		const std::vector<size_array_t>& eventsPastTheEndIndecesStock,
		const state_vector_array2_t& stateTrajectoriesStock,
		const control_vector_array2_t& inputTrajectoriesStock,
		const std::vector<size_array_t>& nc2TrajectoriesStock,
		const constraint2_vector_array2_t& HvTrajectoryStock,
		const std::vector<size_array_t>& nc2FinalStock,
		const constraint2_vector_array2_t& HvFinalStock,
		scalar_t& totalCost) {

	calculateRolloutCost(threadId,
			timeTrajectoriesStock, eventsPastTheEndIndecesStock,
			stateTrajectoriesStock, inputTrajectoriesStock,
			totalCost);

	const double stateConstraintPenalty = BASE::options_.stateConstraintPenaltyCoeff_ * pow(BASE::options_.stateConstraintPenaltyBase_, BASE::iteration_);

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
	}  // end of i loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQP_MP<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::lineSearch() {

	BASE::learningRateStar_ = 0.0;	// default learning rate as zero

	// display
	if (BASE::options_.dispayGSLQP_==true)
	{
		// less-equal operator for eigen vectors
		auto eigenVectorLessEqual = [] (const control_vector_t& u1, const control_vector_t& u2){ return u1.norm() < u2.norm(); };

		control_vector_array_t maxDeltaUffStock(BASE::finalActiveSubsystem_-BASE::initActiveSubsystem_+1);
		for (size_t i=BASE::initActiveSubsystem_; i<=BASE::finalActiveSubsystem_; i++)  {
			maxDeltaUffStock[i-BASE::initActiveSubsystem_]  = *std::max_element(
					BASE::nominalControllersStock_[i].deltaUff_.begin(), BASE::nominalControllersStock_[i].deltaUff_.template end(), eigenVectorLessEqual);
		}
		control_vector_t maxDeltaUff  = *std::max_element(maxDeltaUffStock.begin(), maxDeltaUffStock.end(), eigenVectorLessEqual);
		std::cerr << "max delta_uff norm: " << maxDeltaUff.norm()  << std::endl;
	}

	// perform one rollout while the input correction for the type-1 constraint is considered.
	rolloutTrajectory(BASE::options_.nThreads_,
			BASE::initTime_, BASE::initState_, BASE::finalTime_, BASE::nominalControllersStock_,
			BASE::nominalTimeTrajectoriesStock_, BASE::nominalEventsPastTheEndIndecesStock_,
			BASE::nominalStateTrajectoriesStock_, BASE::nominalInputTrajectoriesStock_);
	calculateRolloutConstraints(BASE::options_.nThreads_,
			BASE::nominalTimeTrajectoriesStock_, BASE::nominalEventsPastTheEndIndecesStock_,
			BASE::nominalStateTrajectoriesStock_, BASE::nominalInputTrajectoriesStock_,
			BASE::nc1TrajectoriesStock_, BASE::EvTrajectoryStock_,BASE::nc2TrajectoriesStock_,
			BASE::HvTrajectoryStock_, BASE::nc2FinalStock_, BASE::HvFinalStock_);
	calculateRolloutCost(BASE::options_.nThreads_,
			BASE::nominalTimeTrajectoriesStock_, BASE::nominalEventsPastTheEndIndecesStock_,
			BASE::nominalStateTrajectoriesStock_, BASE::nominalInputTrajectoriesStock_,
			BASE::nc2TrajectoriesStock_, BASE::HvTrajectoryStock_, BASE::nc2FinalStock_, BASE::HvFinalStock_,
			BASE::nominalTotalCost_);

	lowestTotalCost_  = BASE::nominalTotalCost_;

	// display
	if (BASE::options_.dispayGSLQP_) {
		this->calculateConstraintISE(BASE::nominalTimeTrajectoriesStock_, BASE::nc1TrajectoriesStock_, BASE::EvTrajectoryStock_, BASE::nominalConstraint1ISE_);
		std::cerr << "\t learningRate 0.0 \t cost: " << BASE::nominalTotalCost_ << " \t constraint ISE: " << BASE::nominalConstraint1ISE_ << std::endl;
	}

	initLScontrollersStock_ = BASE::nominalControllersStock_;		// this will serve to init the workers

	subsystemProcessed_ = 0; // not required for linesearch, but assign to not let it dangle around
	alphaProcessed_.clear();
	alphaTaken_ = 0;
	alphaBestFound_ = false;
	lsWorkerCompleted_ = 0;

	size_t maxNumOfLineSearches =  (int) (log(BASE::options_.minLearningRateGSLQP_/BASE::options_.maxLearningRateGSLQP_) / log(BASE::options_.lineSearchContractionRate_)) +1;
	alphaExpMax_ = maxNumOfLineSearches;
	alphaExpBest_ = maxNumOfLineSearches;
	alphaProcessed_.resize(maxNumOfLineSearches, 0);

	if(BASE::options_.debugPrintMP_)
		printString("[MP]: calculated maximum number of line searches " + std::to_string(alphaExpMax_));

	if(BASE::options_.debugPrintMP_)
		printString("[MP] Waking up workers for line search ");

	workerTask_ = LINE_SEARCH;

	std::unique_lock<std::mutex> lock (workerWakeUpMutex_);
	workerWakeUpCondition_.notify_all();
	lock.unlock();

	if(BASE::options_.debugPrintMP_)
		printString("[MP] Will sleep now until we have results ");


	std::unique_lock<std::mutex> waitLock(alphaBestFoundMutex_);
	while(lsWorkerCompleted_.load() < BASE::options_.nThreads_)
		alphaBestFoundCondition_.wait(waitLock);

	waitLock.unlock();

	workerTask_ = IDLE;

	if(BASE::options_.debugPrintMP_)
		printString("[MP]: Woke up again, should have results now.");

	// clear the feedforward increments
	for (size_t j=0; j<BASE::options_.nThreads_+1; j++)
		for (size_t i=0; i<BASE::options_.numPartitionings_; i++)
			BASE::nominalControllersStock_[i].deltaUff_.clear();

	// reset integrator events
	event_handler_t::DeactivateKillIntegration();	// reset all integrations

	if (BASE::options_.dispayGSLQP_)
		if(BASE::options_.lsStepsizeGreedy_ == true)
			printString("Finished step-size greedy linesearch.");
		else
			printString("Finished cost greedy linesearch.");

	// display
	if (BASE::options_.dispayGSLQP_)
		printString("The chosen learningRate is: " + std::to_string(BASE::learningRateStar_));

	BASE::nominalTotalCost_ =	lowestTotalCost_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQP_MP<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::launchWorkerThreads()
{
	workersActive_ = true;
	workerTask_ = IDLE;

	for (size_t i=0; i < BASE::options_.nThreads_; i++) {
		workerThreads_.push_back(std::thread(&SLQP_MP::threadWork, this, i));
	}
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQP_MP<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::threadWork(size_t threadId)
{
	if(BASE::options_.debugPrintMP_)
		printString("[Thread " + std::to_string(threadId) + "]: launched");

	// local variables
	size_t uniqueProcessID = 0;
	size_t subsystemProcessed_local = 0;
	size_t iteration_local = BASE::iteration_;
	int workerTask_local = IDLE;

	while(workersActive_)
	{
		subsystemProcessed_local = subsystemProcessed_.load();
		workerTask_local = workerTask_.load();
		iteration_local = BASE::iteration_;

		// display
		if(BASE::options_.debugPrintMP_){
			printString("[Thread " + std::to_string(threadId) + "]: previous procId: " + std::to_string(uniqueProcessID) +
					", current procId: " +std::to_string(generateUniqueProcessID(iteration_local, (int) workerTask_local, (int) subsystemProcessed_local)));
		}

		/* We want to put the worker to sleep if
		 * - the workerTask_ is IDLE
		 * - or we are finished both workerTask_ is not yet reset, thus the process ID is still the same
		 * */
		if ( workerTask_local == IDLE || uniqueProcessID == generateUniqueProcessID(iteration_local, (int) workerTask_local, (int) subsystemProcessed_local))
		{
			if(BASE::options_.debugPrintMP_)
				printString("[Thread " + std::to_string(threadId) + "]: going to sleep !");

			// sleep until the state is not IDLE any more and we have a different process ID than before
			std::unique_lock<std::mutex> waitLock(workerWakeUpMutex_);
			while(workerTask_ == IDLE ||  (uniqueProcessID == generateUniqueProcessID(BASE::iteration_, (int)workerTask_.load(), (int) subsystemProcessed_.load()))){
				workerWakeUpCondition_.wait(waitLock);
			}
			waitLock.unlock();

			subsystemProcessed_local = subsystemProcessed_.load();
			workerTask_local = workerTask_.load();
			iteration_local = BASE::iteration_;

			if(BASE::options_.debugPrintMP_)
				printString("[Thread " + std::to_string(threadId) + "]: woke up !");
		}

		if (!workersActive_)
			break;

		switch(workerTask_local)
		{
		case APPROXIMATE_LQ:
		{
			if(BASE::options_.debugPrintMP_)
				printString("[Thread " + std::to_string(threadId) + "]: now busy with APPROXIMATE_LQ on subsystem " + std::to_string(subsystemProcessed_local));

			approximateSubsystemLQWorker(threadId, subsystemProcessed_local);
			uniqueProcessID = generateUniqueProcessID (iteration_local, APPROXIMATE_LQ, subsystemProcessed_local);

			break;
		}
		case CALCULATE_CONTROLLER_AND_LAGRANGIAN:
		{
			if(BASE::options_.debugPrintMP_)
				printString("[Thread " + std::to_string(threadId) + "]: now busy with CALCULATE_CONTROLLER_AND_LAGRANGIAN !");

			calculateControllerWorker(threadId, subsystemProcessed_local);
			uniqueProcessID = generateUniqueProcessID (iteration_local, CALCULATE_CONTROLLER_AND_LAGRANGIAN, subsystemProcessed_local);

			break;
		}
		case LINE_SEARCH:
		{
			if(BASE::options_.debugPrintMP_)
				printString("[Thread " + std::to_string(threadId) + "]: now busy with LINE_SEARCH !");

			lineSearchWorker(threadId);
			uniqueProcessID = generateUniqueProcessID (iteration_local, LINE_SEARCH, subsystemProcessed_local);
			break;
		}
		case SOLVE_RICCATI:
		{
			if(BASE::options_.debugPrintMP_)
				printString("[Thread "+ std::to_string(threadId) +"]: now busy with RiccatiSolver!");
			uniqueProcessID = generateUniqueProcessID (iteration_local, SOLVE_RICCATI, subsystemProcessed_local);
			executeRiccatiSolver(threadId);
			break;
		}
		case SHUTDOWN:
		{
			if(BASE::options_.debugPrintMP_)
				printString("[Thread "+ std::to_string(threadId) +"]: now shutting down!");
			return;
		}
		}

		if(BASE::options_.debugPrintMP_)
			printString("[Thread " + std::to_string(threadId) +"]: done with job. Will wait for next now!");
	}
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
// FIXME: the final values are not computed in parallel
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQP_MP<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::approximatePartitionLQ(const size_t& i)
{
	subsystemProcessed_ = i;

	size_t N = BASE::nominalTimeTrajectoriesStock_[i].size();

	if (N > 0) {
		for(size_t j = 0; j< BASE::options_.nThreads_+1; j++) {
			// initialize subsystem i dynamics derivatives
			BASE::subsystemDerivativesPtrStock_[i]->initializeModel(BASE::logicRulesMachine_.getLogicRules(), BASE::partitioningTimes_,
					BASE::nominalStateTrajectoriesStock_[i].front(), i, "GSLPQ");
			// initialize subsystem i constraint
			BASE::subsystemConstraintsPtrStock_[i]->initializeModel(BASE::logicRulesMachine_.getLogicRules(), BASE::partitioningTimes_,
					BASE::nominalStateTrajectoriesStock_[i].front(), i, "GSLPQ");
			// initialize subsystem i cost
			BASE::subsystemCostFunctionsPtrStock_[i]->initializeModel(BASE::logicRulesMachine_.getLogicRules(), BASE::partitioningTimes_,
					BASE::nominalStateTrajectoriesStock_[i].front(), i, "GSLPQ");
		}

		kTaken_approx_[i] = 0;
		kCompleted_approx_[i]= 0;
		KMax_subsystem_approx_[i] = N;

		if(BASE::options_.debugPrintMP_)  printString("[MP]: Waking up workers to do linearisation for subsystem " + std::to_string(i));

		workerTask_ = APPROXIMATE_LQ;
		std::unique_lock<std::mutex> lock (workerWakeUpMutex_);
		workerWakeUpCondition_.notify_all();
		lock.unlock();

		if(BASE::options_.debugPrintMP_)  printString("[MP]: Will wait now until workers have linearized dynamics of subsystem " + std::to_string(i));

		std::unique_lock<std::mutex> waitLock(kCompletedMutex_);
		while(kCompleted_approx_[i].load() < KMax_subsystem_approx_[i]){
			kCompletedCondition_.wait(waitLock);
		}

		waitLock.unlock();
		workerTask_ = IDLE;
	}

	// TODO: this part of the code is not prallel
	// if a switch took place calculate switch related variables
	size_t NE = BASE::nominalEventsPastTheEndIndecesStock_[i].size();

	for (size_t k=0; k<NE; k++) {
		BASE::approximateLQFinalTimeWorker(i, k, i);
	}  // end of k loop

}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQP_MP<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::calculateController() {

	for (size_t i=0; i<BASE::options_.numPartitionings_; i++)  {

		if (i<BASE::initActiveSubsystem_ || i>BASE::finalActiveSubsystem_) {
			BASE::nominalControllersStock_[i].clear();
			continue;
		}

		subsystemProcessed_ =  i;

		kTaken_ctrl_[i] = 0;
		kCompleted_ctrl_[i] = 0;
		KMax_subsystem_ctrl_[i]  = BASE::SsTimeTrajectoryStock_[i].size(); // number of elements in the trajectory of this subsystem

		// initialize interpolators
		for(size_t n = 0; n< BASE::options_.nThreads_+1; n++) {

			// functions for controller and lagrange-multiplier
			nominalStateFunc_[n].reset();
			nominalStateFunc_[n].setTimeStamp( &(BASE::nominalTimeTrajectoriesStock_[i]) );
			nominalStateFunc_[n].setData( &(BASE::nominalStateTrajectoriesStock_[i]) );

			nominalInputFunc_[n].reset();
			nominalInputFunc_[n].setTimeStamp( &(BASE::nominalTimeTrajectoriesStock_[i]) );
			nominalInputFunc_[n].setData( &(BASE::nominalInputTrajectoriesStock_[i]) );

			BmFunc_[n].reset();
			BmFunc_[n].setTimeStamp( &(BASE::nominalTimeTrajectoriesStock_[i]) );
			BmFunc_[n].setData( &(BASE::BmTrajectoryStock_[i]) );

			PmFunc_[n].reset();
			PmFunc_[n].setTimeStamp( &(BASE::nominalTimeTrajectoriesStock_[i]) );
			PmFunc_[n].setData( &(BASE::PmTrajectoryStock_[i]) );

			RmInverseFunc_[n].reset();
			RmInverseFunc_[n].setTimeStamp( &(BASE::nominalTimeTrajectoriesStock_[i]) );
			RmInverseFunc_[n].setData( &(BASE::RmInverseTrajectoryStock_[i]) );

			RvFunc_[n].reset();
			RvFunc_[n].setTimeStamp( &(BASE::nominalTimeTrajectoriesStock_[i]) );
			RvFunc_[n].setData( &(BASE::RvTrajectoryStock_[i]) );

			EvProjectedFunc_[n].reset();
			EvProjectedFunc_[n].setTimeStamp( &(BASE::nominalTimeTrajectoriesStock_[i]) );
			EvProjectedFunc_[n].setData( &(BASE::EvProjectedTrajectoryStock_[i]) );

			CmProjectedFunc_[n].reset();
			CmProjectedFunc_[n].setTimeStamp( &(BASE::nominalTimeTrajectoriesStock_[i]) );
			CmProjectedFunc_[n].setData( &(BASE::CmProjectedTrajectoryStock_[i]) );

			DmProjectedFunc_[n].reset();
			DmProjectedFunc_[n].setTimeStamp( &(BASE::nominalTimeTrajectoriesStock_[i]) );
			DmProjectedFunc_[n].setData( &(BASE::DmProjectedTrajectoryStock_[i]) );
		}

		BASE::nominalControllersStock_[i].time_ = BASE::SsTimeTrajectoryStock_[i];
		BASE::nominalControllersStock_[i].k_.resize(KMax_subsystem_ctrl_[i]);
		BASE::nominalControllersStock_[i].uff_.resize(KMax_subsystem_ctrl_[i]);
		BASE::nominalControllersStock_[i].deltaUff_.resize(KMax_subsystem_ctrl_[i]);

		if(BASE::options_.debugPrintMP_)
			printString("[MP]: Waking up workers to calc. controller for subsystem " + std::to_string(i));

		workerTask_ = CALCULATE_CONTROLLER_AND_LAGRANGIAN;
		std::unique_lock<std::mutex> lock (workerWakeUpMutex_);
		workerWakeUpCondition_.notify_all();
		lock.unlock();

		if(BASE::options_.debugPrintMP_)
			printString("[MP]: Will wait now controllers have been calculated for subsystem " + std::to_string(i));


		std::unique_lock<std::mutex> waitLock(kCompletedMutex_);

		while(kCompleted_ctrl_[i].load() < KMax_subsystem_ctrl_[i] ){
			kCompletedCondition_.wait(waitLock);
		}

		waitLock.unlock();
		workerTask_ = IDLE;

		if(BASE::options_.debugPrintMP_)
			printString("[MP]: Back to main thread, workers should now have designed controllers for subsystem " + std::to_string(i));

	}  // end of i loop

}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
size_t SLQP_MP<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::approximateSubsystemLQWorker(size_t threadId, size_t subsystemProcessed)
{

	size_t k = 0;
	size_t kCompleted_local = 0;

	while(true)
	{
		k = kTaken_approx_[subsystemProcessed]++;

		if(k < KMax_subsystem_approx_[subsystemProcessed]){

			if(BASE::options_.debugPrintMP_){
				if (k%10 == 0) {
					printString("[Thread " + std::to_string(threadId) + "], subsystem " + std::to_string(subsystemProcessed)
					+ ":Start approximating system LQ on index k = " + std::to_string(k) + " out of " + std::to_string(KMax_subsystem_approx_[subsystemProcessed]-1));
				}
			}

//			executeApproximateSubsystemLQ(threadId, k, subsystemProcessed);
			BASE::approximateLQIntermediateTimeWorker(threadId, subsystemProcessed, k);

			kCompleted_local = ++kCompleted_approx_[subsystemProcessed];
		}

		if (k >= KMax_subsystem_approx_[subsystemProcessed]-1) // if all k's are already covered, notify and return
		{
			if(kCompleted_local >=KMax_subsystem_approx_[subsystemProcessed])
			{
				if(BASE::options_.debugPrintMP_){
					printString("[Thread " + std::to_string(threadId) + "], subsystem "
							+ std::to_string(subsystemProcessed) + ", k " + std::to_string(k)
					+ ", kCompleted_local " + std::to_string(kCompleted_local)
					+ ", KMax_subsystem_approx_ " + std::to_string(KMax_subsystem_approx_[subsystemProcessed])
					+ ": leaving approximateSubsystemLQWorker AND NOTIFYING ");
				}
				std::unique_lock<std::mutex> lock (kCompletedMutex_);
				kCompletedCondition_.notify_all();
				lock.unlock();
			}
			else{
				if(BASE::options_.debugPrintMP_){
					printString("[Thread " + std::to_string(threadId) + "], subsystem "
							+ std::to_string(subsystemProcessed) + ", k " + std::to_string(k) + ", kCompleted_local " + std::to_string(kCompleted_local)
					+ ", KMax_subsystem_approx_ " + std::to_string(KMax_subsystem_approx_[subsystemProcessed])
					+ ": leaving approximateSubsystemLQWorker but NOT notifying ");
				}
			}

			return subsystemProcessed;
		}
	}

	return subsystemProcessed;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
size_t SLQP_MP<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::calculateControllerWorker(size_t threadId, size_t subsystemProcessed)
{

	size_t k = 0;
	size_t kCompleted_local = 0;

	while(true)
	{
		k = kTaken_ctrl_[subsystemProcessed]++;

		if(k < KMax_subsystem_ctrl_[subsystemProcessed]){

			if(BASE::options_.debugPrintMP_){
				if (k%10 == 0) {
					printString("[Thread " + std::to_string(threadId) + "]: Start calculating controller on index k = " + std::to_string(k) +
							" out of " + std::to_string(KMax_subsystem_ctrl_[subsystemProcessed]-1));
				}
			}

			executeCalculateController(threadId, k, subsystemProcessed);
			kCompleted_local = ++kCompleted_ctrl_[subsystemProcessed];
		}


		if (k >= KMax_subsystem_ctrl_[subsystemProcessed]-1)	// if all k's are already covered, notify and return
		{
			if(kCompleted_local>=KMax_subsystem_ctrl_[subsystemProcessed])
			{
				if(BASE::options_.debugPrintMP_)
					printString("[Thread " + std::to_string(threadId) + "], subsystem " + std::to_string(subsystemProcessed) + ": leaving calculateControllerWorker() AND NOTIFYING ");

				std::unique_lock<std::mutex> lock (kCompletedMutex_);
				kCompletedCondition_.notify_all();
				lock.unlock();
			} else {
				if(BASE::options_.debugPrintMP_)
					printString("[Thread " + std::to_string(threadId) + "], subsystem " + std::to_string(subsystemProcessed) + ": leaving calculateControllerWorker() but NOT notifying ");
			}

			return subsystemProcessed;
		}
	}

	return subsystemProcessed;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
// FIXME: delete this, it is substitude with BASE::executeIntermediateApproximateLQ
//template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
//size_t SLQP_MP<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::executeApproximateSubsystemLQ(size_t threadId, size_t k, size_t subsystemProcessed)
//{
//	const size_t i = subsystemProcessed;
//
//	// LINEARIZE SYSTEM DYNAMICS AND CONSTRAINTS
//	linearizedSystems_[threadId][i]->setCurrentStateAndControl(
//			BASE::nominalTimeTrajectoriesStock_[i][k],
//			BASE::nominalStateTrajectoriesStock_[i][k],
//			BASE::nominalInputTrajectoriesStock_[i][k]);
//	linearizedSystems_[threadId][i]->getDerivativeState(BASE::AmTrajectoryStock_[i][k]);
//	linearizedSystems_[threadId][i]->getDerivativesControl(BASE::BmTrajectoryStock_[i][k]);
//
//	// if constraint type 1 is active
//	if (BASE::nc1TrajectoriesStock_[i][k] > 0) {
//		linearizedSystems_[threadId][i]->getConstraint1DerivativesState(BASE::CmTrajectoryStock_[i][k]);
//		linearizedSystems_[threadId][i]->getConstraint1DerivativesControl(BASE::DmTrajectoryStock_[i][k]);
//	}
//
//	// if constraint type 2 is active
//	if (BASE::nc2TrajectoriesStock_[i][k] > 0) {
//		linearizedSystems_[threadId][i]->getConstraint2DerivativesState(BASE::FmTrajectoryStock_[i][k]);
//	}
//
//	// QUADRATIC APPROXIMATION TO THE COST FUNCTION
//	costFunctions_[threadId][i]->setCurrentStateAndControl(
//			BASE::nominalTimeTrajectoriesStock_[i][k],
//			BASE::nominalStateTrajectoriesStock_[i][k],
//			BASE::nominalInputTrajectoriesStock_[i][k]);
//	costFunctions_[threadId][i]->evaluate(BASE::qTrajectoryStock_[i][k](0));
//	costFunctions_[threadId][i]->stateDerivative(BASE::QvTrajectoryStock_[i][k]);
//	costFunctions_[threadId][i]->stateSecondDerivative(BASE::QmTrajectoryStock_[i][k]);
//	costFunctions_[threadId][i]->controlDerivative(BASE::RvTrajectoryStock_[i][k]);
//	costFunctions_[threadId][i]->controlSecondDerivative(BASE::RmTrajectoryStock_[i][k]);
//	BASE::RmInverseTrajectoryStock_[i][k] = BASE::RmTrajectoryStock_[i][k].inverse();
//	costFunctions_[threadId][i]->stateControlDerivative(BASE::PmTrajectoryStock_[i][k]);
//
//
//	// constraint type 2 coefficients
//	double stateConstraintPenalty = BASE::options_.stateConstraintPenaltyCoeff_ * pow(BASE::options_.stateConstraintPenaltyBase_, BASE::iteration_);
//	size_t nc2 = BASE::nc2TrajectoriesStock_[i][k];
//
//	if (nc2 > 0) {
//		BASE::qTrajectoryStock_[i][k]  += 0.5 * stateConstraintPenalty * BASE::HvTrajectoryStock_[i][k].head(nc2).transpose() * BASE::HvTrajectoryStock_[i][k].head(nc2);
//		BASE::QvTrajectoryStock_[i][k] += stateConstraintPenalty * BASE::FmTrajectoryStock_[i][k].topRows(nc2).transpose() * BASE::HvTrajectoryStock_[i][k].head(nc2);
//		BASE::QmTrajectoryStock_[i][k] += stateConstraintPenalty * BASE::FmTrajectoryStock_[i][k].topRows(nc2).transpose() * BASE::FmTrajectoryStock_[i][k].topRows(nc2);
//	}
//
//	// constraint type 1 coefficients
//	size_t nc1 = BASE::nc1TrajectoriesStock_[i][k];
//
//	if (nc1 == 0)
//	{
//		BASE::DmDagerTrajectoryStock_[i][k].setZero();
//		BASE::EvProjectedTrajectoryStock_[i][k].setZero();
//		BASE::CmProjectedTrajectoryStock_[i][k].setZero();
//		BASE::DmProjectedTrajectoryStock_[i][k].setZero();
//
//		BASE::AmConstrainedTrajectoryStock_[i][k] = BASE::AmTrajectoryStock_[i][k];
//		BASE::QmConstrainedTrajectoryStock_[i][k] = BASE::QmTrajectoryStock_[i][k];
//		BASE::QvConstrainedTrajectoryStock_[i][k] = BASE::QvTrajectoryStock_[i][k];
//		if (BASE::options_.useRiccatiSolver_==true) {
//			BASE::RmConstrainedTrajectoryStock_[i][k] = BASE::RmTrajectoryStock_[i][k];
//		} else {
//			BASE::BmConstrainedTrajectoryStock_[i][k] = BASE::BmTrajectoryStock_[i][k];
//			BASE::PmConstrainedTrajectoryStock_[i][k] = BASE::PmTrajectoryStock_[i][k];
//			BASE::RvConstrainedTrajectoryStock_[i][k] = BASE::RvTrajectoryStock_[i][k];
//		}
//
//	} else {
//		Eigen::MatrixXd Cm = BASE::CmTrajectoryStock_[i][k].topRows(nc1);
//		Eigen::MatrixXd Dm = BASE::DmTrajectoryStock_[i][k].topRows(nc1);
//		Eigen::MatrixXd Ev = BASE::EvTrajectoryStock_[i][k].head(nc1);
//
//		Eigen::MatrixXd RmProjected = ( Dm*BASE::RmInverseTrajectoryStock_[i][k]*Dm.transpose() ).inverse();
//		Eigen::MatrixXd DmDager = BASE::RmInverseTrajectoryStock_[i][k] * Dm.transpose() * RmProjected;
//
//		BASE::DmDagerTrajectoryStock_[i][k].leftCols(nc1) = DmDager;
//		BASE::EvProjectedTrajectoryStock_[i][k] = DmDager * Ev;
//		BASE::CmProjectedTrajectoryStock_[i][k] = DmDager * Cm;
//		BASE::DmProjectedTrajectoryStock_[i][k] = DmDager * Dm;
//
//		control_matrix_t DmNullSpaceProjection = control_matrix_t::Identity() - BASE::DmProjectedTrajectoryStock_[i][k];
//		state_matrix_t   PmTransDmDagerCm = BASE::PmTrajectoryStock_[i][k].transpose()*BASE::CmProjectedTrajectoryStock_[i][k];
//
//		BASE::AmConstrainedTrajectoryStock_[i][k] = BASE::AmTrajectoryStock_[i][k] - BASE::BmTrajectoryStock_[i][k]*BASE::CmProjectedTrajectoryStock_[i][k];
//		BASE::QmConstrainedTrajectoryStock_[i][k] = BASE::QmTrajectoryStock_[i][k] + Cm.transpose()*RmProjected*Cm - PmTransDmDagerCm - PmTransDmDagerCm.transpose();
//		BASE::QvConstrainedTrajectoryStock_[i][k] = BASE::QvTrajectoryStock_[i][k] - BASE::CmProjectedTrajectoryStock_[i][k].transpose()*BASE::RvTrajectoryStock_[i][k];
//		if (BASE::options_.useRiccatiSolver_==true) {
//			BASE::RmConstrainedTrajectoryStock_[i][k] = DmNullSpaceProjection.transpose() * BASE::RmTrajectoryStock_[i][k] * DmNullSpaceProjection;
//		} else {
//			BASE::BmConstrainedTrajectoryStock_[i][k] = BASE::BmTrajectoryStock_[i][k] * DmNullSpaceProjection;
//			BASE::PmConstrainedTrajectoryStock_[i][k] = DmNullSpaceProjection.transpose() * BASE::PmTrajectoryStock_[i][k];
//			BASE::RvConstrainedTrajectoryStock_[i][k] = DmNullSpaceProjection.transpose() * BASE::RvTrajectoryStock_[i][k];
//		}
//
//	}
//
//	// making sure that constrained Qm is PSD
//	this->makePSD(BASE::QmConstrainedTrajectoryStock_[i][k]);
//
//	return i;
//}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
size_t SLQP_MP<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::executeCalculateController(size_t threadId, size_t k, size_t subsystemProcessed)
{

	const size_t i = subsystemProcessed;

	const double time = BASE::SsTimeTrajectoryStock_[i][k];
	size_t greatestLessTimeStampIndex;

	// local variables
	state_vector_t nominalState;
	control_vector_t nominalInput;
	control_gain_matrix_t Bm;
	control_feedback_t Pm;
	control_vector_t Rv;
	control_matrix_t RmInverse;
	control_vector_t EvProjected;
	control_feedback_t CmProjected;
	control_matrix_t DmProjected;
	control_constraint1_matrix_t DmDager;
	control_matrix_t Rm;

	nominalStateFunc_[threadId].interpolate(time, nominalState);
	greatestLessTimeStampIndex = nominalStateFunc_[threadId].getGreatestLessTimeStampIndex();

	nominalInputFunc_[threadId].interpolate(time, nominalInput, greatestLessTimeStampIndex);

	// interpolate
	BmFunc_[threadId].interpolate(time, Bm, greatestLessTimeStampIndex);
	PmFunc_[threadId].interpolate(time, Pm, greatestLessTimeStampIndex);
	RvFunc_[threadId].interpolate(time, Rv, greatestLessTimeStampIndex);
	RmInverseFunc_[threadId].interpolate(time, RmInverse, greatestLessTimeStampIndex);
	EvProjectedFunc_[threadId].interpolate(time, EvProjected, greatestLessTimeStampIndex);
	CmProjectedFunc_[threadId].interpolate(time, CmProjected, greatestLessTimeStampIndex);
	DmProjectedFunc_[threadId].interpolate(time, DmProjected, greatestLessTimeStampIndex);

	control_feedback_t Lm  = RmInverse * (Pm + Bm.transpose()*BASE::SmTrajectoryStock_[i][k]);
	control_vector_t   Lv  = RmInverse * (Rv + Bm.transpose()*BASE::SvTrajectoryStock_[i][k]);
	control_vector_t   Lve = RmInverse * (Bm.transpose()*BASE::SveTrajectoryStock_[i][k]);

	control_matrix_t DmNullProjection = control_matrix_t::Identity()-DmProjected;
	BASE::nominalControllersStock_[i].k_[k]   = -DmNullProjection*Lm - CmProjected;
	BASE::nominalControllersStock_[i].uff_[k] = nominalInput - BASE::nominalControllersStock_[i].k_[k]*nominalState
			- BASE::options_.constraintStepSize_* (DmNullProjection*Lve + EvProjected);
	BASE::nominalControllersStock_[i].deltaUff_[k] = -DmNullProjection*Lv;

	// checking the numerical stability of the controller parameters
	if (BASE::options_.checkNumericalStability_==true){
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

	return i;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQP_MP<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::lineSearchWorker(size_t threadId)
{
	if(BASE::options_.debugPrintMP_)
		printString("[Thread " + std::to_string(threadId) + "]: Starting lineSearchWorker. ");

	// local search forward simulation's variables
	scalar_t lsTotalCost;
	controller_array_t          lsControllersStock(BASE::options_.numPartitionings_);
	std::vector<scalar_array_t>	lsTimeTrajectoriesStock(BASE::options_.numPartitionings_);
	std::vector<size_array_t>	lsEventsPastTheEndIndecesStock(BASE::options_.numPartitionings_);
	state_vector_array2_t   	lsStateTrajectoriesStock(BASE::options_.numPartitionings_);
	control_vector_array2_t 	lsInputTrajectoriesStock(BASE::options_.numPartitionings_);
	std::vector<size_array_t>	lsNc1TrajectoriesStock(BASE::options_.numPartitionings_);
	constraint1_vector_array2_t lsEvTrajectoryStock(BASE::options_.numPartitionings_);
	std::vector<size_array_t> 	lsNc2TrajectoriesStock(BASE::options_.numPartitionings_);
	constraint2_vector_array2_t lsHvTrajectoryStock(BASE::options_.numPartitionings_);
	std::vector<size_array_t>	lsNc2FinalStock(BASE::options_.numPartitionings_);
	constraint2_vector_array2_t	lsHvFinalStock(BASE::options_.numPartitionings_);

	while(true)
	{
		size_t alphaExp = alphaTaken_++;

		scalar_t learningRate  = BASE::options_.maxLearningRateGSLQP_ * std::pow(BASE::options_.lineSearchContractionRate_, alphaExp);


		if (learningRate < BASE::options_.minLearningRateGSLQP_ || alphaBestFound_.load() == true)
		{
			if(BASE::options_.debugPrintMP_)
			{
				if (alphaBestFound_.load() == true)
					printString("[Thread " + std::to_string(threadId) + "]: Leaving lineSearchWorker because best alpha is found OR no improvement for any alpha");
				else
					printString("[Thread "+ std::to_string(threadId) +"]: Leaving lineSearchWorker because learningRate < BASE::options_.minLearningRateGSLQP_");
			}

			break;
		}

		if(BASE::options_.debugPrintMP_)
			printString("[Thread " + std::to_string(threadId) + "]: Trying learningRate " + std::to_string(learningRate));

		lsControllersStock = initLScontrollersStock_;

		executeLineSearch(threadId,
				learningRate,
				lsTotalCost,
				lsControllersStock,
				lsTimeTrajectoriesStock,
				lsEventsPastTheEndIndecesStock,
				lsStateTrajectoriesStock,
				lsInputTrajectoriesStock,
				lsNc1TrajectoriesStock,
				lsEvTrajectoryStock,
				lsNc2TrajectoriesStock,
				lsHvTrajectoryStock,
				lsNc2FinalStock,
				lsHvFinalStock);


		// make sure we do not alter an existing result
		if (alphaBestFound_.load() == true)
		{
			if(BASE::options_.debugPrintMP_)
				printString("[Thread " + std::to_string(threadId) + "]: Leaving lineSearchWorker because best alpha already found by another thread.");

			break;
		}


		lineSearchResultMutex_.lock();

		bool updatePolicy = false;
		if(BASE::options_.lsStepsizeGreedy_ == true)  // equivalent to single core lineSearch
		{
			// act stepsize greedy, cost should be better than in last iteration but learning rate should be as high as possible
			if(lsTotalCost < (BASE::nominalTotalCost_*(1-1e-3*learningRate)) && learningRate > BASE::learningRateStar_)
			{
				updatePolicy = true;
				if(BASE::options_.debugPrintMP_){
					printString("[LS, Thread " + std::to_string(threadId) + "]: stepsize-greedy mode : better stepsize and cost found: " + std::to_string(lsTotalCost)
					+ " at learningRate: " + std::to_string(learningRate));
				}
			}
			else{
				if(BASE::options_.debugPrintMP_){
					printString("[LS, Thread " + std::to_string(threadId) + "]: stepsize-greedy mode : no better combination found, cost " + std::to_string(lsTotalCost)
					+ " at learningRate: " + std::to_string(learningRate));
				}
			}
		}
		else // line search acts cost greedy, minimize cost as much as possible
		{
			if(lsTotalCost < (lowestTotalCost_*(1-1e-3*learningRate)))
			{
				updatePolicy = true;
				if(BASE::options_.debugPrintMP_){
					printString("[LS, Thread " + std::to_string(threadId) + "]: cost-greedy mode : better cost found: " + std::to_string(lsTotalCost)
					+ " at learningRate: " + std::to_string(learningRate));
				}
			}
			else{
				if(BASE::options_.debugPrintMP_){
					printString("[LS, Thread " + std::to_string(threadId) + "]: cost-greedy mode : no better cost found, cost " + std::to_string(lsTotalCost)
					+ " at learningRate: " + std::to_string(learningRate) + ". Best cost was " + std::to_string(lowestTotalCost_));
				}
			}
		}


		if (updatePolicy == true)
		{
			alphaExpBest_ 	  	= alphaExp;
			lowestTotalCost_ 	= lsTotalCost;
			BASE::learningRateStar_ = learningRate;

			for (size_t i = 0; i<BASE::options_.numPartitionings_; i++)	// swapping where possible for improved efficiency
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
				BASE::nc2FinalStock_[i] = lsNc2FinalStock[i];
				BASE::HvFinalStock_[i] = lsHvFinalStock[i];
			}
		}

		alphaProcessed_[alphaExp] = 1;

		// we now check if all alphas prior to the best have been processed, this also covers the case that there is no better alpha
		bool allPreviousAlphasProcessed = true;
		for (size_t i=0; i<alphaExpBest_; i++)
		{
			if (alphaProcessed_[i] != 1)
			{
				allPreviousAlphasProcessed = false;
				break;
			}
		}
		if (allPreviousAlphasProcessed)
		{
			alphaBestFound_ = true;
			event_handler_t::ActivateKillIntegration();	// kill all integrators
			if (BASE::options_.dispayGSLQP_) {
				printString("\t LS: terminate other rollouts with different alphas. alpha_best found or terminating without improvement. ");
			}
		}

		lineSearchResultMutex_.unlock();

	}

	lsWorkerCompleted_++;

	if(BASE::options_.debugPrintMP_)
		printString("[Thread " + std::to_string(threadId) + "]: Leaving lineSearchWorker ");

	if (lsWorkerCompleted_.load() >= BASE::options_.nThreads_)
	{
		std::unique_lock<std::mutex> lock (alphaBestFoundMutex_);
		alphaBestFoundCondition_.notify_all();
		lock.unlock();

		if(BASE::options_.debugPrintMP_)
			printString("NOTIFYING by LS WORKER since all workers are now done ");
	}
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQP_MP<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::executeLineSearch(
		size_t threadId,
		double learningRate,
		scalar_t& lsTotalCost,
		controller_array_t& lsControllersStock,
		std::vector<scalar_array_t>& lsTimeTrajectoriesStock,
		std::vector<size_array_t>& lsEventsPastTheEndIndecesStock,
		state_vector_array2_t& lsStateTrajectoriesStock,
		control_vector_array2_t& lsInputTrajectoriesStock,
		std::vector<size_array_t>& lsNc1TrajectoriesStock,
		constraint1_vector_array2_t& lsEvTrajectoryStock,
		std::vector<size_array_t>& lsNc2TrajectoriesStock,
		constraint2_vector_array2_t& lsHvTrajectoryStock,
		std::vector<size_array_t>& lsNc2FinalStock,
		constraint2_vector_array2_t& lsHvFinalStock)  {

	// modifying uff by local increments
	for (size_t i=0; i<BASE::options_.numPartitionings_; i++)
		for (size_t k=0; k<lsControllersStock[i].time_.size(); k++)
			lsControllersStock[i].uff_[k] += learningRate * lsControllersStock[i].deltaUff_[k];

	try {
		rolloutTrajectory(threadId,
				BASE::initTime_, BASE::initState_, BASE::finalTime_, lsControllersStock,
				lsTimeTrajectoriesStock, lsEventsPastTheEndIndecesStock,
				lsStateTrajectoriesStock, lsInputTrajectoriesStock);
		calculateRolloutConstraints(threadId,
				lsTimeTrajectoriesStock, lsEventsPastTheEndIndecesStock,
				lsStateTrajectoriesStock, lsInputTrajectoriesStock,
				lsNc1TrajectoriesStock, lsEvTrajectoryStock,
				lsNc2TrajectoriesStock, lsHvTrajectoryStock, lsNc2FinalStock, lsHvFinalStock);
		// calculate rollout cost
		calculateRolloutCost(threadId,
				lsTimeTrajectoriesStock, lsEventsPastTheEndIndecesStock,
				lsStateTrajectoriesStock, lsInputTrajectoriesStock,
				lsNc2TrajectoriesStock, lsHvTrajectoryStock, lsNc2FinalStock, lsHvFinalStock,
				lsTotalCost);

		// display
		if (BASE::options_.dispayGSLQP_){
			scalar_t lsConstraint1ISE;
			this->calculateConstraintISE(lsTimeTrajectoriesStock, lsNc1TrajectoriesStock, lsEvTrajectoryStock, lsConstraint1ISE);

			printString("\t [Thread" + std::to_string(threadId) + "] - learningRate " + std::to_string(learningRate) + " \t cost: " + std::to_string(lsTotalCost) +
					" \t constraint ISE: " + std::to_string(lsConstraint1ISE));
			if (std::accumulate(lsNc2FinalStock.begin(), lsNc2FinalStock.end(), 0) > 0) {
				std::cerr << "\t final constraint type-2:   ";
				for(size_t i=0; i<BASE::options_.numPartitionings_; i++) std::cerr << "[" << i  << "]: " << lsHvFinalStock[i].head(lsNc2FinalStock[i]).transpose() << ",  ";
				std::cerr << std::endl;
			}

		}
	}
	catch(const std::exception& error)
	{
		if(BASE::options_.debugPrintMP_)  std::cout << error.what() << std::endl;
		lsTotalCost  = std::numeric_limits<scalar_t>::max();
	}
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQP_MP<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::solveSequentialRiccatiEquations(const scalar_t& learningRate,
		const state_matrix_t& SmFinal, const state_vector_t& SvFinal, const eigen_scalar_t& sFinal){

	riccatiSolverLearningRate_ = learningRate;

	numSubsystemsProcessed_ = 0;
//	if (async_==true)
//		for (size_t i=0; i < BASE::options_.numPartitionings_; i++){
//			subsystemsDone_[i] = false;
//			subsystemsProcessing_[i] = false;
//		}

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

	// solve it sequentially for the first time or when useDisjointRiccati_ is false
	if(BASE::useDisjointRiccati_==false || BASE::iteration_ == 0) {

		for (int i=BASE::options_.numPartitionings_-1; i>=0; i--)  {

			if (i < (signed)BASE::initActiveSubsystem_ || i > (signed)BASE::finalActiveSubsystem_) {

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

			// solve backward pass
			if (BASE::options_.useRiccatiSolver_==true) {
				BASE::solveRiccatiEquationsWorker(workerIndex, i,
						riccatiSolverLearningRate_,
						BASE::SmFinalStock_[i+1], BASE::SvFinalStock_[i+1], BASE::sFinalStock_[i+1]);
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
	}
	// solve it inparallel if useDisjointRiccati_ is true
	else {

		if(BASE::options_.debugPrintMP_)
			printString("[MP]: Waking up workers to do RiccatiSolver Task ");

		workerTask_ = SOLVE_RICCATI;
		std::unique_lock<std::mutex> lock (workerWakeUpMutex_);
		workerWakeUpCondition_.notify_all();
		lock.unlock();

		if(BASE::options_.debugPrintMP_)
			printString("[MP]: Will wait now until workers have done RiccatiSolver Task ");

		std::unique_lock<std::mutex> waitLock(riccatiSolverBarrierMutex_);
		while(numSubsystemsProcessed_.load() < BASE::options_.numPartitionings_){
			riccatiSolverCompletedCondition_.wait(waitLock);
		}
		waitLock.unlock();

		workerTask_ = IDLE;
	}

	// state at the switching times
	BASE::xFinalStock_[BASE::finalActiveSubsystem_+1] = BASE::nominalStateTrajectoriesStock_[BASE::finalActiveSubsystem_].back();
	for (size_t i=BASE::initActiveSubsystem_; i<=BASE::finalActiveSubsystem_; i++)
		BASE::xFinalStock_[i] = BASE::nominalStateTrajectoriesStock_[i].front();

	if(BASE::options_.debugPrintMP_){
		printString("Iteration: " + std::to_string(BASE::iteration_) + " done");
		printString("----------------------------------");
		printString("----------------------------------");
	}

}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQP_MP<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::executeRiccatiSolver(size_t threadId) {

	bool endSubsystemIsUpdated = false;

	for (int i = endingIndicesRiccatiWorker_[threadId]; i >= startingIndicesRiccatiWorker_[threadId]; i--) {

		if(BASE::options_.debugPrintMP_)
			printString("[MP]>> Thread " + std::to_string(threadId) + " processing subsystem " + std::to_string(i));


		// for inactive subsystems
		if (i < (signed)BASE::initActiveSubsystem_ || i > (signed)BASE::finalActiveSubsystem_) {

			BASE::SsTimeTrajectoryStock_[i].clear();
			BASE::SmTrajectoryStock_[i].clear();
			BASE::SvTrajectoryStock_[i].clear();
			BASE::SveTrajectoryStock_[i].clear();
			BASE::sTrajectoryStock_[i].clear();

			// lock data
			std::unique_lock<std::mutex> dataWriteLock(riccatiSolverDataMutex_);

			BASE::SmFinalStock_[i].setZero();
			BASE::SvFinalStock_[i].setZero();
			BASE::SveFinalStock_[i].setZero();
			BASE::sFinalStock_[i].setZero();
			BASE::xFinalStock_[i].setZero();

			// unlock data
			dataWriteLock.unlock();

			numSubsystemsProcessed_++;

			continue;
		}

		// lock data
		std::unique_lock<std::mutex> dataReadLock(riccatiSolverDataMutex_);

		state_matrix_t SmFinal  = BASE::SmFinalStock_[i+1];
		state_vector_t SvFinal  = BASE::SvFinalStock_[i+1];
		state_vector_t SveFinal = BASE::SveFinalStock_[i+1];
		eigen_scalar_t sFinal   = BASE::sFinalStock_[i+1];
		state_vector_t xFinal   = BASE::xFinalStock_[i+1];

		// unlock data
		dataReadLock.unlock();

		// solve the backward pass
		if (endSubsystemIsUpdated==false) {
			endSubsystemIsUpdated = true;
			SvFinal += SmFinal*(BASE::nominalStateTrajectoriesStock_[i].back()-xFinal);
		}

		if (BASE::options_.useRiccatiSolver_==true) {
			BASE::solveRiccatiEquationsWorker(threadId, i,
					riccatiSolverLearningRate_, SmFinal, SvFinal, sFinal);
			BASE::solveErrorRiccatiEquationWorker(threadId, i,
					SveFinal);
		} else {
			BASE::fullRiccatiBackwardSweepWorker(threadId, i,
					SmFinal, SvFinal, SveFinal, sFinal);
		}

		// lock data
		std::unique_lock<std::mutex> dataWriteLock(riccatiSolverDataMutex_);

		// set the final value for next Riccati equation
		BASE::sFinalStock_[i]   = BASE::sTrajectoryStock_[i].front();
		BASE::SvFinalStock_[i]  = BASE::SvTrajectoryStock_[i].front();
		BASE::SveFinalStock_[i] = BASE::SveTrajectoryStock_[i].front();
		BASE::SmFinalStock_[i]  = BASE::SmTrajectoryStock_[i].front();

		// unlock data
		dataWriteLock.unlock();

		numSubsystemsProcessed_++;
	}

	// notify the main thread so that it stops waiting
//	std::unique_lock<std::mutex> lock(riccatiSolverBarrierNotifyMutex_);
	std::unique_lock<std::mutex> lock(riccatiSolverBarrierMutex_);
	riccatiSolverCompletedCondition_.notify_one();
	lock.unlock();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQP_MP<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::distributeWork(){

	const int N = BASE::options_.nThreads_;
	startingIndicesRiccatiWorker_.resize(N);
	endingIndicesRiccatiWorker_.resize(N);

//	if (BASE::options_.numPartitionings_ < N)
//		throw std::runtime_error("Number of threads is bigger than number of subsystems");

	int subsystemsPerThread = (BASE::finalActiveSubsystem_-BASE::initActiveSubsystem_+1) / N;
	int remainingSubsystems = (BASE::finalActiveSubsystem_-BASE::initActiveSubsystem_+1) % N;

	int startingId, endingId = BASE::finalActiveSubsystem_;
	for (size_t i=0; i<N; i++){
		endingIndicesRiccatiWorker_[i] = endingId;
		if (remainingSubsystems > 0){
			startingId = endingId - subsystemsPerThread;
			remainingSubsystems--;
		} else {
			startingId = endingId - subsystemsPerThread + 1;
		}
		startingIndicesRiccatiWorker_[i] = startingId;
		endingId = startingId - 1;
	}

	// adding the inactive subsystems
	endingIndicesRiccatiWorker_.front() = BASE::options_.numPartitionings_-1;
	startingIndicesRiccatiWorker_.back() = 0;

	if (BASE::options_.dispayGSLQP_==true) {
		std::cout << "Initial Active Subsystem: " << BASE::initActiveSubsystem_ << std::endl;
		std::cout << "Final Active Subsystem:   " << BASE::finalActiveSubsystem_ << std::endl;
		std::cout << "Backward path work distribution:" << std::endl;
		for (size_t i=0; i<N; i++){
			std::cout << "start: " << startingIndicesRiccatiWorker_[i] << "\t";
			std::cout << "end: " << endingIndicesRiccatiWorker_[i]  << "\t";
			std::cout << "num: " << endingIndicesRiccatiWorker_[i]-startingIndicesRiccatiWorker_[i]+1 << std::endl;;
		}
		std::cout << std::endl;
	}
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQP_MP<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::setupOptimizer() {

	// call Base routine
	BASE::setupOptimizer();

//	// for all threads + 1
//	for (size_t i=0; i<BASE::options_.nThreads_+1; i++)  {
//
//		dynamics_[i].clear();
//		dynamics_[i].reserve(BASE::options_.numPartitionings_);
//		linearizedSystems_[i].clear();
//		linearizedSystems_[i].reserve(BASE::options_.numPartitionings_);
//		constraints_[i].clear();
//		constraints_[i].reserve(BASE::options_.numPartitionings_);
//		costFunctions_[i].clear();
//		costFunctions_[i].reserve(BASE::options_.numPartitionings_);
//		eventHandlerPtrs_[i].clear();
//		eventHandlerPtrs_[i].reserve(BASE::options_.numPartitionings_);
//		integratorsODE45_[i].clear();
//		integratorsODE45_[i].reserve(BASE::options_.numPartitionings_);
//
//		// .. initialize all subsystems, etc.
//		for(size_t j = 0; j<BASE::options_.numPartitionings_; j++)  {
//
//			// initialize dynamics
//			dynamics_[i].push_back(std::move( BASE::subsystemDynamicsPtr_->clone() ));
//
//			// initialize linearized systems
//			linearizedSystems_[i].push_back(std::move( BASE::subsystemDerivativesPtr_->clone() ));
//
//			// initialize constraints
//			constraints_[i].push_back(std::move( BASE::subsystemConstraintPtr_->clone() ));
//
//			// initialize cost functions
//			costFunctions_[i].push_back(std::move( BASE::subsystemCostFunctionsPtr_->clone() ));
//
//			// initialize operating trajectories
//			operatingTrajectories_[i].push_back(std::move( BASE::operatingTrajectoriesPtr_->clone() ));
//
//			// initialize events
//			typedef Eigen::aligned_allocator<event_handler_t> event_handler_alloc_t;
//			eventHandlerPtrs_[i].push_back(std::move( std::allocate_shared<event_handler_t, event_handler_alloc_t>(event_handler_alloc_t()) ));
//
//			// initialize integrators
//			typedef ODE45<STATE_DIM> ode_t;
//			typedef Eigen::aligned_allocator<ode_t> ode_alloc_t;
//			integratorsODE45_[i].push_back(std::move(
//					std::allocate_shared<ode_t, ode_alloc_t>(ode_alloc_t(), dynamics_[i].back(), eventHandlerPtrs_[i].back()) ));
//		}
//	}

	KMax_subsystem_approx_.resize(BASE::options_.numPartitionings_);
	KMax_subsystem_ctrl_.resize(BASE::options_.numPartitionings_);

}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQP_MP<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::runInit() {

	// set the start and final time for costFuntions
	for(size_t i=0; i<BASE::options_.numPartitionings_; i++)
		BASE::subsystemCostFunctionsPtr_[i]->setTimePeriod(BASE::partitioningTimes_[i], BASE::partitioningTimes_[i+1]);

	//distribute work
	distributeWork();

	// initial controller rollout
	rolloutTrajectory(BASE::initTime_,
			BASE::initState_,
			BASE::finalTime_,
			BASE::nominalControllersStock_,
			BASE::nominalTimeTrajectoriesStock_, BASE::nominalEventsPastTheEndIndecesStock_,
			BASE::nominalStateTrajectoriesStock_, BASE::nominalInputTrajectoriesStock_);

	calculateRolloutConstraints(BASE::nominalTimeTrajectoriesStock_, BASE::nominalEventsPastTheEndIndecesStock_,
			BASE::nominalStateTrajectoriesStock_, BASE::nominalInputTrajectoriesStock_,
			BASE::nc1TrajectoriesStock_, BASE::EvTrajectoryStock_,
			BASE::nc2TrajectoriesStock_, BASE::HvTrajectoryStock_, BASE::nc2FinalStock_, BASE::HvFinalStock_);

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
void SLQP_MP<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::runIteration(const scalar_t& maxLearningRateStar,
		const state_matrix_t& SmFinal /*= state_matrix_t::Zero()*/,
		const state_vector_t& SvFinal /*= state_vector_t::Zero()*/,
		const eigen_scalar_t& sFinal /*= eigen_scalar_t::Zero()*/) {

	// disable Eigen multi-threading
	Eigen::setNbThreads(1);
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
	lineSearch();

#ifdef BENCHMARK
	end = std::chrono::steady_clock::now();
	diff = end - start;
	tAvg3 = ((1.0 - 1.0/nIterations)* tAvg3) + (1.0/nIterations)*std::chrono::duration_cast<std::chrono::milliseconds>(diff).count();
#endif

	// restore default Eigen thread number
	Eigen::setNbThreads(0);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQP_MP<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::runExit(const state_matrix_t& SmFinal /*= state_matrix_t::Zero()*/,
		const state_vector_t& SvFinal /*= state_vector_t::Zero()*/,
		const eigen_scalar_t& sFinal /*= eigen_scalar_t::Zero()*/) {

	// disable Eigen multi-threading
	Eigen::setNbThreads(1);

	// linearizing the dynamics and quadratizing the cost function along nominal trajectories
	BASE::approximateOptimalControlProblem();

	// solve Riccati equations
	solveSequentialRiccatiEquations(0.0 /*learningRate*/, SmFinal, SvFinal, sFinal);

	// restore default Eigen thread number
	Eigen::setNbThreads(0);

	// calculate the nominal co-state
	BASE::calculateRolloutCostate(BASE::nominalTimeTrajectoriesStock_, BASE::nominalStateTrajectoriesStock_, BASE::nominalcostateTrajectoriesStock_);
}


} // namespace ocs2
