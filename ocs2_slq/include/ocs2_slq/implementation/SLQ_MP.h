/*
 * Implementation of SLQ_MP.h
 *
 *  Created on: Jun 20, 2016
 *      Author: farbod
 */


namespace ocs2{

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
SLQ_MP<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::~SLQ_MP()  {

	workersActive_ = false;
	workerTask_ = SHUTDOWN;

	workerWakeUpCondition_.notify_all();

	if(BASE::settings_.debugPrintMP_)
		std::cerr << "Shutting down workers." << std::endl;

	for (size_t i=0; i<workerThreads_.size(); i++)
		workerThreads_[i].join();

	if(BASE::settings_.debugPrintMP_)
		std::cerr << "All workers shut down" << std::endl;
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQ_MP<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::lineSearch(bool computeISEs) {

	BASE::lsComputeISEs_ = computeISEs;

	// display
	if (BASE::settings_.displayInfo_) {
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
	} else {
		BASE::nominalConstraint1ISE_ = BASE::nominalConstraint1MaxNorm_ = 0.0;
		BASE::nominalConstraint2ISE_ = BASE::nominalConstraint2MaxNorm_ = 0.0;
	}

	// calculates cost
	BASE::calculateRolloutCost(BASE::nominalTimeTrajectoriesStock_, BASE::nominalEventsPastTheEndIndecesStock_,
			BASE::nominalStateTrajectoriesStock_, BASE::nominalInputTrajectoriesStock_,
			BASE::nominalConstraint2ISE_, BASE::nc2FinalStock_, BASE::HvFinalStock_,
			BASE::nominalTotalCost_);

	baselineTotalCost_ = BASE::nominalTotalCost_;
	BASE::learningRateStar_ = 0.0;	// input correction learning rate is zero

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

	BASE::initLScontrollersStock_ = BASE::nominalControllersStock_;  // this will serve to init the workers

	subsystemProcessed_ = 0; // not required for linesearch, but assign to not let it dangle around
	alphaProcessed_.clear();
	alphaTaken_ = 0;
	alphaBestFound_ = false;
	lsWorkerCompleted_ = 0;

	size_t maxNumOfLineSearches =  (int) (log(BASE::settings_.minLearningRateGSLQP_/BASE::settings_.maxLearningRateGSLQP_) / log(BASE::settings_.lineSearchContractionRate_)) +1;
	alphaExpMax_ = maxNumOfLineSearches;
	alphaExpBest_ = maxNumOfLineSearches;
	alphaProcessed_ = std::vector<bool>(maxNumOfLineSearches, false);

	if(BASE::settings_.debugPrintMP_) {
		std::cerr << "[MP]: calculated maximum number of line searches " + std::to_string(alphaExpMax_) << std::endl;
		std::cerr << "[MP]: Waking up workers for line search " << std::endl;
	}

	workerTask_ = LINE_SEARCH;
	std::unique_lock<std::mutex> lock (workerWakeUpMutex_);
	workerWakeUpCondition_.notify_all();
	lock.unlock();

	if(BASE::settings_.debugPrintMP_)
		BASE::printString("[MP]: Will sleep now until we have results ");


	std::unique_lock<std::mutex> waitLock(alphaBestFoundMutex_);
	while(lsWorkerCompleted_.load() < BASE::settings_.nThreads_)
		alphaBestFoundCondition_.wait(waitLock);
	waitLock.unlock();

	workerTask_ = IDLE;

	// revitalize all integrators
	event_handler_t::DeactivateKillIntegration();

	// clear the feedforward increments
	for (size_t i=0; i<BASE::numPartitionings_; i++)
		BASE::nominalControllersStock_[i].deltaUff_.clear();

	// display
	if (BASE::settings_.displayInfo_)
		std::cerr << "The chosen learningRate is: " + std::to_string(BASE::learningRateStar_) << std::endl;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQ_MP<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::launchWorkerThreads()
{
	workersActive_ = true;
	workerTask_ = IDLE;

	workerThreads_.clear();
	for (size_t i=0; i<BASE::settings_.nThreads_; i++) {
		workerThreads_.push_back(std::thread(&SLQ_MP::threadWork, this, i));
	}
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQ_MP<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::threadWork(size_t threadId)
{
	if(BASE::settings_.debugPrintMP_)
		BASE::printString("[MP]: [Thread " + std::to_string(threadId) + "]: launched");

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
		if(BASE::settings_.debugPrintMP_){
			BASE::printString("[MP]: [Thread " + std::to_string(threadId) + "]: previous procId: " + std::to_string(uniqueProcessID) +
					", current procId: " +std::to_string(generateUniqueProcessID(iteration_local, (int) workerTask_local, (int) subsystemProcessed_local)));
		}

		/* We want to put the worker to sleep if
		 * - the workerTask_ is IDLE
		 * - or we are finished both workerTask_ is not yet reset, thus the process ID is still the same
		 * */
		if ( workerTask_local == IDLE || uniqueProcessID == generateUniqueProcessID(iteration_local, (int) workerTask_local, (int) subsystemProcessed_local))
		{
			if(BASE::settings_.debugPrintMP_)
				BASE::printString("[MP]: [Thread " + std::to_string(threadId) + "]: going to sleep !");

			// sleep until the state is not IDLE any more and we have a different process ID than before
			std::unique_lock<std::mutex> waitLock(workerWakeUpMutex_);
			while(workerTask_ == IDLE ||  (uniqueProcessID == generateUniqueProcessID(BASE::iteration_, (int)workerTask_.load(), (int) subsystemProcessed_.load()))){
				workerWakeUpCondition_.wait(waitLock);
			}
			waitLock.unlock();

			subsystemProcessed_local = subsystemProcessed_.load();
			workerTask_local = workerTask_.load();
			iteration_local = BASE::iteration_;

			if(BASE::settings_.debugPrintMP_)
				BASE::printString("[MP]: [Thread " + std::to_string(threadId) + "]: woke up !");
		}

		if (!workersActive_)
			break;

		switch(workerTask_local)
		{
		case APPROXIMATE_LQ:
		{
			if(BASE::settings_.debugPrintMP_)
				BASE::printString("[MP]: [Thread " + std::to_string(threadId) + "]: is busy with APPROXIMATE_LQ on partition "
						+ std::to_string(subsystemProcessed_local));

			executeApproximatePartitionLQWorker(threadId, subsystemProcessed_local);
			uniqueProcessID = generateUniqueProcessID (iteration_local, APPROXIMATE_LQ, subsystemProcessed_local);

			break;
		}
		case CALCULATE_CONTROLLER:
		{
			if(BASE::settings_.debugPrintMP_)
				BASE::printString("[MP]: [Thread " + std::to_string(threadId) + "]: now busy with CALCULATE_CONTROLLER !");

			executeCalculatePartitionController(threadId, subsystemProcessed_local);
			uniqueProcessID = generateUniqueProcessID (iteration_local, CALCULATE_CONTROLLER, subsystemProcessed_local);

			break;
		}
		case LINE_SEARCH:
		{
			if(BASE::settings_.debugPrintMP_)
				BASE::printString("[MP]: [Thread " + std::to_string(threadId) + "]: now busy with LINE_SEARCH !");

			executeLineSearchWorker(threadId);
			uniqueProcessID = generateUniqueProcessID (iteration_local, LINE_SEARCH, subsystemProcessed_local);
			break;
		}
		case SOLVE_RICCATI:
		{
			if(BASE::settings_.debugPrintMP_)
				BASE::printString("[MP]: [Thread "+ std::to_string(threadId) +"]: now busy with RiccatiSolver!");
			uniqueProcessID = generateUniqueProcessID (iteration_local, SOLVE_RICCATI, subsystemProcessed_local);
			executeRiccatiSolver(threadId);
			break;
		}
		case SHUTDOWN:
		{
			if(BASE::settings_.debugPrintMP_)
				BASE::printString("[MP]: [Thread "+ std::to_string(threadId) +"]: now shutting down!");
			return;
		}
		}

		if(BASE::settings_.debugPrintMP_)
			BASE::printString("[MP]: [Thread " + std::to_string(threadId) +"]: done with job. Will wait for next now!");
	}
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQ_MP<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::approximatePartitionLQ(const size_t& partitionIndex)  {

	subsystemProcessed_ = partitionIndex;

	size_t N = BASE::nominalTimeTrajectoriesStock_[partitionIndex].size();

	if (N > 0) {

		// dispaly
		if(BASE::settings_.debugPrintMP_)
			std::cerr << "[MP]: Activating threads to perform LQ approximation for partition " + std::to_string(partitionIndex) << std::endl;

		kTaken_approx_[partitionIndex] = 0;
		kCompleted_approx_[partitionIndex]= 0;

		// activates all threads' APPROXIMATE_LQ task which in turn runs executeApproximatePartitionLQWorker routine
		workerTask_ = APPROXIMATE_LQ;
		std::unique_lock<std::mutex> lock (workerWakeUpMutex_);
		workerWakeUpCondition_.notify_all();
		lock.unlock();

		// dispaly
		if(BASE::settings_.debugPrintMP_)
			BASE::printString("[MP]: Waiting until threads finish LQ approximation for partition " + std::to_string(partitionIndex));

		// wait until all threads finish their task
		std::unique_lock<std::mutex> waitLock(kCompletedMutex_);
		while(kCompleted_approx_[partitionIndex].load() < N) {
			kCompletedCondition_.wait(waitLock);
		}
		waitLock.unlock();

		// reset threads to no task mode
		workerTask_ = IDLE;

		// display
		if(BASE::settings_.debugPrintMP_)
			std::cerr << "[MP]: Back to main thread, workers should now have finished LQ approximation for partition "
				+ std::to_string(partitionIndex) << std::endl;
	}
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQ_MP<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::executeApproximatePartitionLQWorker(
		size_t threadId,
		const size_t& partitionIndex) {

	size_t kCompleted_local;
	int N = BASE::nominalTimeTrajectoriesStock_[partitionIndex].size();
	int k = -1;  // to make use that the while loop runs at least once

	while(k+1 < N)  {

		// update k
		k = kTaken_approx_[partitionIndex]++;

		if(k < N) {
			// display
			if (BASE::settings_.debugPrintMP_ && k%10 == 0) {
				BASE::printString("[MP]: [Thread " + std::to_string(threadId) + "], partition " + std::to_string(partitionIndex)
				+ ":Start approximating system LQ on index k = " + std::to_string(k) + " out of " + std::to_string(N-1));
			}

			// execute approximateLQ for the given partition and time node index (k)
			BASE::approximateLQWorker(threadId, partitionIndex, k);

			// increment the number of completed nodes
			kCompleted_local = ++kCompleted_approx_[partitionIndex];
		}

	}  // enf of while loop

	// all k's are already covered. If all the nodes are completed notify and return, else just return.
	if(kCompleted_local >= N)  {

		std::unique_lock<std::mutex> lock (kCompletedMutex_);
		kCompletedCondition_.notify_all();
		lock.unlock();

		// display
		if(BASE::settings_.debugPrintMP_) {
			BASE::printString("[MP]: [Thread " + std::to_string(threadId) + "], partition " + std::to_string(partitionIndex)
				+ ", k " + std::to_string(k) + ", kCompleted_local " + std::to_string(kCompleted_local)
				+ ", KMax " + std::to_string(N) + ": leaving executeApproximatePartitionLQWorker AND NOTIFYING ");
		}

	} else {
		// display
		if(BASE::settings_.debugPrintMP_){
			BASE::printString("[MP]: [Thread " + std::to_string(threadId) + "], partition " + std::to_string(partitionIndex)
				+ ", k " + std::to_string(k) + ", kCompleted_local " + std::to_string(kCompleted_local)
				+ ", KMax " + std::to_string(N) + ": leaving executeApproximatePartitionLQWorker but NOT notifying ");
		}
	}
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQ_MP<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::calculatePartitionController(const size_t& partitionIndex) {

	subsystemProcessed_ = partitionIndex;

	size_t N = BASE::SsTimeTrajectoryStock_[partitionIndex].size();

	if (N > 0) {

		// display
		if(BASE::settings_.debugPrintMP_)
			std::cerr << "[MP]: Waking up workers to calculate controller for partition "
				+ std::to_string(partitionIndex) << std::endl;

		kTaken_ctrl_[partitionIndex] = 0;
		kCompleted_ctrl_[partitionIndex] = 0;

		workerTask_ = CALCULATE_CONTROLLER;
		std::unique_lock<std::mutex> lock (workerWakeUpMutex_);
		workerWakeUpCondition_.notify_all();
		lock.unlock();

		// display
		if(BASE::settings_.debugPrintMP_)
			BASE::printString("[MP]: Will wait now controllers have been calculated for partition "
				+ std::to_string(partitionIndex));

		// wait until all threads finish their task
		std::unique_lock<std::mutex> waitLock(kCompletedMutex_);
		while(kCompleted_ctrl_[partitionIndex].load() < N){
			kCompletedCondition_.wait(waitLock);
		}
		waitLock.unlock();

		// reset threads to no task mode
		workerTask_ = IDLE;

		// display
		if(BASE::settings_.debugPrintMP_)
			std::cerr << "[MP]: Back to main thread, workers should now have designed controllers for partition "
				+ std::to_string(partitionIndex) << std::endl;
	}
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQ_MP<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::executeCalculatePartitionController(
		size_t threadId,
		const size_t& partitionIndex)  {

	size_t kCompleted_local;
	int N = BASE::SsTimeTrajectoryStock_[partitionIndex].size();
	int k = -1;  // to make use that the while loop runs at least once

	while(k+1 < N)  {

		// update k
		k = kTaken_ctrl_[partitionIndex]++;

		if(k<N) {

			// display
			if(BASE::settings_.debugPrintMP_ && k%10 == 0) {
				BASE::printString("[MP]: [Thread " + std::to_string(threadId) + "]: Start calculating controller on index k = "
						+ std::to_string(k) + " out of " + std::to_string(N-1));
			}

			// execute calculateControllerWorker for the given partition and time node index (k)
			BASE::calculateControllerWorker(threadId, partitionIndex, k);

			// increment the number of completed nodes
			kCompleted_local = ++kCompleted_ctrl_[partitionIndex];
		}

	}  // enf of while loop

	// all k's are already covered. If all the nodes are completed notify and return, else just return.
	if(kCompleted_local >= N) {

		std::unique_lock<std::mutex> lock (kCompletedMutex_);
		kCompletedCondition_.notify_all();
		lock.unlock();

		// display
		if(BASE::settings_.debugPrintMP_) {
			BASE::printString("[MP]: [Thread " + std::to_string(threadId) + "], subsystem " + std::to_string(partitionIndex)
				+ ": leaving executeCalculatePartitionController() AND NOTIFYING ");
		}

	} else {
		// display
		if(BASE::settings_.debugPrintMP_) {
			BASE::printString("[MP]: [Thread " + std::to_string(threadId) + "], subsystem " + std::to_string(partitionIndex)
				+ ": leaving executeCalculatePartitionController() but NOT notifying ");
		}
	}
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQ_MP<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::executeLineSearchWorker(size_t threadId)
{
	if(BASE::settings_.debugPrintMP_)
		BASE::printString("[MP]: [Thread " + std::to_string(threadId) + "]: Starting executeLineSearchWorker. ");

	// local search forward simulation's variables
	scalar_t lsTotalCost;
	scalar_t lsConstraint1ISE, lsConstraint2ISE;
	scalar_t lsConstraint1MaxNorm, lsConstraint2MaxNorm;
	controller_array_t          lsControllersStock(BASE::numPartitionings_);
	std::vector<scalar_array_t>	lsTimeTrajectoriesStock(BASE::numPartitionings_);
	std::vector<size_array_t>	lsEventsPastTheEndIndecesStock(BASE::numPartitionings_);
	state_vector_array2_t   	lsStateTrajectoriesStock(BASE::numPartitionings_);
	input_vector_array2_t 		lsInputTrajectoriesStock(BASE::numPartitionings_);

	while(true)  {

		size_t alphaExp = alphaTaken_++;
		scalar_t learningRate = BASE::maxLearningRate_ * std::pow(BASE::settings_.lineSearchContractionRate_, alphaExp);

		// break condition
		if (learningRate<BASE::settings_.minLearningRateGSLQP_ || alphaBestFound_.load()==true) {

			// dispaly
			if(BASE::settings_.debugPrintMP_)  {
				if (alphaBestFound_.load()==true)
					BASE::printString("[MP]: [Thread " + std::to_string(threadId)
						+ "]: Leaving executeLineSearchWorker because best alpha is found OR no improvement for any alpha");
				else
					BASE::printString("[MP]: [Thread "+ std::to_string(threadId)
						+ "]: Leaving executeLineSearchWorker because learningRate is less than settings_.minLearningRateGSLQP_");
			}

			break;
		}

		// dispaly
		if(BASE::settings_.debugPrintMP_)
			BASE::printString("[MP]: [Thread " + std::to_string(threadId) + "]: Trying learningRate " + std::to_string(learningRate));

		// do a line search
		lsControllersStock = BASE::initLScontrollersStock_;
		BASE::lineSearchWorker(threadId, learningRate,
				lsTotalCost,
				lsConstraint1ISE, lsConstraint1MaxNorm,
				lsConstraint2ISE, lsConstraint2MaxNorm,
				lsControllersStock,
				lsTimeTrajectoriesStock, lsEventsPastTheEndIndecesStock,
				lsStateTrajectoriesStock, lsInputTrajectoriesStock);

		// break condition: make sure we do not alter an existing result
		if (alphaBestFound_.load() == true)  {
			// display
			if(BASE::settings_.debugPrintMP_)
				BASE::printString("[MP]: [Thread " + std::to_string(threadId)
					+ "]: Leaving executeLineSearchWorker because best alpha already found by another thread.");

			break;
		}


		lineSearchResultMutex_.lock();

		// Based on the LS policy check whether the best solution should be updated with these results.
		bool updatePolicy = false;
		if (BASE::settings_.lsStepsizeGreedy_==true)  {

			/*
			 * Use stepsize greedy where cost should be better than the last iteration but learning rate
			 * should be as high as possible. This is equivalent to a single core lineSearch.
			 */

			if(lsTotalCost<(baselineTotalCost_*(1-1e-3*learningRate)) && learningRate>BASE::learningRateStar_)  {
				updatePolicy = true;
				// display
				if(BASE::settings_.debugPrintMP_) {
					BASE::printString("[MP]: [LS, Thread " + std::to_string(threadId) + "]: stepsize-greedy mode: better stepsize and cost found: "
							+ std::to_string(lsTotalCost) + " at learningRate: " + std::to_string(learningRate));
				}
			} else {
				// display
				if(BASE::settings_.debugPrintMP_) {
					BASE::printString("[MP]: [LS, Thread " + std::to_string(threadId) + "]: stepsize-greedy mode: no better combination found, cost "
							+ std::to_string(lsTotalCost) + " at learningRate: " + std::to_string(learningRate));
				}
			}

		} else {

			/*
			 * line search acts cost greedy which minimize cost as much as possible
			 */

			if(lsTotalCost < (BASE::nominalTotalCost_*(1-1e-3*learningRate)))  {
				updatePolicy = true;
				if(BASE::settings_.debugPrintMP_){
					BASE::printString("[MP]: [LS, Thread " + std::to_string(threadId) + "]: cost-greedy mode : better cost found: "
							+ std::to_string(lsTotalCost) + " at learningRate: " + std::to_string(learningRate));
				}
			} else {
				if(BASE::settings_.debugPrintMP_){
					BASE::printString("[MP]: [LS, Thread " + std::to_string(threadId) + "]: cost-greedy mode : no better cost found, cost "
							+ std::to_string(lsTotalCost) + " at learningRate: " + std::to_string(learningRate) + ". Best cost was "
							+ std::to_string(BASE::nominalTotalCost_));
				}
			}
		}


		if (updatePolicy == true) {
			alphaExpBest_ = alphaExp;
			BASE::nominalTotalCost_ = lsTotalCost;
			BASE::learningRateStar_ = learningRate;
			BASE::nominalConstraint1ISE_ = lsConstraint1ISE;
			BASE::nominalConstraint1MaxNorm_ = lsConstraint1MaxNorm;
			BASE::nominalConstraint2ISE_ = lsConstraint2ISE;
			BASE::nominalConstraint2MaxNorm_ = lsConstraint2MaxNorm;

			BASE::nominalControllersStock_.swap(lsControllersStock);
			BASE::nominalTimeTrajectoriesStock_.swap(lsTimeTrajectoriesStock);
			BASE::nominalEventsPastTheEndIndecesStock_.swap(lsEventsPastTheEndIndecesStock);
			BASE::nominalStateTrajectoriesStock_.swap(lsStateTrajectoriesStock);
			BASE::nominalInputTrajectoriesStock_.swap(lsInputTrajectoriesStock);
		}

		alphaProcessed_[alphaExp] = true;

		// we now check if all alphas prior to the best have been processed, this also covers the case that there is no better alpha
		bool allPreviousAlphasProcessed = true;
		for (size_t i=0; i<alphaExpBest_; i++)
			if (alphaProcessed_[i]==false) {
				allPreviousAlphasProcessed = false;
				break;
			}

		if (allPreviousAlphasProcessed==true)  {
			alphaBestFound_ = true;
			event_handler_t::ActivateKillIntegration();	// kill all integrators
			if (BASE::settings_.displayInfo_) {
				BASE::printString("\t LS: terminate other rollouts with different alphas. alpha_best found or terminating without improvement.");
			}
		}

		lineSearchResultMutex_.unlock();

	}  // end of while loop

	// add to the number of threads that finished their tasks
	lsWorkerCompleted_++;

	if(BASE::settings_.debugPrintMP_)
		BASE::printString("[MP]: [Thread " + std::to_string(threadId) + "]: Leaving executeLineSearchWorker ");

	if (lsWorkerCompleted_.load() >= BASE::settings_.nThreads_)  {
		std::unique_lock<std::mutex> lock (alphaBestFoundMutex_);
		alphaBestFoundCondition_.notify_all();
		lock.unlock();

		if(BASE::settings_.debugPrintMP_)
			BASE::printString("[MP]: NOTIFYING LS WORKERs since all workers are now done.");
	}
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQ_MP<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::solveSequentialRiccatiEquations(
		const state_matrix_t& SmFinal,
		const state_vector_t& SvFinal,
		const eigen_scalar_t& sFinal){

	numSubsystemsProcessed_ = 0;
//	if (async_==true)
//		for (size_t i=0; i < BASE::numPartitionings_; i++){
//			subsystemsDone_[i] = false;
//			subsystemsProcessing_[i] = false;
//		}

	BASE::SmFinalStock_[BASE::finalActivePartition_+1]  = SmFinal;
	BASE::SvFinalStock_[BASE::finalActivePartition_+1]  = SvFinal;
	BASE::SveFinalStock_[BASE::finalActivePartition_+1] = state_vector_t::Zero();
	BASE::sFinalStock_[BASE::finalActivePartition_+1]   = sFinal;

	// solve it sequentially for the first time when useParallelRiccatiSolverFromInitItr_ is false
	if(BASE::useParallelRiccatiSolverFromInitItr_==false && BASE::iteration_ == 0) {

		for (int i=BASE::numPartitionings_-1; i>=0; i--)  {

			if (i < (signed)BASE::initActivePartition_ || i > (signed)BASE::finalActivePartition_) {

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

			// for each partition, there is one worker
			const size_t workerIndex = 0;

			// solve backward pass
			if (BASE::settings_.useRiccatiSolver_==true) {
				BASE::solveRiccatiEquationsWorker(workerIndex, i,
						BASE::SmFinalStock_[i+1], BASE::SvFinalStock_[i+1], BASE::sFinalStock_[i+1]);
				BASE::solveErrorRiccatiEquationWorker(workerIndex, i,
						BASE::SveFinalStock_[i+1]);
			} else {
				scalar_t constraintStepSize = BASE::updateFeedForwardPoliciesStock_[i] ? BASE::settings_.constraintStepSize_ : 0.0;
				BASE::fullRiccatiBackwardSweepWorker(workerIndex, i,
						BASE::SmFinalStock_[i+1], BASE::SvFinalStock_[i+1], BASE::SveFinalStock_[i+1], BASE::sFinalStock_[i+1],
						constraintStepSize);
			}

			// set the final value for next Riccati equation
			BASE::sFinalStock_[i]   = BASE::sTrajectoryStock_[i].front();
			BASE::SvFinalStock_[i]  = BASE::SvTrajectoryStock_[i].front();
			BASE::SveFinalStock_[i] = BASE::SveTrajectoryStock_[i].front();
			BASE::SmFinalStock_[i]  = BASE::SmTrajectoryStock_[i].front();

		}
	}
	// solve it in parallel if useParallelRiccatiSolverFromInitItr_ is true
	else {

		if(BASE::settings_.debugPrintMP_)
			BASE::printString("[MP]: Waking up workers to do RiccatiSolver Task.");

		workerTask_ = SOLVE_RICCATI;
		std::unique_lock<std::mutex> lock (workerWakeUpMutex_);
		workerWakeUpCondition_.notify_all();
		lock.unlock();

		if(BASE::settings_.debugPrintMP_)
			BASE::printString("[MP]: Will wait now until workers have done RiccatiSolver Task.");

		std::unique_lock<std::mutex> waitLock(riccatiSolverBarrierMutex_);
		while(numSubsystemsProcessed_.load() < BASE::numPartitionings_){
			riccatiSolverCompletedCondition_.wait(waitLock);
		}
		waitLock.unlock();

		workerTask_ = IDLE;
	}

	// state at the switching times
	BASE::xFinalStock_[BASE::finalActivePartition_+1] = BASE::nominalStateTrajectoriesStock_[BASE::finalActivePartition_].back();
	for (size_t i=BASE::initActivePartition_; i<=BASE::finalActivePartition_; i++)
		BASE::xFinalStock_[i] = BASE::nominalStateTrajectoriesStock_[i].front();

	if(BASE::settings_.debugPrintMP_){
		BASE::printString("[MP]: Iteration: " + std::to_string(BASE::iteration_) + " done.");
		BASE::printString("----------------------------------");
		BASE::printString("----------------------------------");
	}

}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQ_MP<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::executeRiccatiSolver(size_t threadId) {

	bool endSubsystemIsUpdated = false;

	for (int i = endingIndicesRiccatiWorker_[threadId]; i >= startingIndicesRiccatiWorker_[threadId]; i--) {

		if(BASE::settings_.debugPrintMP_)
			BASE::printString("[MP]: Thread " + std::to_string(threadId) + " processing subsystem " + std::to_string(i));


		// for inactive subsystems
		if (i < (signed)BASE::initActivePartition_ || i > (signed)BASE::finalActivePartition_) {

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

		if (BASE::settings_.useRiccatiSolver_==true) {
			BASE::solveRiccatiEquationsWorker(threadId, i,
					SmFinal, SvFinal, sFinal);
			BASE::solveErrorRiccatiEquationWorker(threadId, i,
					SveFinal);
		} else {
			scalar_t constraintStepSize = BASE::updateFeedForwardPoliciesStock_[i] ? BASE::settings_.constraintStepSize_ : 0.0;
			BASE::fullRiccatiBackwardSweepWorker(threadId, i,
					SmFinal, SvFinal, SveFinal, sFinal,
					constraintStepSize);
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
void SLQ_MP<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::distributeWork(){

	const int N = BASE::settings_.nThreads_;
	startingIndicesRiccatiWorker_.resize(N);
	endingIndicesRiccatiWorker_.resize(N);

//	if (BASE::numPartitionings_ < N)
//		throw std::runtime_error("Number of threads is bigger than number of subsystems");

	int subsystemsPerThread = (BASE::finalActivePartition_-BASE::initActivePartition_+1) / N;
	int remainingSubsystems = (BASE::finalActivePartition_-BASE::initActivePartition_+1) % N;

	int startingId, endingId = BASE::finalActivePartition_;
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
	endingIndicesRiccatiWorker_.front() = BASE::numPartitionings_-1;
	startingIndicesRiccatiWorker_.back() = 0;

	if (BASE::settings_.displayInfo_) {
		std::cout << "Initial Active Subsystem: " << BASE::initActivePartition_ << std::endl;
		std::cout << "Final Active Subsystem:   " << BASE::finalActivePartition_ << std::endl;
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
void SLQ_MP<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::runInit() {

	//distribute work
	distributeWork();

	// run BASE routine
	BASE::runInit();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQ_MP<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::runIteration() {

	// disable Eigen multi-threading
	Eigen::setNbThreads(1);

	// run BASE routine
	BASE::runIteration();

	// restore default Eigen thread number
	Eigen::setNbThreads(0);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQ_MP<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::runExit() {

	// disable Eigen multi-threading
	Eigen::setNbThreads(1);

	// linearizing the dynamics and quadratizing the cost function along nominal trajectories
	BASE::approximateOptimalControlProblem();

	// solve Riccati equations
	solveSequentialRiccatiEquations(BASE::SmHeuristics_, BASE::SvHeuristics_, BASE::sHeuristics_);

	// restore default Eigen thread number
	Eigen::setNbThreads(0);

	// calculate the nominal co-state
	BASE::calculateRolloutCostate(BASE::nominalTimeTrajectoriesStock_, BASE::nominalStateTrajectoriesStock_, BASE::nominalcostateTrajectoriesStock_);
}


} // namespace ocs2
