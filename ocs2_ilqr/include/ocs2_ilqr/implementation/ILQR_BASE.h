/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

namespace ocs2
{

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
ILQR_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::ILQR_BASE(
		const controlled_system_base_t* systemDynamicsPtr,
		const derivatives_base_t* systemDerivativesPtr,
		const constraint_base_t* systemConstraintsPtr,
		const cost_function_base_t* costFunctionPtr,
		const operating_trajectories_base_t* operatingTrajectoriesPtr,
		const ILQR_Settings& settings /*= ILQR_Settings()*/,
		const LOGIC_RULES_T* logicRulesPtr /*= nullptr*/,
		const cost_function_base_t* heuristicsFunctionPtr /* = nullptr*/)

	: BASE()
	, settings_(settings)
	, costDesiredTrajectories_()
	, costDesiredTrajectoriesBuffer_()
	, costDesiredTrajectoriesUpdated_(false)
	, rewindCounter_(0)
{
	if (logicRulesPtr != nullptr)
		logicRulesMachinePtr_ = logic_rules_machine_ptr_t( new logic_rules_machine_t(*logicRulesPtr) );
	else
		logicRulesMachinePtr_ = logic_rules_machine_ptr_t( new logic_rules_machine_t(LOGIC_RULES_T()) );

	// Dynamics, Constraints, derivatives, and cost
	linearQuadraticApproximatorPtrStock_.clear();
	linearQuadraticApproximatorPtrStock_.reserve(settings_.nThreads_);
	heuristicsFunctionsPtrStock_.clear();
	heuristicsFunctionsPtrStock_.reserve(settings_.nThreads_);
	penaltyPtrStock_.clear();
	penaltyPtrStock_.reserve(settings_.nThreads_);

	dynamicsForwardRolloutPtrStock_.resize(settings_.nThreads_);
	operatingTrajectoriesRolloutPtrStock_.resize(settings_.nThreads_);

	// initialize all subsystems, etc.
	for (size_t i=0; i<settings_.nThreads_; i++) {

		// initialize rollout
		dynamicsForwardRolloutPtrStock_[i].reset( new time_triggered_rollout_t(
				*systemDynamicsPtr, settings_.rolloutSettings_, "ILQR") );

		// initialize operating points
		operatingTrajectoriesRolloutPtrStock_[i].reset( new operating_trajectorie_rollout_t(
				*operatingTrajectoriesPtr, settings_.rolloutSettings_, "ILQR") );

		// initialize LQ approximator
		linearQuadraticApproximatorPtrStock_.emplace_back(new linear_quadratic_approximator_t(
				*systemDerivativesPtr, *systemConstraintsPtr, *costFunctionPtr, "ILQR", settings_.checkNumericalStability_) );

		// initialize operating trajectories
		operatingTrajectoriesPtrStock_.emplace_back( operatingTrajectoriesPtr->clone() );

		// initialize heuristics functions
		if (heuristicsFunctionPtr != nullptr)
			heuristicsFunctionsPtrStock_.emplace_back( heuristicsFunctionPtr->clone() );
		else // use the cost function if no heuristics function is defined
			heuristicsFunctionsPtrStock_.emplace_back( costFunctionPtr->clone() );

		// initialize penalty functions
		penaltyPtrStock_.emplace_back(
				std::shared_ptr<PenaltyBase<STATE_DIM, INPUT_DIM>>(new RelaxedBarrierPenalty<STATE_DIM, INPUT_DIM>(
						settings_.inequalityConstraintMu_,
						settings_.inequalityConstraintDelta_))
		);

	} // end of i loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
ILQR_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::~ILQR_BASE()  {

#ifdef BENCHMARK
	auto BENCHMARK_total = BENCHMARK_tAvgFP_ + BENCHMARK_tAvgBP_ + BENCHMARK_tAvgLQ_;
	if (BENCHMARK_total>0 && (settings_.displayInfo_  || settings_.displayShortSummary_)) {
		std::cerr << std::endl << "#####################################################" << std::endl;
		std::cerr << "Benchmarking over " << BENCHMARK_nIterationsBP_ << " samples." << std::endl;
		std::cerr << "Average time for Forward Pass:      " << BENCHMARK_tAvgFP_ << " [ms] \t(" <<
				BENCHMARK_tAvgFP_/BENCHMARK_total*100 << "%)" << std::endl;
		std::cerr << "Average time for Backward Pass:     " << BENCHMARK_tAvgBP_ << " [ms] \t(" <<
				BENCHMARK_tAvgBP_/BENCHMARK_total*100 << "%)" << std::endl;
		std::cerr << "Average time for LQ Approximation:  " << BENCHMARK_tAvgLQ_ << " [ms] \t(" <<
				BENCHMARK_tAvgLQ_/BENCHMARK_total*100 << "%)" << std::endl;
	}
#endif
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ILQR_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::reset() {

	iteration_ = 0;
	rewindCounter_ = 0;

	learningRateStar_ = 1.0;
	maxLearningRate_  = 1.0;
	constraintStepSize_ = 1.0;

	blockwiseMovingHorizon_ = false;
	useParallelRiccatiSolverFromInitItr_ = false;

	costDesiredTrajectories_.clear();
	costDesiredTrajectoriesBuffer_.clear();
	costDesiredTrajectoriesUpdated_ = false;

	for (size_t i=0; i<numPartitions_; i++) {

		// very important :)
		nominalControllersStock_[i].clear();

		// for Riccati equation parallel computation
		SmFinalStock_[i]  = state_matrix_t::Zero();
		SvFinalStock_[i]  = state_vector_t::Zero();
		SveFinalStock_[i] = state_vector_t::Zero();
		sFinalStock_[i]   = eigen_scalar_t::Zero();
		xFinalStock_[i]   = state_vector_t::Zero();
	}  // end of i loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
typename ILQR_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::scalar_t
	ILQR_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::rolloutTrajectory(
		const scalar_t& initTime,
		const state_vector_t& initState,
		const scalar_t& finalTime,
		const scalar_array_t& partitioningTimes,
		const controller_array_t& controllersStock,
		std::vector<scalar_array_t>& timeTrajectoriesStock,
		std::vector<size_array_t>& eventsPastTheEndIndecesStock,
		state_vector_array2_t& stateTrajectoriesStock,
		input_vector_array2_t& inputTrajectoriesStock,
		size_t threadId /*= 0*/)  {

	size_t numPartitions = partitioningTimes.size()-1;

	if (controllersStock.size() != numPartitions)
		throw std::runtime_error("controllersStock has less controllers then the number of subsystems");

	timeTrajectoriesStock.resize(numPartitions);
	eventsPastTheEndIndecesStock.resize(numPartitions);
	stateTrajectoriesStock.resize(numPartitions);
	inputTrajectoriesStock.resize(numPartitions);

	// finding the active subsystem index at initTime
	size_t initActivePartition = BASE::findActivePartitionIndex(partitioningTimes, initTime);
	// finding the active subsystem index at initTime
	size_t finalActivePartition = BASE::findActivePartitionIndex(partitioningTimes, finalTime);

	scalar_t t0 = initTime;
	state_vector_t x0 = initState;
	scalar_t tf;
	size_t numSteps = 0;
	for (size_t i=0; i<numPartitions; i++)  {

		// for subsystems before the initial time
		if (i<initActivePartition || i>finalActivePartition) {
			timeTrajectoriesStock[i].clear();
			eventsPastTheEndIndecesStock[i].clear();
			stateTrajectoriesStock[i].clear();
			inputTrajectoriesStock[i].clear();
			continue;
		}

		// final time
		tf = (i != finalActivePartition) ? partitioningTimes[i+1] : finalTime;

		// if blockwiseMovingHorizon_ is not set, use the previous partition's controller for
		// the first rollout of the partition. However for the very first run of the ILQR
		// it will still use operating trajectories if an initial controller is not provided.
		const controller_t* controllerPtrTemp = &controllersStock[i];
		if (blockwiseMovingHorizon_==false)
			if (controllerPtrTemp->empty()==true && i>0 && controllersStock[i-1].empty()==false)
				controllerPtrTemp = &controllersStock[i-1];

		// call rollout worker for the partition 'i' on the thread 'threadId'
		state_vector_t x0Temp;
		if (controllerPtrTemp->empty()==false) {
			x0Temp = dynamicsForwardRolloutPtrStock_[threadId]->run(
					i, t0, x0, tf, *controllerPtrTemp, *logicRulesMachinePtr_,
					timeTrajectoriesStock[i], eventsPastTheEndIndecesStock[i],
					stateTrajectoriesStock[i], inputTrajectoriesStock[i]);

		} else {
			x0Temp = operatingTrajectoriesRolloutPtrStock_[threadId]->run(
					i, t0, x0, tf, *controllerPtrTemp, *logicRulesMachinePtr_,
					timeTrajectoriesStock[i], eventsPastTheEndIndecesStock[i],
					stateTrajectoriesStock[i], inputTrajectoriesStock[i]);
		}
		// if there was an event time at the end of the previous partition
		if (initActivePartition<i && eventsPastTheEndIndecesStock[i-1].size()>0)
			if(eventsPastTheEndIndecesStock[i-1].back()==stateTrajectoriesStock[i-1].size()) {
				timeTrajectoriesStock[i-1].push_back(t0);
				stateTrajectoriesStock[i-1].push_back(x0);
				inputTrajectoriesStock[i-1].push_back(inputTrajectoriesStock[i].front());
			}

		// reset the initial time and state
		t0 = timeTrajectoriesStock[i].back();
		x0.swap(x0Temp);

		// total number of steps
		numSteps += timeTrajectoriesStock[i].size();

	}  // end of i loop

	// if there is an active event at the finalTime, we remove it.
	if (eventsPastTheEndIndecesStock[finalActivePartition].size()>0)
		if(eventsPastTheEndIndecesStock[finalActivePartition].back()==stateTrajectoriesStock[finalActivePartition].size()) {
			eventsPastTheEndIndecesStock[finalActivePartition].pop_back();
		}

	if (x0 != x0)
		throw std::runtime_error("System became unstable during the ILQR rollout.");

	// debug print
	if (settings_.debugPrintRollout_ == true)
		for (size_t i=0; i<numPartitions; i++)  {
			rollout_base_t::display(i, timeTrajectoriesStock[i], eventsPastTheEndIndecesStock[i],
					stateTrajectoriesStock[i], inputTrajectoriesStock[i]);
		}

	// average time step
	return (finalTime-initTime)/(scalar_t)numSteps;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ILQR_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::calculateConstraintsWorker(
		size_t workerIndex,
		const size_t& partitionIndex,
		const scalar_array_t& timeTrajectory,
		const size_array_t& eventsPastTheEndIndeces,
		const state_vector_array_t& stateTrajectory,
		const input_vector_array_t& inputTrajectory,
		size_array_t& nc1Trajectory,
		constraint1_vector_array_t& EvTrajectory,
		size_array_t& nc2Trajectory,
		constraint2_vector_array_t& HvTrajectory,
		size_array_t& ncIneqTrajectory,
		scalar_array2_t& hTrajectory,
		size_array_t& nc2Finals,
		constraint2_vector_array_t& HvFinals) {

	constraint_base_t& systemConstraints = linearQuadraticApproximatorPtrStock_[workerIndex]->systemConstraints();

	size_t N = timeTrajectory.size();

	// initialize subsystem i constraint
	if (N>0)
		systemConstraints.initializeModel(*logicRulesMachinePtr_, partitionIndex, "ILQR");

	// constraint type 1 computations which consists of number of active constraints at each time point
	// and the value of the constraint (if the rollout is constrained the value is always zero otherwise
	// it is nonzero)
	nc1Trajectory.resize(N);
	EvTrajectory.resize(N);

	// constraint type 2 computations which consists of number of active constraints at each time point
	// and the value of the constraint
	nc2Trajectory.resize(N);
	HvTrajectory.resize(N);

	// Inequality constraints
	ncIneqTrajectory.resize(N);
	hTrajectory.resize(N);

	nc2Finals.clear();
	nc2Finals.reserve(eventsPastTheEndIndeces.size());
	HvFinals.clear();
	HvFinals.reserve(eventsPastTheEndIndeces.size());

	auto eventsPastTheEndItr = eventsPastTheEndIndeces.begin();

	// compute constraint1 trajectory for subsystem i
	for (size_t k=0; k<N; k++) {

		// set data
		systemConstraints.setCurrentStateAndControl(
				timeTrajectory[k], stateTrajectory[k], inputTrajectory[k]);

		// constraint 1 type
		nc1Trajectory[k] = systemConstraints.numStateInputConstraint(timeTrajectory[k]);
		systemConstraints.getConstraint1(EvTrajectory[k]);
		if (nc1Trajectory[k] > INPUT_DIM)
			throw std::runtime_error("Number of active type-1 constraints should be less-equal to the number of input dimension.");

		// constraint type 2
		nc2Trajectory[k] = systemConstraints.numStateOnlyConstraint(timeTrajectory[k]);
		systemConstraints.getConstraint2(HvTrajectory[k]);
		if (nc2Trajectory[k] > INPUT_DIM)
			throw std::runtime_error("Number of active type-2 constraints should be less-equal to the number of input dimension.");

		// inequality constraints
		ncIneqTrajectory[k] = systemConstraints.numInequalityConstraint(timeTrajectory[k]);
		if (ncIneqTrajectory[k] > 0){
			systemConstraints.getInequalityConstraint(hTrajectory[k]);
		}

		// switching time state-constraints
		if (eventsPastTheEndItr!=eventsPastTheEndIndeces.end() && k+1==*eventsPastTheEndItr) {
			size_t nc2Final;
			constraint2_vector_t HvFinal;
			nc2Final = systemConstraints.numStateOnlyFinalConstraint(timeTrajectory[k]);
			systemConstraints.getFinalConstraint2(HvFinal);
			if (nc2Final > INPUT_DIM)
				throw std::runtime_error("Number of active type-2 constraints at final time should be less-equal to the number of input dimension.");

			nc2Finals.push_back(std::move( nc2Final ));
			HvFinals.push_back(std::move( HvFinal ));
			eventsPastTheEndItr++;
		}

	}  // end of k loop
}

/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ILQR_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::calculateRolloutConstraints(
		const std::vector<scalar_array_t>& timeTrajectoriesStock,
		const std::vector<size_array_t>& eventsPastTheEndIndecesStock,
		const state_vector_array2_t& stateTrajectoriesStock,
		const input_vector_array2_t& inputTrajectoriesStock,
		std::vector<size_array_t>& nc1TrajectoriesStock,
		constraint1_vector_array2_t& EvTrajectoryStock,
		std::vector<size_array_t>& nc2TrajectoriesStock,
		constraint2_vector_array2_t& HvTrajectoryStock,
		std::vector<size_array_t>& ncIneqTrajectoriesStock,
		scalar_array3_t& hTrajectoryStock,
		std::vector<size_array_t>& nc2FinalStock,
		constraint2_vector_array2_t& HvFinalStock,
		size_t threadId /*= 0*/) {

	// calculate constraint violations
	// constraint type 1 computations which consists of number of active constraints at each time point
	// and the value of the constraint (if the rollout is constrained the value is always zero otherwise
	// it is nonzero)
	nc1TrajectoriesStock.resize(numPartitions_);
	EvTrajectoryStock.resize(numPartitions_);

	// constraint type 2 computations which consists of number of active constraints at each time point
	// and the value of the constraint
	nc2TrajectoriesStock.resize(numPartitions_);
	HvTrajectoryStock.resize(numPartitions_);
	nc2FinalStock.resize(numPartitions_);
	HvFinalStock.resize(numPartitions_);

	// Inequality constraints
	ncIneqTrajectoriesStock.resize(numPartitions_);
	hTrajectoryStock.resize(numPartitions_);

	for (size_t i=0; i<numPartitions_; i++) {

		calculateConstraintsWorker(threadId, i,
				timeTrajectoriesStock[i], eventsPastTheEndIndecesStock[i],
				stateTrajectoriesStock[i], inputTrajectoriesStock[i],
				nc1TrajectoriesStock[i], EvTrajectoryStock[i],
				nc2TrajectoriesStock[i], HvTrajectoryStock[i],
				ncIneqTrajectoriesStock[i], hTrajectoryStock[i],
				nc2FinalStock[i], HvFinalStock[i]);
	}  // end of i loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
typename ILQR_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::scalar_t
ILQR_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::calculateConstraintISE(
		const std::vector<scalar_array_t>& timeTrajectoriesStock,
		const std::vector<std::vector<size_t>>& nc1TrajectoriesStock,
		const constraint1_vector_array2_t& EvTrajectoriesStock,
		scalar_t& constraintISE)  {

	constraintISE = 0.0;
	double maxConstraintNorm = 0.0;

	scalar_t currentSquaredNormError;
	scalar_t nextSquaredNormError;

	for (size_t i=0; i<numPartitions_; i++)  {

		currentSquaredNormError = 0.0;
		nextSquaredNormError = 0.0;

		for (size_t k=0; k+1<timeTrajectoriesStock[i].size(); k++)  {

			if (k==0) {
				const size_t& nc1 = nc1TrajectoriesStock[i][0];
				if (nc1>0)
					currentSquaredNormError = EvTrajectoriesStock[i][0].head(nc1).squaredNorm();
				else
					currentSquaredNormError = 0.0;
			} else
				currentSquaredNormError = nextSquaredNormError;

			maxConstraintNorm = ((maxConstraintNorm<currentSquaredNormError)? currentSquaredNormError: maxConstraintNorm);

			const size_t& nc1 = nc1TrajectoriesStock[i][k+1];
			if (nc1>0)
				nextSquaredNormError = EvTrajectoriesStock[i][k+1].head(nc1).squaredNorm();
			else
				nextSquaredNormError = 0.0;

			constraintISE += 0.5 * (currentSquaredNormError+nextSquaredNormError) * (timeTrajectoriesStock[i][k+1]-timeTrajectoriesStock[i][k]);

		}  // end of k loop
	}  // end of i loop

	return sqrt(maxConstraintNorm);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
typename ILQR_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::scalar_t
ILQR_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::calculateInequalityConstraintPenalty (
		const std::vector<scalar_array_t> &timeTrajectoriesStock,
		const std::vector<size_array_t> &ncIneqTrajectoriesStock,
		const scalar_array3_t &hTrajectoriesStock,
		scalar_t& inequalityISE,
		size_t workerIndex /* = 0 */ ) {

		scalar_t constraintPenalty(0.0);
		scalar_t currentPenalty(0.0);
		scalar_t nextPenalty(0.0);

		inequalityISE = 0.0;
		scalar_t currentInequalityViolationSquaredNorm(0.0);
		scalar_t nextInequalityViolationSquaredNorm(0.0);

		for (size_t i=0; i<numPartitions_; i++)  {
			for (size_t k=0; k+1<timeTrajectoriesStock[i].size(); k++)  {

				if (k==0) {
					if (ncIneqTrajectoriesStock[i][0]>0){
						penaltyPtrStock_[workerIndex]->getPenaltyCost(hTrajectoriesStock[i][k], currentPenalty);
                        penaltyPtrStock_[workerIndex]->getConstraintViolationSquaredNorm(
                                hTrajectoriesStock[i][k], currentInequalityViolationSquaredNorm);
                    }
					else {
						currentPenalty = 0.0;
						currentInequalityViolationSquaredNorm = 0.0;
					}
				} else {
					currentPenalty = nextPenalty;
					currentInequalityViolationSquaredNorm = nextInequalityViolationSquaredNorm;
				}

				if (ncIneqTrajectoriesStock[i][k+1]>0){
					penaltyPtrStock_[workerIndex]->getPenaltyCost(hTrajectoriesStock[i][k+1], nextPenalty);
                    penaltyPtrStock_[workerIndex]->getConstraintViolationSquaredNorm(
                            hTrajectoriesStock[i][k+1], nextInequalityViolationSquaredNorm);
				} else {
					nextPenalty = 0.0;
                    nextInequalityViolationSquaredNorm = 0.0;
				}

                constraintPenalty += 0.5 * (currentPenalty + nextPenalty) *
                                     (timeTrajectoriesStock[i][k + 1] - timeTrajectoriesStock[i][k]);
                inequalityISE += 0.5 * (currentInequalityViolationSquaredNorm + nextInequalityViolationSquaredNorm) *
                                 (timeTrajectoriesStock[i][k + 1] - timeTrajectoriesStock[i][k]);

			}  // end of k loop
		}  // end of i loop

		return constraintPenalty;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ILQR_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::calculateCostWorker(
		size_t workerIndex,
		const size_t& partitionIndex,
		const scalar_array_t& timeTrajectory,
		const size_array_t& eventsPastTheEndIndeces,
		const state_vector_array_t& stateTrajectory,
		const input_vector_array_t& inputTrajectory,
		scalar_t& totalCost)  {

	cost_function_base_t& costFunction = linearQuadraticApproximatorPtrStock_[workerIndex]->costFunction();

	// initialize subsystem i cost
	costFunction.initializeModel(*logicRulesMachinePtr_, partitionIndex, "ILQR");
	// set desired trajectories
	costFunction.setCostDesiredTrajectories(costDesiredTrajectories_);

	totalCost = 0.0;
	auto eventsPastTheEndItr = eventsPastTheEndIndeces.begin();

	// integrates the intermediate cost using the trapezoidal approximation method
	scalar_t prevIntermediateCost = 0.0;
	scalar_t currIntermediateCost = 0.0;
	for (size_t k=0; k<timeTrajectory.size(); k++) {

		if (k>0)
			prevIntermediateCost = currIntermediateCost;

		// feed state and control to cost function
		costFunction.setCurrentStateAndControl(
				timeTrajectory[k], stateTrajectory[k], inputTrajectory[k]);
		// getIntermediateCost intermediate cost for next time step
		costFunction.getIntermediateCost(currIntermediateCost);

		if (k>0)
			totalCost += 0.5*(prevIntermediateCost+currIntermediateCost)*(timeTrajectory[k]-timeTrajectory[k-1]);

		// terminal cost at switching times
		if (eventsPastTheEndItr!=eventsPastTheEndIndeces.end() && k+1==*eventsPastTheEndItr) {
			scalar_t finalCost;
			costFunction.getTerminalCost(finalCost);
			totalCost += finalCost;

			eventsPastTheEndItr++;
		}

	}  // end of k loop
}

/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ILQR_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::calculateRolloutCost(
		const std::vector<scalar_array_t>& timeTrajectoriesStock,
		const std::vector<size_array_t>& eventsPastTheEndIndecesStock,
		const state_vector_array2_t& stateTrajectoriesStock,
		const input_vector_array2_t& inputTrajectoriesStock,
		scalar_t& totalCost,
		size_t threadId /*= 0*/)  {

	totalCost = 0.0;

	for (size_t i=0; i<numPartitions_; i++) {
		scalar_t cost;
		calculateCostWorker(threadId, i,
				timeTrajectoriesStock[i], eventsPastTheEndIndecesStock[i],
				stateTrajectoriesStock[i], inputTrajectoriesStock[i],
				cost);
		totalCost += cost;
	}  // end of i loop

	// calculate the Heuristics function at the final time
	// initialize
	heuristicsFunctionsPtrStock_[threadId]->initializeModel(*logicRulesMachinePtr_, finalActivePartition_, "ILQR");
	// set desired trajectories
	heuristicsFunctionsPtrStock_[threadId]->setCostDesiredTrajectories(costDesiredTrajectories_);
	// set state-input
	heuristicsFunctionsPtrStock_[threadId]->setCurrentStateAndControl(
			timeTrajectoriesStock[finalActivePartition_].back(),
			stateTrajectoriesStock[finalActivePartition_].back(),
			inputTrajectoriesStock[finalActivePartition_].back());
	// compute
	scalar_t sHeuristics;
	heuristicsFunctionsPtrStock_[threadId]->getTerminalCost(sHeuristics);
	totalCost += sHeuristics;
}

/*****************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ILQR_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::calculateRolloutCost(
		const std::vector<scalar_array_t>& timeTrajectoriesStock,
		const std::vector<size_array_t>& eventsPastTheEndIndecesStock,
		const state_vector_array2_t& stateTrajectoriesStock,
		const input_vector_array2_t& inputTrajectoriesStock,
		const scalar_t& constraint2ISE,
		const scalar_t& inequalityConstraintPenalty,
		const std::vector<size_array_t>& nc2FinalStock,
		const constraint2_vector_array2_t& HvFinalStock,
		scalar_t& totalCost,
		size_t threadId /*= 0*/) {

	calculateRolloutCost(timeTrajectoriesStock, eventsPastTheEndIndecesStock,
			stateTrajectoriesStock, inputTrajectoriesStock,
			totalCost,
			threadId);

	const scalar_t stateConstraintPenalty = settings_.stateConstraintPenaltyCoeff_ *
			std::pow(settings_.stateConstraintPenaltyBase_, iteration_);

	// ISE of type-2 constraint
	totalCost += 0.5 * stateConstraintPenalty * constraint2ISE;

	// Inequality constraints
	totalCost += inequalityConstraintPenalty;

	// final constraint type 2
	if (settings_.noStateConstraints_==false)
		for (size_t i=0; i<numPartitions_; i++) {
			for (size_t k=0; k<nc2FinalStock[i].size(); k++) {
				const size_t& nc2Final = nc2FinalStock[i][k];
				totalCost += 0.5 * stateConstraintPenalty * HvFinalStock[i][k].head(nc2Final).squaredNorm();
			}  // end of k loop
		}  // end of i loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ILQR_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::approximateOptimalControlProblem()  {

	for (size_t i=0; i<numPartitions_; i++) {

		// number of the intermediate LQ variables
		size_t N = nominalTimeTrajectoriesStock_[i].size();

		// system dynamics
		AmTrajectoryStock_[i].resize(N);
		BmTrajectoryStock_[i].resize(N);

		// for equality constraints
		nc1TrajectoriesStock_[i].resize(N);
		EvTrajectoryStock_[i].resize(N);
		CmTrajectoryStock_[i].resize(N);
		DmTrajectoryStock_[i].resize(N);
		nc2TrajectoriesStock_[i].resize(N);
		HvTrajectoryStock_[i].resize(N);
		FmTrajectoryStock_[i].resize(N);

		// for inequality constraints
		ncIneqTrajectoriesStock_[i].resize(N);  // ncIneq: Number of inequality constraints
		hTrajectoryStock_[i].resize(N);
		dhdxTrajectoryStock_[i].resize(N);
		ddhdxdxTrajectoryStock_[i].resize(N);
		dhduTrajectoryStock_[i].resize(N);
		ddhduduTrajectoryStock_[i].resize(N);
		ddhdudxTrajectoryStock_[i].resize(N);

		// cost function
		qTrajectoryStock_[i].resize(N);
		QvTrajectoryStock_[i].resize(N);
		QmTrajectoryStock_[i].resize(N);
		RvTrajectoryStock_[i].resize(N);
		RmTrajectoryStock_[i].resize(N);
		PmTrajectoryStock_[i].resize(N);
		RmInverseTrajectoryStock_[i].resize(N);

		// Discrete-time coefficients
		AmDtimeTrajectoryStock_[i].resize(N);
		BmDtimeTrajectoryStock_[i].resize(N);

		qDtimeTrajectoryStock_[i].resize(N);
		QvDtimeTrajectoryStock_[i].resize(N);
		QmDtimeTrajectoryStock_[i].resize(N);
		RvDtimeTrajectoryStock_[i].resize(N);
		RmDtimeTrajectoryStock_[i].resize(N);
		PmDtimeTrajectoryStock_[i].resize(N);
		RmInverseDtimeTrajectoryStock_[i].resize(N);


		// event times LQ variables
		size_t NE = nominalEventsPastTheEndIndecesStock_[i].size();

		// final state equality constraints at event times
		nc2FinalStock_[i].resize(NE);
		HvFinalStock_[i].resize(NE);
		FmFinalStock_[i].resize(NE);

		// final cost at event times
		qFinalStock_[i].resize(NE);
		QvFinalStock_[i].resize(NE);
		QmFinalStock_[i].resize(NE);

		if (N > 0) {
			for(size_t j=0; j<settings_.nThreads_; j++) {
				// initializes the model
				linearQuadraticApproximatorPtrStock_[j]->initializeModel(*logicRulesMachinePtr_, i, "ILQR");
				// set desired trajectories
				linearQuadraticApproximatorPtrStock_[j]->costFunction().setCostDesiredTrajectories(costDesiredTrajectories_);
			}  // end of j loop

			//perform the approximateSubsystemLQ for partition i
			approximatePartitionLQ(i);
		}

	}  // end of i loop

	// calculate the Heuristics function at the final time
	heuristicsFunctionsPtrStock_[0]->initializeModel(*logicRulesMachinePtr_, finalActivePartition_, "ILQR");
	heuristicsFunctionsPtrStock_[0]->setCostDesiredTrajectories(costDesiredTrajectories_);
	heuristicsFunctionsPtrStock_[0]->setCurrentStateAndControl(
			nominalTimeTrajectoriesStock_[finalActivePartition_].back(),
			nominalStateTrajectoriesStock_[finalActivePartition_].back(),
			nominalInputTrajectoriesStock_[finalActivePartition_].back());
	heuristicsFunctionsPtrStock_[0]->getTerminalCost(sHeuristics_(0));
	heuristicsFunctionsPtrStock_[0]->getTerminalCostDerivativeState(SvHeuristics_);
	heuristicsFunctionsPtrStock_[0]->getTerminalCostSecondDerivativeState(SmHeuristics_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ILQR_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::approximateLQWorker(
		size_t workerIndex,
		const size_t& partitionIndex,
		const size_t& timeIndex)  {

	// unconstrained LQ problem
	approximateUnconstrainedLQWorker(workerIndex, partitionIndex, timeIndex);

	// discretize LQ problem
	discreteLQWorker(workerIndex, partitionIndex, timeIndex);

	const scalar_t stateConstraintPenalty = settings_.stateConstraintPenaltyCoeff_ *
				pow(settings_.stateConstraintPenaltyBase_, iteration_);

//	// modify the unconstrained LQ coefficients to constrained ones
//	approximateConstrainedLQWorker(workerIndex, partitionIndex, timeIndex, stateConstraintPenalty);

	// calculate an LQ approximate of the event times process.
	approximateEventsLQWorker(workerIndex, partitionIndex, timeIndex, stateConstraintPenalty);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ILQR_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::approximateUnconstrainedLQWorker(
		size_t workerIndex,
		const size_t& i,
		const size_t& k) {

	linearQuadraticApproximatorPtrStock_[workerIndex]->approximateUnconstrainedLQProblem(
			nominalTimeTrajectoriesStock_[i][k],
			nominalStateTrajectoriesStock_[i][k],
			nominalInputTrajectoriesStock_[i][k]);

	/*
	 * linearize system dynamics
	 */
	AmTrajectoryStock_[i][k].swap(linearQuadraticApproximatorPtrStock_[workerIndex]->Am_);
	BmTrajectoryStock_[i][k].swap(linearQuadraticApproximatorPtrStock_[workerIndex]->Bm_);

	/*
	 * constraints and linearized constraints
	 */
	// State-input equality constraint
	nc1TrajectoriesStock_[i][k] = linearQuadraticApproximatorPtrStock_[workerIndex]->ncEqStateInput_;
	EvTrajectoryStock_[i][k].swap(linearQuadraticApproximatorPtrStock_[workerIndex]->Ev_);
	CmTrajectoryStock_[i][k].swap(linearQuadraticApproximatorPtrStock_[workerIndex]->Cm_);
	DmTrajectoryStock_[i][k].swap(linearQuadraticApproximatorPtrStock_[workerIndex]->Dm_);
	// State-only equality constraint
	nc2TrajectoriesStock_[i][k] = linearQuadraticApproximatorPtrStock_[workerIndex]->ncEqStateOnly_;
	HvTrajectoryStock_[i][k].swap(linearQuadraticApproximatorPtrStock_[workerIndex]->Hv_);
	FmTrajectoryStock_[i][k].swap(linearQuadraticApproximatorPtrStock_[workerIndex]->Fm_);
	// Inequality constraint
	ncIneqTrajectoriesStock_[i][k] = linearQuadraticApproximatorPtrStock_[workerIndex]->ncIneq_;
	hTrajectoryStock_[i][k].swap(linearQuadraticApproximatorPtrStock_[workerIndex]->h_);
	dhdxTrajectoryStock_[i][k].swap(linearQuadraticApproximatorPtrStock_[workerIndex]->dhdx_);
	dhduTrajectoryStock_[i][k].swap(linearQuadraticApproximatorPtrStock_[workerIndex]->dhdu_);
	ddhdxdxTrajectoryStock_[i][k].swap(linearQuadraticApproximatorPtrStock_[workerIndex]->ddhdxdx_);
	ddhduduTrajectoryStock_[i][k].swap(linearQuadraticApproximatorPtrStock_[workerIndex]->ddhdudu_);
	ddhdudxTrajectoryStock_[i][k].swap(linearQuadraticApproximatorPtrStock_[workerIndex]->ddhdudx_);

	/*
	 * quadratic approximation to the cost function
	 */
	qTrajectoryStock_[i][k].swap(linearQuadraticApproximatorPtrStock_[workerIndex]->q_);
	QvTrajectoryStock_[i][k].swap(linearQuadraticApproximatorPtrStock_[workerIndex]->Qv_);
	QmTrajectoryStock_[i][k].swap(linearQuadraticApproximatorPtrStock_[workerIndex]->Qm_);
	RvTrajectoryStock_[i][k].swap(linearQuadraticApproximatorPtrStock_[workerIndex]->Rv_);
	RmTrajectoryStock_[i][k].swap(linearQuadraticApproximatorPtrStock_[workerIndex]->Rm_);
	PmTrajectoryStock_[i][k].swap(linearQuadraticApproximatorPtrStock_[workerIndex]->Pm_);
	RmInverseTrajectoryStock_[i][k].swap(linearQuadraticApproximatorPtrStock_[workerIndex]->RmInverse_);

	// making sure that constrained Qm is PSD
	if (settings_.useMakePSD_==true)
		BASE::makePSD(QmTrajectoryStock_[i][k]);

	// TODO: add support for the constrained ILQR
	if (nc1TrajectoriesStock_[i][k]!=0 || nc2TrajectoriesStock_[i][k]!=0 || ncIneqTrajectoriesStock_[i][k]!=0)
		throw std::runtime_error("We currently only support unconstrained ILQR.");
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ILQR_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::discreteLQWorker(
			size_t workerIndex,
			const size_t& i,
			const size_t& k) {

	// time step
	scalar_t dt = 0.0;
	if (k+1 < nominalTimeTrajectoriesStock_[i].size()) {
		 dt = nominalTimeTrajectoriesStock_[i][k+1] - nominalTimeTrajectoriesStock_[i][k];
	}

//	std::cerr << std::endl << "partition: " << i << std::endl;
//	std::cerr << "time: " << nominalTimeTrajectoriesStock_[i][k] << ",  dt: " << dt << std::endl;

	/*
	 * linearize system dynamics
	 */
	AmDtimeTrajectoryStock_[i][k] = AmTrajectoryStock_[i][k] * dt + state_matrix_t::Identity();
	BmDtimeTrajectoryStock_[i][k] = BmTrajectoryStock_[i][k] * dt;

	/*
	 * quadratic approximation to the cost function
	 */
	qDtimeTrajectoryStock_[i][k]  = qTrajectoryStock_[i][k] * dt;
	QvDtimeTrajectoryStock_[i][k] = QvTrajectoryStock_[i][k] * dt;
	QmDtimeTrajectoryStock_[i][k] = QmTrajectoryStock_[i][k] * dt;
	RvDtimeTrajectoryStock_[i][k] = RvTrajectoryStock_[i][k] * dt;
	RmDtimeTrajectoryStock_[i][k] = RmTrajectoryStock_[i][k] * dt;
	PmDtimeTrajectoryStock_[i][k] = PmTrajectoryStock_[i][k] * dt;
	RmInverseDtimeTrajectoryStock_[i][k] = RmInverseTrajectoryStock_[i][k] / dt;

//	std::cerr << "A:\n" << AmTrajectoryStock_[i][k] << std::endl;
//	std::cerr << "B:\n" << BmTrajectoryStock_[i][k] << std::endl;
//	std::cerr << "R:\n" << RmDtimeTrajectoryStock_[i][k] << std::endl;
//	std::cerr << "R^{-1}:\n" << RmInverseDtimeTrajectoryStock_[i][k] << std::endl;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ILQR_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::approximateEventsLQWorker(
		size_t workerIndex,
		const size_t& i,
		const size_t& k,
		const scalar_t& stateConstraintPenalty) 	{

	// if a switch took place calculate switch related variables
	size_t NE = nominalEventsPastTheEndIndecesStock_[i].size();
	for (size_t ke=0; ke<NE; ke++)  {
		if (nominalEventsPastTheEndIndecesStock_[i][ke] == k+1)  {

			linearQuadraticApproximatorPtrStock_[workerIndex]->approximateUnconstrainedLQProblemAtEventTime(
					nominalTimeTrajectoriesStock_[i][k],
					nominalStateTrajectoriesStock_[i][k],
					nominalInputTrajectoriesStock_[i][k]);

			// Final state-only equality constraint
			nc2FinalStock_[i][ke] = linearQuadraticApproximatorPtrStock_[workerIndex]->ncFinalEqStateOnly_;
			HvFinalStock_[i][ke].swap(linearQuadraticApproximatorPtrStock_[workerIndex]->HvFinal_);
			FmFinalStock_[i][ke].swap(linearQuadraticApproximatorPtrStock_[workerIndex]->FmFinal_);

			// Final cost
			qFinalStock_[i][ke].swap(linearQuadraticApproximatorPtrStock_[workerIndex]->qFinal_);
			QvFinalStock_[i][ke].swap(linearQuadraticApproximatorPtrStock_[workerIndex]->QvFinal_);
			QmFinalStock_[i][ke].swap(linearQuadraticApproximatorPtrStock_[workerIndex]->QmFinal_);

			// TODO: support for constrained ILQR
			if (nc2FinalStock_[i][ke]!=0)
				throw std::runtime_error("We currently only support unconstrained ILQR.");

			/*
			 * Modify the unconstrained LQ coefficients to constrained ones
			 */
			// final constraint type 2 coefficients
			const size_t& nc2 = nc2FinalStock_[i][ke];
			if (nc2 > 0) {
				qFinalStock_[i][ke]  += 0.5 * stateConstraintPenalty *
						HvFinalStock_[i][ke].head(nc2).transpose() * HvFinalStock_[i][ke].head(nc2);
				QvFinalStock_[i][ke] += stateConstraintPenalty *
						FmFinalStock_[i][ke].topRows(nc2).transpose() * HvFinalStock_[i][ke].head(nc2);
				QmFinalStock_[i][ke] += stateConstraintPenalty *
						FmFinalStock_[i][ke].topRows(nc2).transpose() * FmFinalStock_[i][ke].topRows(nc2);
			}

			// making sure that Qm remains PSD
			if (settings_.useMakePSD_==true)
				BASE::makePSD(QmFinalStock_[i][ke]);

			break;
		}
	}  // end of ke loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ILQR_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::calculateController() {

	for (size_t i=0; i<numPartitions_; i++)  {

		if (i<initActivePartition_ || i>finalActivePartition_) {
			nominalControllersStock_[i].clear();
			continue;
		}

		const size_t N = SsTimeTrajectoryStock_[i].size();

		nominalControllersStock_[i].time_ = SsTimeTrajectoryStock_[i];
		nominalControllersStock_[i].k_.resize(N);
		nominalControllersStock_[i].uff_.resize(N);
		nominalControllersStock_[i].deltaUff_.resize(N);

		// if the partition is not active
		if (N==0)  continue;

		// current partition update
		constraintStepSize_ = initialControllerDesignStock_[i] ? 0.0 : settings_.constraintStepSize_;

		/*
		 * perform the calculatePartitionController for partition i
		 */
		calculatePartitionController(i);

	}  // end of i loop

	// correcting for the last controller element of partitions
	for (size_t i=initActivePartition_; i<finalActivePartition_; i++) {
		nominalControllersStock_[i].k_.back()        = nominalControllersStock_[i+1].k_.front();
		nominalControllersStock_[i].uff_.back()      = nominalControllersStock_[i+1].uff_.front();
		nominalControllersStock_[i].deltaUff_.back() = nominalControllersStock_[i+1].deltaUff_.front();
	}
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ILQR_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::calculateControllerWorker (
		size_t workerIndex,
		const size_t& partitionIndex,
		const size_t& timeIndex)  {

	const size_t& i = partitionIndex;
	const size_t& k = timeIndex;

	state_vector_t nominalState = nominalStateTrajectoriesStock_[i][k];
	input_vector_t nominalInput = nominalInputTrajectoriesStock_[i][k];

	input_state_matrix_t Lm  = HmInverseTrajectoryStock_[i][k] * GmTrajectoryStock_[i][k];
	input_vector_t       Lv  = HmInverseTrajectoryStock_[i][k] * GvTrajectoryStock_[i][k];
	input_vector_t       Lve = input_vector_t::Zero();

	nominalControllersStock_[i].k_[k]   = -Lm;
	nominalControllersStock_[i].uff_[k] = nominalInput - nominalControllersStock_[i].k_[k]*nominalState;
	nominalControllersStock_[i].deltaUff_[k] = -Lv;

	// checking the numerical stability of the controller parameters
	if (settings_.checkNumericalStability_==true){
		try {
			if (!nominalControllersStock_[i].k_[k].allFinite())
				throw std::runtime_error("Feedback gains are unstable.");
			if (!nominalControllersStock_[i].deltaUff_[k].allFinite())
				throw std::runtime_error("feedForwardControl is unstable.");
		}
		catch(const std::exception& error)  {
			std::cerr << "what(): " << error.what() << " at time " << nominalControllersStock_[i].time_[k] << " [sec]." << std::endl;
		}
	}
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ILQR_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::lineSearchBase(bool computeISEs) {

	// display
	if (settings_.displayInfo_) {
		scalar_t maxDeltaUffNorm, maxDeltaUeeNorm;
		calculateControllerUpdateMaxNorm(maxDeltaUffNorm, maxDeltaUeeNorm);

		std::cerr << "max feedforward update norm:  " << maxDeltaUffNorm << std::endl;
		std::cerr << "max type-1 error update norm: " << maxDeltaUeeNorm << std::endl;
	}

	// catch the nominal trajectories for which the LQ problem is constructed and solved
	nominalPrevTimeTrajectoriesStock_.swap(nominalTimeTrajectoriesStock_);
	nominalPrevEventsPastTheEndIndecesStock_.swap(nominalEventsPastTheEndIndecesStock_);
	nominalPrevStateTrajectoriesStock_.swap(nominalStateTrajectoriesStock_);
	nominalPrevInputTrajectoriesStock_.swap(nominalInputTrajectoriesStock_);

	// perform one rollout while the input correction for the type-1 constraint is considered.
	avgTimeStepFP_ = rolloutTrajectory(initTime_, initState_, finalTime_,
			partitioningTimes_, nominalControllersStock_,
			nominalTimeTrajectoriesStock_, nominalEventsPastTheEndIndecesStock_,
			nominalStateTrajectoriesStock_, nominalInputTrajectoriesStock_);

	if (computeISEs==true) {
		// calculate constraint
		calculateRolloutConstraints(nominalTimeTrajectoriesStock_, nominalEventsPastTheEndIndecesStock_,
				nominalStateTrajectoriesStock_, nominalInputTrajectoriesStock_,
				nc1TrajectoriesStock_, EvTrajectoryStock_,nc2TrajectoriesStock_,
				HvTrajectoryStock_, ncIneqTrajectoriesStock_, hTrajectoryStock_,
				nc2FinalStock_, HvFinalStock_);
		// calculate constraint type-1 ISE and maximum norm
		nominalConstraint1MaxNorm_ = calculateConstraintISE(
				nominalTimeTrajectoriesStock_, nc1TrajectoriesStock_, EvTrajectoryStock_,
				nominalConstraint1ISE_);
		// calculates type-2 constraint ISE and maximum norm
		nominalConstraint2MaxNorm_ = calculateConstraintISE(
				nominalTimeTrajectoriesStock_, nc2TrajectoriesStock_, HvTrajectoryStock_,
				nominalConstraint2ISE_);
		// calculate penalty
		nominalInequalityConstraintPenalty_ = calculateInequalityConstraintPenalty(nominalTimeTrajectoriesStock_,
																				   ncIneqTrajectoriesStock_,
																				   hTrajectoryStock_,
																				   nominalInequalityConstraintISE_);
	} else {
		// calculate constraint type-1 ISE and maximum norm
		nominalConstraint1ISE_ = nominalConstraint1MaxNorm_ = 0.0;
		// calculates type-2 constraint ISE and maximum norm
		nominalConstraint2ISE_ = nominalConstraint2MaxNorm_ = 0.0;
		// inequality constraints
		nominalInequalityConstraintPenalty_ = 0.0;
        nominalInequalityConstraintISE_ = 0.0;
	}

	// calculates cost
	calculateRolloutCost(nominalTimeTrajectoriesStock_, nominalEventsPastTheEndIndecesStock_,
			nominalStateTrajectoriesStock_, nominalInputTrajectoriesStock_,
			nominalConstraint2ISE_, nominalInequalityConstraintPenalty_,
			nc2FinalStock_, HvFinalStock_,
			nominalTotalCost_);

	// display
	if (settings_.displayInfo_)  {
		std::cerr << "\t learningRate 0.0 \t cost: " << nominalTotalCost_ <<
		" \t constraint ISE: " << nominalConstraint1ISE_ <<
		" \t inequality penalty: " << nominalInequalityConstraintPenalty_ <<
		" \t inequality ISE: " << nominalInequalityConstraintISE_ << std::endl;
		std::cerr << "\t final constraint type-2:  ";
		size_t itr = 0;
		for(size_t i=initActivePartition_; i<=finalActivePartition_; i++)
			for (size_t k=0; k<nc2FinalStock_[i].size(); k++) {
				std::cerr << "[" << itr  << "]: " << HvFinalStock_[i][k].head(nc2FinalStock_[i][k]).transpose() << ",  ";
				itr++;
			}
		std::cerr << std::endl;
		std::cerr << "\t forward pass average time step: " << avgTimeStepFP_*1e+3 << " [ms]." << std::endl;
	}
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ILQR_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::lineSearchWorker(
		size_t workerIndex,
		scalar_t learningRate,
		scalar_t& lsTotalCost,
		scalar_t& lsConstraint1ISE, scalar_t& lsConstraint1MaxNorm,
		scalar_t& lsConstraint2ISE, scalar_t& lsConstraint2MaxNorm,
		scalar_t& lsInequalityConstraintPenalty, scalar_t& lsInequalityConstraintISE,
		controller_array_t& lsControllersStock,
		std::vector<scalar_array_t>& lsTimeTrajectoriesStock,
		std::vector<size_array_t>& lsEventsPastTheEndIndecesStock,
		state_vector_array2_t& lsStateTrajectoriesStock,
		input_vector_array2_t& lsInputTrajectoriesStock)  {

	// modifying uff by local increments
	for (size_t i=0; i<numPartitions_; i++)
		for (size_t k=0; k<lsControllersStock[i].time_.size(); k++)
			lsControllersStock[i].uff_[k] += learningRate * lsControllersStock[i].deltaUff_[k];

	try {

		// perform a rollout
		scalar_t avgTimeStepFP = rolloutTrajectory(initTime_, initState_, finalTime_,
				partitioningTimes_, lsControllersStock,
				lsTimeTrajectoriesStock, lsEventsPastTheEndIndecesStock,
				lsStateTrajectoriesStock, lsInputTrajectoriesStock,
				workerIndex);

		// calculate rollout constraints
		std::vector<size_array_t>   lsNc1TrajectoriesStock(numPartitions_);
		constraint1_vector_array2_t lsEvTrajectoryStock(numPartitions_);
		std::vector<size_array_t>   lsNc2TrajectoriesStock(numPartitions_);
		constraint2_vector_array2_t lsHvTrajectoryStock(numPartitions_);
		std::vector<size_array_t>   lsNcIneqTrajectoriesStock(numPartitions_);
		scalar_array3_t				lshTrajectoryStock(numPartitions_);
		std::vector<size_array_t>	lsNc2FinalStock(numPartitions_);
		constraint2_vector_array2_t	lsHvFinalStock(numPartitions_);


		if (lsComputeISEs_==true) {
			// calculate rollout constraints
			calculateRolloutConstraints(lsTimeTrajectoriesStock, lsEventsPastTheEndIndecesStock,
					lsStateTrajectoriesStock, lsInputTrajectoriesStock,
					lsNc1TrajectoriesStock, lsEvTrajectoryStock,
					lsNc2TrajectoriesStock, lsHvTrajectoryStock,
					lsNcIneqTrajectoriesStock, lshTrajectoryStock,
					lsNc2FinalStock, lsHvFinalStock,
					workerIndex);
			// calculate constraint type-1 ISE and maximum norm
			lsConstraint1MaxNorm = calculateConstraintISE(lsTimeTrajectoriesStock,
					lsNc1TrajectoriesStock, lsEvTrajectoryStock,
					lsConstraint1ISE);
			// calculates type-2 constraint ISE and maximum norm
			lsConstraint2MaxNorm = calculateConstraintISE(lsTimeTrajectoriesStock,
					lsNc2TrajectoriesStock, lsHvTrajectoryStock,
					lsConstraint2ISE);
			// inequalityConstraints
			lsInequalityConstraintPenalty = calculateInequalityConstraintPenalty(lsTimeTrajectoriesStock,
																				 lsNcIneqTrajectoriesStock,
																				 lshTrajectoryStock,
                                                                                 lsInequalityConstraintISE,
																				 workerIndex);
		} else {
			lsConstraint1ISE = lsConstraint1MaxNorm = 0.0;
			lsConstraint2ISE = lsConstraint2MaxNorm = 0.0;
			lsInequalityConstraintPenalty = 0.0;
            lsInequalityConstraintISE = 0.0;
		}

		// calculate rollout cost
		calculateRolloutCost(lsTimeTrajectoriesStock, lsEventsPastTheEndIndecesStock,
				lsStateTrajectoriesStock, lsInputTrajectoriesStock,
				lsConstraint2ISE, lsInequalityConstraintPenalty,
				lsNc2FinalStock, lsHvFinalStock,
				lsTotalCost,
				workerIndex);

		// display
		if (settings_.displayInfo_) {
			std::string finalConstraintDisplay;
			finalConstraintDisplay = "\t [Thread" + std::to_string(workerIndex) + "] - learningRate " + std::to_string(learningRate)
					+ " \t cost: " + std::to_string(lsTotalCost) +
					" \t constraint ISE: " + std::to_string(lsConstraint1ISE) +
					" \t inequality penalty: " + std::to_string(lsInequalityConstraintPenalty) +
                    " \t inequality ISE: " + std::to_string(lsInequalityConstraintISE) + "\n";
			finalConstraintDisplay += "\t final constraint type-2:   ";
			for(size_t i=0; i<numPartitions_; i++) {
				finalConstraintDisplay += "[" + std::to_string(i) + "]: ";
				for (size_t j=0; j<lsNc2FinalStock[i].size(); j++)
					for (size_t m=0; m<lsNc2FinalStock[i][j]; m++)
						finalConstraintDisplay += std::to_string(lsHvFinalStock[i][j](m)) + ", ";
				finalConstraintDisplay += "  ";
			} // end of i loop
			finalConstraintDisplay += "\n\t forward pass average time step: " + std::to_string(avgTimeStepFP*1e+3) + " [ms].";
			BASE::printString(finalConstraintDisplay);
		}

	} catch(const std::exception& error) {
		lsTotalCost  = std::numeric_limits<scalar_t>::max();
		if(settings_.displayInfo_)
			BASE::printString("\t [Thread" + std::to_string(workerIndex) + "] rollout with learningRate " +
					std::to_string(learningRate) + " is terminated.");
	}
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ILQR_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::solveRiccatiEquationsWorker(
		size_t workerIndex,
		const size_t& partitionIndex,
		const state_matrix_t& SmFinal,
		const state_vector_t& SvFinal,
		const eigen_scalar_t& sFinal)  {

	const size_t N  = nominalTimeTrajectoriesStock_[partitionIndex].size();
	const size_t NE = nominalEventsPastTheEndIndecesStock_[partitionIndex].size();

	const scalar_t scalingStart  = partitioningTimes_[partitionIndex];
	const scalar_t scalingFinal  = partitioningTimes_[partitionIndex+1];
	const scalar_t scalingFactor = scalingStart - scalingFinal;  // this is negative

	// normalized time
	SsNormalizedTimeTrajectoryStock_[partitionIndex].resize(N);
	for (size_t k=0; k<N; k++) {
		SsNormalizedTimeTrajectoryStock_[partitionIndex][N-1-k] =
				(nominalTimeTrajectoriesStock_[partitionIndex][k]-scalingFinal) / scalingFactor;
	}

	// normalized event past the index
	SsNormalizedEventsPastTheEndIndecesStock_[partitionIndex].resize(NE);
	for (size_t k=0; k<NE; k++) {
		SsNormalizedEventsPastTheEndIndecesStock_[partitionIndex][NE-1-k] =
				N - nominalEventsPastTheEndIndecesStock_[partitionIndex][k];
	}

	// output containers resizing
	SsTimeTrajectoryStock_[partitionIndex] = nominalTimeTrajectoriesStock_[partitionIndex];
	sTrajectoryStock_[partitionIndex].resize(N);
	SvTrajectoryStock_[partitionIndex].resize(N);
	SmTrajectoryStock_[partitionIndex].resize(N);

	HmTrajectoryStock_[partitionIndex].resize(N);
	HmInverseTrajectoryStock_[partitionIndex].resize(N);
	GmTrajectoryStock_[partitionIndex].resize(N);
	GvTrajectoryStock_[partitionIndex].resize(N);

	// terminate if the partition is not active
	if (N==0) return;

	// switching times
	size_array_t SsSwitchingTimesIndices;
	SsSwitchingTimesIndices.reserve(NE+2);
	SsSwitchingTimesIndices.push_back( 0 );
	for (size_t k=0; k<NE; k++) {
		const size_t& index = nominalEventsPastTheEndIndecesStock_[partitionIndex][k];
		SsSwitchingTimesIndices.push_back( index );
	}
	SsSwitchingTimesIndices.push_back( N );

	// final temporal values
	state_matrix_t SmFinalTemp = SmFinal;
	state_vector_t SvFinalTemp = SvFinal;
	eigen_scalar_t sFinalTemp  = sFinal;

	/*
	 * solving the Riccati equations
	 */
	int beginTimeItr;
	int endTimeItr;

	for (int i=NE; i>=0; i--) {

		beginTimeItr = SsSwitchingTimesIndices[i];   // similar to std::begin()
		endTimeItr   = SsSwitchingTimesIndices[i+1]; // similar to std::end()

		/*
		 * solution at final time of an interval (uses the continuous-time formulation)
		 */
		const size_t finalIndex = endTimeItr-1;
		// note that these are the continuous time coefficients
		const state_input_matrix_t& Bmc = BmTrajectoryStock_[partitionIndex][finalIndex];
		const input_vector_t&       Rvc = RvTrajectoryStock_[partitionIndex][finalIndex];
		const input_matrix_t&       Rmc = RmTrajectoryStock_[partitionIndex][finalIndex];
		const input_state_matrix_t& Pmc = PmTrajectoryStock_[partitionIndex][finalIndex];
		const input_matrix_t&       RmcInverse = RmInverseTrajectoryStock_[partitionIndex][finalIndex];

		sTrajectoryStock_[partitionIndex][finalIndex]  = sFinalTemp;
		SvTrajectoryStock_[partitionIndex][finalIndex] = SvFinalTemp;
		SmTrajectoryStock_[partitionIndex][finalIndex] = SmFinalTemp;

		HmTrajectoryStock_[partitionIndex][finalIndex] = Rmc;
		HmInverseTrajectoryStock_[partitionIndex][finalIndex] = RmcInverse;
		GmTrajectoryStock_[partitionIndex][finalIndex] = Pmc + Bmc.transpose() * SmFinalTemp;
		GvTrajectoryStock_[partitionIndex][finalIndex] = Rvc + Bmc.transpose() * SvFinalTemp;


		// solve Riccati equations if interval length is not zero
		if (beginTimeItr < endTimeItr-1) {
			for (int k=endTimeItr-2; k>=beginTimeItr; k--) {

				const state_matrix_t&       Am = AmDtimeTrajectoryStock_[partitionIndex][k];
				const state_input_matrix_t& Bm = BmDtimeTrajectoryStock_[partitionIndex][k];
				const eigen_scalar_t&       q  = qDtimeTrajectoryStock_[partitionIndex][k];
				const state_vector_t&       Qv = QvDtimeTrajectoryStock_[partitionIndex][k];
				const state_matrix_t&       Qm = QmDtimeTrajectoryStock_[partitionIndex][k];
				const input_vector_t&       Rv = RvDtimeTrajectoryStock_[partitionIndex][k];
				const input_matrix_t&       Rm = RmDtimeTrajectoryStock_[partitionIndex][k];
				const input_state_matrix_t& Pm = PmDtimeTrajectoryStock_[partitionIndex][k];

				input_matrix_t&       Hm = HmTrajectoryStock_[partitionIndex][k];
				input_matrix_t&       HmInverse = HmInverseTrajectoryStock_[partitionIndex][k];
				input_vector_t&       Gv = GvTrajectoryStock_[partitionIndex][k];
				input_state_matrix_t& Gm = GmTrajectoryStock_[partitionIndex][k];

				Hm = Rm + Bm.transpose() * SmTrajectoryStock_[partitionIndex][k+1] * Bm;
				HmInverse = Hm.ldlt().solve(input_matrix_t::Identity());
				Gm = Pm + Bm.transpose() * SmTrajectoryStock_[partitionIndex][k+1] * Am;
				Gv = Rv + Bm.transpose() * SvTrajectoryStock_[partitionIndex][k+1];

				sTrajectoryStock_[partitionIndex][k] = q + sTrajectoryStock_[partitionIndex][k+1] -0.5*Gv.transpose()*HmInverse*Gv;
				SvTrajectoryStock_[partitionIndex][k] = Qv + Am.transpose()*SvTrajectoryStock_[partitionIndex][k+1] - Gm.transpose()*HmInverse*Gv;
				SmTrajectoryStock_[partitionIndex][k] = Qm + Am.transpose()*SmTrajectoryStock_[partitionIndex][k+1]*Am - Gm.transpose()*HmInverse*Gm;
			}
		}

		if (i > 0) {
			sFinalTemp  = sTrajectoryStock_[partitionIndex][beginTimeItr]  + qFinalStock_[partitionIndex][i-1];
			SvFinalTemp = SvTrajectoryStock_[partitionIndex][beginTimeItr] + QvFinalStock_[partitionIndex][i-1];
			SmFinalTemp = SmTrajectoryStock_[partitionIndex][beginTimeItr] + QmFinalStock_[partitionIndex][i-1];
		}

	}  // end of i loop


	// testing the numerical stability of the Riccati equations
	if (settings_.checkNumericalStability_)
		for (int k=N-1; k>=0; k--) {
			try {
				if (!SmTrajectoryStock_[partitionIndex][k].allFinite())  throw std::runtime_error("Sm is unstable.");
				if (SmTrajectoryStock_[partitionIndex][k].eigenvalues().real().minCoeff() < -Eigen::NumTraits<scalar_t>::epsilon())
					throw std::runtime_error("Sm matrix is not positive semi-definite. It's smallest eigenvalue is " +
							std::to_string(SmTrajectoryStock_[partitionIndex][k].eigenvalues().real().minCoeff()) + ".");
				if (!SvTrajectoryStock_[partitionIndex][k].allFinite())  throw std::runtime_error("Sv is unstable.");
				if (!sTrajectoryStock_[partitionIndex][k].allFinite())   throw std::runtime_error("s is unstable");
			}
			catch(const std::exception& error)
			{
				std::cerr << "what(): " << error.what() << " at time " << SsTimeTrajectoryStock_[partitionIndex][k] << " [sec]." << std::endl;
				for (int kp=k; kp<k+10; kp++)  {
					if (kp >= N) continue;
					std::cerr << "Sm[" << SsTimeTrajectoryStock_[partitionIndex][kp] << "]:\n"<< SmTrajectoryStock_[partitionIndex][kp].norm() << std::endl;
					std::cerr << "Sv[" << SsTimeTrajectoryStock_[partitionIndex][kp] << "]:\t"<< SvTrajectoryStock_[partitionIndex][kp].transpose().norm() << std::endl;
					std::cerr << "s["  << SsTimeTrajectoryStock_[partitionIndex][kp] << "]:\t"<< sTrajectoryStock_[partitionIndex][kp].transpose().norm() << std::endl;
				}
				exit(0);
			}
		}
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ILQR_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::solveErrorRiccatiEquationWorker(
		size_t workerIndex,
		const size_t& partitionIndex,
		const state_vector_t& SveFinal)  {

}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ILQR_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::calculateControllerUpdateMaxNorm(
		scalar_t& maxDeltaUffNorm,
		scalar_t& maxDeltaUeeNorm) {

	maxDeltaUffNorm = 0.0;
	maxDeltaUeeNorm = 0.0;
	for (size_t i=initActivePartition_; i<=finalActivePartition_; i++)  {

		for (size_t k=0; k<nominalControllersStock_[i].time_.size(); k++)  {

			maxDeltaUffNorm = std::max(maxDeltaUffNorm, nominalControllersStock_[i].deltaUff_[k].norm());

			const state_vector_t& nominalState = nominalStateTrajectoriesStock_[i][k];
			const input_vector_t& nominalInput = nominalInputTrajectoriesStock_[i][k];
			input_vector_t deltaUee = nominalInput - nominalControllersStock_[i].k_[k]*nominalState - nominalControllersStock_[i].uff_[k];
			maxDeltaUeeNorm = std::max(maxDeltaUeeNorm, deltaUee.norm());

		}  // end of k loop
	}  // end of i loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ILQR_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::printRolloutInfo()  {

	std::cerr << "optimization cost:         " << nominalTotalCost_ << std::endl;
	std::cerr << "constraint type-1 ISE:     " << nominalConstraint1ISE_ << std::endl;
	std::cerr << "constraint type-1 MaxNorm: " << nominalConstraint1MaxNorm_ << std::endl;
	std::cerr << "constraint type-2 ISE:     " << nominalConstraint2ISE_ << std::endl;
	std::cerr << "constraint type-2 MaxNorm: " << nominalConstraint2MaxNorm_ << std::endl;
	std::cerr << "inequality Penalty:        " << nominalInequalityConstraintPenalty_ << std::endl;
	std::cerr << "inequality ISE:            " << nominalInequalityConstraintISE_ << std::endl;
	std::cerr << "final constraint type-2: 	 ";
	size_t itr = 0;
	for(size_t i=initActivePartition_; i<=finalActivePartition_; i++)
		for (size_t k=0; k<nc2FinalStock_[i].size(); k++) {
			std::cerr << "[" << itr  << "]: " << HvFinalStock_[i][k].head(nc2FinalStock_[i][k]).transpose() << ",  ";
			itr++;
		}
	std::cerr << std::endl;
	std::cerr << "forward pass average time step:  " << avgTimeStepFP_*1e+3 << " [ms]." << std::endl;
	std::cerr << "backward pass average time step: " << avgTimeStepBP_*1e+3 << " [ms]." << std::endl;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ILQR_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::calculateMeritFunction(
		const std::vector<scalar_array_t>& timeTrajectoriesStock,
		const std::vector<std::vector<size_t> >& nc1TrajectoriesStock,
		const constraint1_vector_array2_t& EvTrajectoryStock,
		const std::vector<std::vector<Eigen::VectorXd>>& lagrangeTrajectoriesStock,
		const scalar_t& totalCost,
		scalar_t& meritFunctionValue,
		scalar_t& constraintISE)  {

	// add cost function
	meritFunctionValue = totalCost;

	// add the L2 penalty for constraint violation
	calculateConstraintISE(timeTrajectoriesStock, nc1TrajectoriesStock, EvTrajectoryStock, constraintISE);
	double pho = 1.0;
	if (settings_.maxNumIterationsILQR_>1)
		pho = (iteration_-1)/(settings_.maxNumIterationsILQR_-1) * settings_.meritFunctionRho_;

	meritFunctionValue += 0.5*pho*constraintISE;

	// add the the lagrangian term for the constraint
	scalar_t currentIntermediateMerit;
	scalar_t nextIntermediateMerit;

	for (size_t i=0; i<numPartitions_; i++)
	{
		// integrates the intermediate merit using the trapezoidal approximation method
		currentIntermediateMerit = 0.0;
		nextIntermediateMerit = 0.0;
		for (size_t k=0; k+1<timeTrajectoriesStock[i].size(); k++)
		{
			if (k==0)
				currentIntermediateMerit = EvTrajectoryStock[i][k].head(nc1TrajectoriesStock[i][k]).transpose() * lagrangeTrajectoriesStock[i][k];
			else
				currentIntermediateMerit = nextIntermediateMerit;

			nextIntermediateMerit = EvTrajectoryStock[i][k+1].head(nc1TrajectoriesStock[i][k+1]).transpose() * lagrangeTrajectoriesStock[i][k+1];

			meritFunctionValue += 0.5*(currentIntermediateMerit+nextIntermediateMerit)*(timeTrajectoriesStock[i][k+1]-timeTrajectoriesStock[i][k]);
		}  // end of k loop
	}  // end of i loop

}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ILQR_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getValueFuntion (
		const scalar_t& time, const state_vector_t& state, scalar_t& valueFuntion)  {

	size_t activeSubsystem = BASE::findActivePartitionIndex(partitioningTimes_, time);

	state_matrix_t Sm;
	LinearInterpolation<state_matrix_t,Eigen::aligned_allocator<state_matrix_t> > SmFunc(
			&SsTimeTrajectoryStock_[activeSubsystem], &SmTrajectoryStock_[activeSubsystem]);
	SmFunc.interpolate(time, Sm);
	size_t greatestLessTimeStampIndex = SmFunc.getGreatestLessTimeStampIndex();

	state_vector_t Sv;
	LinearInterpolation<state_vector_t,Eigen::aligned_allocator<state_vector_t> > SvFunc(
			&SsTimeTrajectoryStock_[activeSubsystem], &SvTrajectoryStock_[activeSubsystem]);
	SvFunc.interpolate(time, Sv, greatestLessTimeStampIndex);

	eigen_scalar_t s;
	LinearInterpolation<eigen_scalar_t,Eigen::aligned_allocator<eigen_scalar_t> > sFunc(
			&SsTimeTrajectoryStock_[activeSubsystem], &sTrajectoryStock_[activeSubsystem]);
	sFunc.interpolate(time, s, greatestLessTimeStampIndex);

	state_vector_t xNominal;
	LinearInterpolation<state_vector_t,Eigen::aligned_allocator<state_vector_t> > xNominalFunc(
			&nominalTimeTrajectoriesStock_[activeSubsystem], &nominalStateTrajectoriesStock_[activeSubsystem]);
	xNominalFunc.interpolate(time, xNominal);

	state_vector_t deltaX = state-xNominal;

	valueFuntion = (s + deltaX.transpose()*Sv + 0.5*deltaX.transpose()*Sm*deltaX).eval()(0);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
ILQR_Settings& ILQR_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::settings() {

	return settings_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ILQR_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::useParallelRiccatiSolverFromInitItr(bool flag) {

	useParallelRiccatiSolverFromInitItr_ = flag;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ILQR_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::blockwiseMovingHorizon(bool flag) {

	blockwiseMovingHorizon_ = flag;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ILQR_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getPerformanceIndeces(
		scalar_t& costFunction,
		scalar_t& constraint1ISE,
		scalar_t& constraint2ISE) const {

	costFunction = nominalTotalCost_;
	constraint1ISE = nominalConstraint1ISE_;
	constraint2ISE = nominalConstraint2ISE_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
size_t ILQR_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getNumIterations() const {

	return iteration_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ILQR_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getIterationsLog(
		eigen_scalar_array_t& iterationCost,
		eigen_scalar_array_t& iterationISE1,
		eigen_scalar_array_t& iterationISE2) const {

	iterationCost = iterationCost_;
	iterationISE1 = iterationISE1_;
	iterationISE2 = iterationISE2_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ILQR_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getIterationsLogPtr(
		const eigen_scalar_array_t*& iterationCostPtr,
		const eigen_scalar_array_t*& iterationISE1Ptr,
		const eigen_scalar_array_t*& iterationISE2Ptr) const {

	iterationCostPtr = &iterationCost_;
	iterationISE1Ptr = &iterationISE1_;
	iterationISE2Ptr = &iterationISE2_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
const typename ILQR_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::controller_array_t&
	ILQR_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getController() const {

	return nominalControllersStock_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ILQR_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getControllerPtr(
		const controller_array_t*& controllersStockPtr) const {

	controllersStockPtr = &nominalControllersStock_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ILQR_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::swapController(
		controller_array_t& controllersStock) {

	controllersStock.swap(nominalControllersStock_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
const std::vector<typename ILQR_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::scalar_array_t>&
	ILQR_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getNominalTimeTrajectories () const {

	return nominalTimeTrajectoriesStock_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
const typename ILQR_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::state_vector_array2_t&
	ILQR_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getNominalStateTrajectories () const {

	return nominalStateTrajectoriesStock_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
const typename ILQR_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::input_vector_array2_t&
	ILQR_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getNominalInputTrajectories () const  {

	return nominalInputTrajectoriesStock_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ILQR_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getNominalTrajectoriesPtr(
		const std::vector<scalar_array_t>*& nominalTimeTrajectoriesStockPtr,
		const state_vector_array2_t*& nominalStateTrajectoriesStockPtr,
		const input_vector_array2_t*& nominalInputTrajectoriesStockPtr) const  {

	nominalTimeTrajectoriesStockPtr  = &nominalTimeTrajectoriesStock_;
	nominalStateTrajectoriesStockPtr = &nominalStateTrajectoriesStock_;
	nominalInputTrajectoriesStockPtr = &nominalInputTrajectoriesStock_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ILQR_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::swapNominalTrajectories (
		std::vector<scalar_array_t>& nominalTimeTrajectoriesStock,
		state_vector_array2_t& nominalStateTrajectoriesStock,
		input_vector_array2_t& nominalInputTrajectoriesStock)  {

	nominalTimeTrajectoriesStock.swap(nominalTimeTrajectoriesStock_);
	nominalStateTrajectoriesStock.swap(nominalStateTrajectoriesStock_);
	nominalInputTrajectoriesStock.swap(nominalInputTrajectoriesStock_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ILQR_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::truncateConterller(
		const scalar_array_t& partitioningTimes,
		const double& initTime,
		controller_array_t& controllersStock,
		size_t& initActivePartition,
		controller_array_t& deletedcontrollersStock) {

	deletedcontrollersStock.resize(numPartitions_);
	for (size_t i=0; i<numPartitions_; i++)
		deletedcontrollersStock[i].clear();

	// finding the active subsystem index at initTime_
	initActivePartition = BASE::findActivePartitionIndex(partitioningTimes, initTime);

	// saving the deleting part and clearing controllersStock
	for (size_t i=0; i<initActivePartition; i++)
		deletedcontrollersStock[i].swap(controllersStock[i]);

	if (controllersStock[initActivePartition].time_.empty()==true)  return;

	// interpolating uff
	LinearInterpolation<input_vector_t,Eigen::aligned_allocator<input_vector_t> > uffFunc;
	uffFunc.setTimeStamp(&controllersStock[initActivePartition].time_);
	uffFunc.setData(&controllersStock[initActivePartition].uff_);
	input_vector_t uffInit;
	uffFunc.interpolate(initTime, uffInit);
	size_t greatestLessTimeStampIndex = uffFunc.getGreatestLessTimeStampIndex();

	// interpolating k
	LinearInterpolation<input_state_matrix_t,Eigen::aligned_allocator<input_state_matrix_t> > kFunc;
	kFunc.setTimeStamp(&controllersStock[initActivePartition].time_);
	kFunc.setData(&controllersStock[initActivePartition].k_);
	input_state_matrix_t kInit;
	kFunc.interpolate(initTime, kInit, greatestLessTimeStampIndex);

	// deleting the controller in the active subsystem for the subsystems before initTime
	if (greatestLessTimeStampIndex>0) {

		deletedcontrollersStock[initActivePartition].time_.resize(greatestLessTimeStampIndex+1);
		deletedcontrollersStock[initActivePartition].uff_.resize(greatestLessTimeStampIndex+1);
		deletedcontrollersStock[initActivePartition].k_.resize(greatestLessTimeStampIndex+1);
		for (size_t k=0; k<=greatestLessTimeStampIndex; k++) {
			deletedcontrollersStock[initActivePartition].time_[k] = controllersStock[initActivePartition].time_[k];
			deletedcontrollersStock[initActivePartition].uff_[k] = controllersStock[initActivePartition].uff_[k];
			deletedcontrollersStock[initActivePartition].k_[k] = controllersStock[initActivePartition].k_[k];
		}

		controllersStock[initActivePartition].time_.erase (
				controllersStock[initActivePartition].time_.begin(),
				controllersStock[initActivePartition].time_.begin()+greatestLessTimeStampIndex);
		controllersStock[initActivePartition].uff_.erase (
				controllersStock[initActivePartition].uff_.begin(),
				controllersStock[initActivePartition].uff_.begin()+greatestLessTimeStampIndex);
		controllersStock[initActivePartition].k_.erase (
				controllersStock[initActivePartition].k_.begin(),
				controllersStock[initActivePartition].k_.begin()+greatestLessTimeStampIndex);
	}

	controllersStock[initActivePartition].time_[0] = initTime;
	controllersStock[initActivePartition].uff_[0] = uffInit;
	controllersStock[initActivePartition].k_[0] = kInit;

}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ILQR_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::rewindOptimizer(const size_t& firstIndex) {

	// No rewind is needed
	if (firstIndex==0)  return;

	// increment rewindCounter_
	rewindCounter_ += firstIndex;

	if (firstIndex > numPartitions_)
		throw std::runtime_error("Index for rewinding is greater than the current size.");

	const size_t preservedLength = numPartitions_ - firstIndex;
	for (size_t i=0; i<numPartitions_; i++)
		if (i<preservedLength) {
			nominalControllersStock_[i].swap(nominalControllersStock_[firstIndex+i]);
			SmFinalStock_[i]  = SmFinalStock_[firstIndex+i];
			SvFinalStock_[i]  = SvFinalStock_[firstIndex+i];
			SveFinalStock_[i] = SveFinalStock_[firstIndex+i];
			sFinalStock_[i]   = sFinalStock_[firstIndex+i];
			xFinalStock_[i]   = xFinalStock_[firstIndex+i];
		} else {
			nominalControllersStock_[i].clear();
			SmFinalStock_[i].setZero();
			SvFinalStock_[i].setZero();
			SveFinalStock_[i].setZero();
			sFinalStock_[i].setZero();
			xFinalStock_[i].setZero();
		}
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
const unsigned long long int& ILQR_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getRewindCounter() const {

	return rewindCounter_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ILQR_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::setupOptimizer(const size_t& numPartitions) {

	if (numPartitions==0)
		throw std::runtime_error("Number of partitions cannot be zero!");

	/*
	 * nominal trajectories
	 */
	nominalControllersStock_.resize(numPartitions);
	nominalTimeTrajectoriesStock_.resize(numPartitions);
	nominalEventsPastTheEndIndecesStock_.resize(numPartitions);
	nominalStateTrajectoriesStock_.resize(numPartitions);
	nominalInputTrajectoriesStock_.resize(numPartitions);

	nominalPrevTimeTrajectoriesStock_.resize(numPartitions);
	nominalPrevEventsPastTheEndIndecesStock_.resize(numPartitions);
	nominalPrevStateTrajectoriesStock_.resize(numPartitions);
	nominalPrevInputTrajectoriesStock_.resize(numPartitions);

	/*
	 * Riccati solver variables and controller update
	 */
	SmFinalStock_  = state_matrix_array_t(numPartitions, state_matrix_t::Zero());
	SvFinalStock_  = state_vector_array_t(numPartitions, state_vector_t::Zero());
	SveFinalStock_ = state_vector_array_t(numPartitions, state_vector_t::Zero());
	sFinalStock_   = eigen_scalar_array_t(numPartitions, eigen_scalar_t::Zero());
	xFinalStock_   = state_vector_array_t(numPartitions, state_vector_t::Zero());

	SsTimeTrajectoryStock_.resize(numPartitions);
	SsNormalizedTimeTrajectoryStock_.resize(numPartitions);
	SsNormalizedEventsPastTheEndIndecesStock_.resize(numPartitions);
	sTrajectoryStock_.resize(numPartitions);
	SvTrajectoryStock_.resize(numPartitions);
	SveTrajectoryStock_.resize(numPartitions);
	SmTrajectoryStock_.resize(numPartitions);

	HmTrajectoryStock_.resize(numPartitions);
	HmInverseTrajectoryStock_.resize(numPartitions);
	GmTrajectoryStock_.resize(numPartitions);
	GvTrajectoryStock_.resize(numPartitions);

	initialControllerDesignStock_.resize(numPartitions);

	/*
	 * approximate LQ variables
	 */
	AmTrajectoryStock_.resize(numPartitions);
	BmTrajectoryStock_.resize(numPartitions);

	nc1TrajectoriesStock_.resize(numPartitions);
	EvTrajectoryStock_.resize(numPartitions);
	CmTrajectoryStock_.resize(numPartitions);
	DmTrajectoryStock_.resize(numPartitions);
	nc2TrajectoriesStock_.resize(numPartitions);
	HvTrajectoryStock_.resize(numPartitions);
	FmTrajectoryStock_.resize(numPartitions);

	nc2FinalStock_.resize(numPartitions);
	HvFinalStock_.resize(numPartitions);
	FmFinalStock_.resize(numPartitions);
	qFinalStock_.resize(numPartitions);
	QvFinalStock_.resize(numPartitions);
	QmFinalStock_.resize(numPartitions);

	ncIneqTrajectoriesStock_.resize(numPartitions);  // ncIneq: Number of inequality constraints
	hTrajectoryStock_.resize(numPartitions);
	dhdxTrajectoryStock_.resize(numPartitions);
	ddhdxdxTrajectoryStock_.resize(numPartitions);
	dhduTrajectoryStock_.resize(numPartitions);
	ddhduduTrajectoryStock_.resize(numPartitions);
	ddhdudxTrajectoryStock_.resize(numPartitions);

	qTrajectoryStock_.resize(numPartitions);
	QvTrajectoryStock_.resize(numPartitions);
	QmTrajectoryStock_.resize(numPartitions);
	RvTrajectoryStock_.resize(numPartitions);
	RmTrajectoryStock_.resize(numPartitions);
	RmInverseTrajectoryStock_.resize(numPartitions);
	PmTrajectoryStock_.resize(numPartitions);

	// Discrete-time coefficients
	AmDtimeTrajectoryStock_.resize(numPartitions);
	BmDtimeTrajectoryStock_.resize(numPartitions);

	qDtimeTrajectoryStock_.resize(numPartitions);
	QvDtimeTrajectoryStock_.resize(numPartitions);
	QmDtimeTrajectoryStock_.resize(numPartitions);
	RvDtimeTrajectoryStock_.resize(numPartitions);
	RmDtimeTrajectoryStock_.resize(numPartitions);
	PmDtimeTrajectoryStock_.resize(numPartitions);
	RmInverseDtimeTrajectoryStock_.resize(numPartitions);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
const typename ILQR_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::scalar_t&
	ILQR_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getFinalTime() const {

	return finalTime_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
const typename ILQR_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::scalar_array_t&
	ILQR_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getPartitioningTimes() const {

	return partitioningTimes_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
typename ILQR_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::logic_rules_machine_t*
	ILQR_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getLogicRulesMachinePtr() {

	return logicRulesMachinePtr_.get();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
const typename ILQR_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::logic_rules_machine_t*
	ILQR_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getLogicRulesMachinePtr() const {

	return logicRulesMachinePtr_.get();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ILQR_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::setLogicRules(const LOGIC_RULES_T& logicRules) {

	logicRulesMachinePtr_->setLogicRules(logicRules);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
const LOGIC_RULES_T* ILQR_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getLogicRulesPtr() const {

	return logicRulesMachinePtr_->getLogicRulesPtr();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
LOGIC_RULES_T* ILQR_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getLogicRulesPtr() {

	return logicRulesMachinePtr_->getLogicRulesPtr();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ILQR_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getCostDesiredTrajectoriesPtr(
		const cost_desired_trajectories_t*& costDesiredTrajectoriesPtr) const {

	costDesiredTrajectoriesPtr = &costDesiredTrajectories_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ILQR_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::setCostDesiredTrajectories(
		const cost_desired_trajectories_t& costDesiredTrajectories) {

	costDesiredTrajectoriesUpdated_ = true;
	costDesiredTrajectoriesBuffer_ = costDesiredTrajectories;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ILQR_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::setCostDesiredTrajectories(
		const scalar_array_t& desiredTimeTrajectory,
		const dynamic_vector_array_t& desiredStateTrajectory,
		const dynamic_vector_array_t& desiredInputTrajectory) {

	costDesiredTrajectoriesUpdated_ = true;
	costDesiredTrajectoriesBuffer_.desiredTimeTrajectory()  = desiredTimeTrajectory;
	costDesiredTrajectoriesBuffer_.desiredStateTrajectory() = desiredStateTrajectory;
	costDesiredTrajectoriesBuffer_.desiredInputTrajectory() = desiredInputTrajectory;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ILQR_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::swapCostDesiredTrajectories(
		cost_desired_trajectories_t& costDesiredTrajectories) {

	costDesiredTrajectoriesUpdated_ = true;
	costDesiredTrajectoriesBuffer_.swap(costDesiredTrajectories);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ILQR_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::swapCostDesiredTrajectories(
		scalar_array_t& desiredTimeTrajectory,
		dynamic_vector_array_t& desiredStateTrajectory,
		dynamic_vector_array_t& desiredInputTrajectory) {

	costDesiredTrajectoriesUpdated_ = true;
	costDesiredTrajectoriesBuffer_.desiredTimeTrajectory().swap(desiredTimeTrajectory);
	costDesiredTrajectoriesBuffer_.desiredStateTrajectory().swap(desiredStateTrajectory);
	costDesiredTrajectoriesBuffer_.desiredInputTrajectory().swap(desiredInputTrajectory);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
bool ILQR_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::costDesiredTrajectoriesUpdated() const {

	return costDesiredTrajectoriesUpdated_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ILQR_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::runInit() {

#ifdef BENCHMARK
	// Benchmarking
	BENCHMARK_nIterationsLQ_++;
	BENCHMARK_nIterationsBP_++;
	BENCHMARK_nIterationsFP_++;
	BENCHMARK_start_ = std::chrono::steady_clock::now();
#endif

	// initial controller rollout
	avgTimeStepFP_ = rolloutTrajectory(initTime_, initState_, finalTime_,
			partitioningTimes_, nominalControllersStock_,
			nominalTimeTrajectoriesStock_, nominalEventsPastTheEndIndecesStock_,
			nominalStateTrajectoriesStock_, nominalInputTrajectoriesStock_);

#ifdef BENCHMARK
	BENCHMARK_end_ = std::chrono::steady_clock::now();
	auto BENCHMARK_diff_ = BENCHMARK_end_ - BENCHMARK_start_;
	BENCHMARK_tAvgFP_ = ((1.0 - 1.0/BENCHMARK_nIterationsFP_)* BENCHMARK_tAvgFP_) +
			(1.0/BENCHMARK_nIterationsFP_)*std::chrono::duration_cast<std::chrono::milliseconds>(BENCHMARK_diff_).count();
	BENCHMARK_start_ = std::chrono::steady_clock::now();
#endif

	// linearizing the dynamics and quadratizing the cost function along nominal trajectories
	approximateOptimalControlProblem();

	// to check convergence of the main loop, we need to compute the total cost and ISEs
	bool computePerformanceIndex = settings_.displayInfo_==true || settings_.maxNumIterationsILQR_>1;
	if (computePerformanceIndex==true) {
		// calculate rollout constraint type-1 ISE
		nominalConstraint1MaxNorm_ = calculateConstraintISE(nominalTimeTrajectoriesStock_, nc1TrajectoriesStock_, EvTrajectoryStock_,
				nominalConstraint1ISE_);
		// calculate rollout constraint type-2 ISE
		if (settings_.noStateConstraints_==false)
			nominalConstraint2MaxNorm_ = calculateConstraintISE(nominalTimeTrajectoriesStock_, nc2TrajectoriesStock_, HvTrajectoryStock_,
					nominalConstraint2ISE_);
		else
			nominalConstraint2ISE_ = nominalConstraint2MaxNorm_ = 0.0;
		// calculate rollout cost
		calculateRolloutCost(nominalTimeTrajectoriesStock_, nominalEventsPastTheEndIndecesStock_,
				nominalStateTrajectoriesStock_, nominalInputTrajectoriesStock_,
				nominalTotalCost_);
	} else {
		nominalTotalCost_ = 0.0;
		nominalConstraint1ISE_ = nominalConstraint1MaxNorm_ = 0.0;
		nominalConstraint2ISE_ = nominalConstraint2MaxNorm_ = 0.0;
	}

#ifdef BENCHMARK
	BENCHMARK_end_ = std::chrono::steady_clock::now();
	BENCHMARK_diff_ = BENCHMARK_end_ - BENCHMARK_start_;
	BENCHMARK_tAvgLQ_ = ((1.0 - 1.0/BENCHMARK_nIterationsLQ_)* BENCHMARK_tAvgLQ_) +
			(1.0/BENCHMARK_nIterationsLQ_)*std::chrono::duration_cast<std::chrono::milliseconds>(BENCHMARK_diff_).count();
	BENCHMARK_start_ = std::chrono::steady_clock::now();
#endif

	// solve Riccati equations
	avgTimeStepBP_ = solveSequentialRiccatiEquations(SmHeuristics_, SvHeuristics_, sHeuristics_);
	// calculate controller
	calculateController();

#ifdef BENCHMARK
	BENCHMARK_end_ = std::chrono::steady_clock::now();
	BENCHMARK_diff_ = BENCHMARK_end_ - BENCHMARK_start_;
	BENCHMARK_tAvgBP_ = ((1.0 - 1.0/BENCHMARK_nIterationsBP_)* BENCHMARK_tAvgBP_) +
			(1.0/BENCHMARK_nIterationsBP_)*std::chrono::duration_cast<std::chrono::milliseconds>(BENCHMARK_diff_).count();
#endif

	// display
	if (settings_.displayInfo_)
		printRolloutInfo();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ILQR_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::runIteration() {

#ifdef BENCHMARK
	// Benchmarking
	BENCHMARK_nIterationsLQ_++;
	BENCHMARK_nIterationsBP_++;
	BENCHMARK_nIterationsFP_++;
	BENCHMARK_start_ = std::chrono::steady_clock::now();
#endif

	bool computeISEs = settings_.displayInfo_==true || settings_.noStateConstraints_==false;

	// finding the optimal learningRate
	maxLearningRate_ = settings_.maxLearningRateILQR_;
	lineSearch(computeISEs);

#ifdef BENCHMARK
	BENCHMARK_end_ = std::chrono::steady_clock::now();
	BENCHMARK_diff_ = BENCHMARK_end_ - BENCHMARK_start_;
	BENCHMARK_tAvgFP_ = ((1.0 - 1.0/BENCHMARK_nIterationsFP_)* BENCHMARK_tAvgFP_) +
			(1.0/BENCHMARK_nIterationsFP_)*std::chrono::duration_cast<std::chrono::milliseconds>(BENCHMARK_diff_).count();
	BENCHMARK_start_ = std::chrono::steady_clock::now();
#endif

	// linearizing the dynamics and quadratizing the cost function along nominal trajectories
	approximateOptimalControlProblem();

	// to check convergence of the main loop, we need to compute ISEs
	if (computeISEs==false) {
		// calculate constraint type-1 ISE and maximum norm
		nominalConstraint1MaxNorm_ = calculateConstraintISE(nominalTimeTrajectoriesStock_, nc1TrajectoriesStock_, EvTrajectoryStock_,
				nominalConstraint1ISE_);
		// calculates type-2 constraint ISE and maximum norm
		if (settings_.noStateConstraints_==false)
		nominalConstraint2MaxNorm_ = calculateConstraintISE(nominalTimeTrajectoriesStock_, nc2TrajectoriesStock_, HvTrajectoryStock_,
				nominalConstraint2ISE_);
		else
			nominalConstraint2ISE_ = nominalConstraint2MaxNorm_ = 0.0;
	}

#ifdef BENCHMARK
	BENCHMARK_end_ = std::chrono::steady_clock::now();
	BENCHMARK_diff_ = BENCHMARK_end_ - BENCHMARK_start_;
	BENCHMARK_tAvgLQ_ = ((1.0 - 1.0/BENCHMARK_nIterationsLQ_)* BENCHMARK_tAvgLQ_) +
			(1.0/BENCHMARK_nIterationsLQ_)*std::chrono::duration_cast<std::chrono::milliseconds>(BENCHMARK_diff_).count();
	BENCHMARK_start_ = std::chrono::steady_clock::now();
#endif

	// solve Riccati equations
	avgTimeStepBP_ = solveSequentialRiccatiEquations(SmHeuristics_, SvHeuristics_, sHeuristics_);
	// calculate controller
	calculateController();

#ifdef BENCHMARK
	BENCHMARK_end_ = std::chrono::steady_clock::now();
	BENCHMARK_diff_ = BENCHMARK_end_ - BENCHMARK_start_;
	BENCHMARK_tAvgBP_ = ((1.0 - 1.0/BENCHMARK_nIterationsBP_)* BENCHMARK_tAvgBP_) +
			(1.0/BENCHMARK_nIterationsBP_)*std::chrono::duration_cast<std::chrono::milliseconds>(BENCHMARK_diff_).count();
#endif

	// display
	if (settings_.displayInfo_)
		printRolloutInfo();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ILQR_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::runExit()  {

//	// add the deleted parts of the controller
//	for (size_t i=0; i<initActivePartition_; i++)
//		nominalControllersStock_[i].swap(deletedcontrollersStock_[i]);
//
//	if (deletedcontrollersStock_[initActivePartition_].time_.empty()==false) {
//
//		nominalControllersStock_[initActivePartition_].swap(deletedcontrollersStock_[initActivePartition_]);
//
//		for (size_t k=0; k<deletedcontrollersStock_[initActivePartition_].time_.size(); k++) {
//			nominalControllersStock_[initActivePartition_].time_.push_back(deletedcontrollersStock_[initActivePartition_].time_[k]);
//			nominalControllersStock_[initActivePartition_].uff_.push_back(deletedcontrollersStock_[initActivePartition_].uff_[k]);
//			nominalControllersStock_[initActivePartition_].k_.push_back(deletedcontrollersStock_[initActivePartition_].k_[k]);
//		}  // end of k loop
//	}
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ILQR_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::adjustController(
		const scalar_array_t& newEventTimes,
		const scalar_array_t& controllerEventTimes) {

	// adjust the nominal controllerStock using trajectory spreading
	if (nominalControllersStock_.size()>0) {
		trajectorySpreadingController_.adjustController(
				newEventTimes, controllerEventTimes,
				nominalControllersStock_);
	}
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ILQR_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::run(
		const scalar_t& initTime,
		const state_vector_t& initState,
		const scalar_t& finalTime,
		const scalar_array_t& partitioningTimes) {

	const controller_array_t noInitialController(partitioningTimes.size()-1, controller_t());

	// call the "run" method which uses the internal controllers stock (i.e. nominalControllersStock_)
	run(initTime, initState, finalTime, partitioningTimes, noInitialController);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void ILQR_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::run(
		const scalar_t& initTime,
		const state_vector_t& initState,
		const scalar_t& finalTime,
		const scalar_array_t& partitioningTimes,
		const controller_array_t& controllersStock) {

	// infeasible learning rate adjustment scheme
	if (settings_.maxLearningRateILQR_ < settings_.minLearningRateILQR_-OCS2NumericTraits<scalar_t>::limit_epsilon())
		throw std::runtime_error("The maximum learning rate is smaller than the minimum learning rate.");

	if (settings_.displayInfo_) {
		std::cerr << "++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
		std::cerr << "++++++++++++++++ ILQR solver is initialized ++++++++++++++++" << std::endl;
		std::cerr << "++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
	}

	// update numPartitions_ if it has been changed
	if (numPartitions_+1 != partitioningTimes.size()) {
		numPartitions_  = partitioningTimes.size()-1;
		partitioningTimes_ = partitioningTimes;
		setupOptimizer(numPartitions_);
	}

	// update partitioningTimes_
	partitioningTimes_ = partitioningTimes;

	// Use the input controller if it is not empty otherwise use the internal controller (nominalControllersStock_).
	// In the later case 2 scenarios are possible: either the internal controller is already set (such as the MPC case
	// where the warm starting option is set true) or the internal controller is empty in which instead of performing
	// a rollout the operating trajectories will be used.
	if (controllersStock.empty()==false) {
		nominalControllersStock_ = controllersStock;
		if (controllersStock.size() != numPartitions_)
			throw std::runtime_error("controllersStock has less controllers than the number of partitions.");
	} else {
		if (nominalControllersStock_.size() != numPartitions_)
			throw std::runtime_error("The internal controller is not compatible with the number of partitions.");
	}

	// set desired trajectories of cost if it is updated
	if (costDesiredTrajectoriesUpdated_ == true) {
		costDesiredTrajectoriesUpdated_ = false;
		costDesiredTrajectories_.swap(costDesiredTrajectoriesBuffer_);
	}

	// update the LOGIC_RULES in the beginning of the run routine
	bool logicRulesModified = logicRulesMachinePtr_->updateLogicRules(partitioningTimes_);

	// display
	if (settings_.displayInfo_) {
		std::cerr << std::endl << "Rewind Counter: " << rewindCounter_ << std::endl;
		std::cerr << "ILQR solver starts from initial time " << initTime << " to final time " << finalTime << ".";
		logicRulesMachinePtr_->display();
		std::cerr << std::endl;
	}

	iteration_ = 0;
	initState_ = initState;
	initTime_  = initTime;
	finalTime_ = finalTime;

	iterationCost_.clear();
	iterationISE1_.clear();
	iterationISE2_.clear();

	// finding the initial active partition index and truncating the controller
	truncateConterller(partitioningTimes_, initTime_,
			nominalControllersStock_, initActivePartition_, deletedcontrollersStock_);

	// the final active partition index.
	finalActivePartition_ = BASE::findActivePartitionIndex(partitioningTimes_, finalTime_);

	// check if after the truncation the internal controller is empty
	bool isInitInternalControllerEmpty = false;
	for (const controller_t& controller: nominalControllersStock_) {
		isInitInternalControllerEmpty = isInitInternalControllerEmpty || controller.empty();
	}

	// display
	if (settings_.displayInfo_)
		std::cerr << "\n#### Iteration " << iteration_ << " (Dynamics might have been violated)" << std::endl;

	// if a controller is not set for a partition
	for (size_t i=0; i<numPartitions_; i++)
		initialControllerDesignStock_[i] = nominalControllersStock_[i].empty() ? true : false;

	// run ILQR initializer and update the member variables
	runInit();

	// after iteration zero always allow feedforward policy update
	for (size_t i=0; i<numPartitions_; i++)
		initialControllerDesignStock_[i] = false;

	iterationCost_.push_back( (Eigen::VectorXd(1) << nominalTotalCost_).finished() );
	iterationISE1_.push_back( (Eigen::VectorXd(1) << nominalConstraint1ISE_).finished() );
	iterationISE2_.push_back( (Eigen::VectorXd(1) << nominalConstraint2ISE_).finished() );

	// convergence conditions variables
	scalar_t relCost;
	scalar_t relConstraint1ISE;
	bool isLearningRateStarZero = false;
	bool isCostFunctionConverged = false;
	bool isConstraint1Satisfied  = false;
	bool isOptimizationConverged = false;

	// ILQR main loop
	while (iteration_+1<settings_.maxNumIterationsILQR_ && isOptimizationConverged==false)  {

		// increment iteration counter
		iteration_++;

		scalar_t costCashed = nominalTotalCost_;
		scalar_t constraint1ISECashed = nominalConstraint1ISE_;

		// display
		if (settings_.displayInfo_)  std::cerr << "\n#### Iteration " << iteration_ << std::endl;

		// run the an iteration of the ILQR algorithm and update the member variables
		runIteration();

		iterationCost_.push_back( (Eigen::VectorXd(1) << nominalTotalCost_).finished() );
		iterationISE1_.push_back( (Eigen::VectorXd(1) << nominalConstraint1ISE_).finished() );
		iterationISE2_.push_back( (Eigen::VectorXd(1) << nominalConstraint2ISE_).finished() );

		// loop break variables
		relCost = std::abs(nominalTotalCost_-costCashed);
		relConstraint1ISE = std::abs(nominalConstraint1ISE_-constraint1ISECashed);
		isConstraint1Satisfied  = nominalConstraint1ISE_<=settings_.minAbsConstraint1ISE_ || relConstraint1ISE<=settings_.minRelConstraint1ISE_;
		isLearningRateStarZero  = learningRateStar_==0 && !isInitInternalControllerEmpty;
		isCostFunctionConverged = relCost<=settings_.minRelCostILQR_ || isLearningRateStarZero;
		isOptimizationConverged = isCostFunctionConverged==true && isConstraint1Satisfied==true;
		isInitInternalControllerEmpty = false;

	}  // end of while loop

	if (settings_.displayInfo_)  std::cerr << "\n#### Final rollout" << std::endl;

#ifdef BENCHMARK
	BENCHMARK_nIterationsFP_++;
	BENCHMARK_start_ = std::chrono::steady_clock::now();
#endif

	bool computeISEs = settings_.noStateConstraints_==false ||
			settings_.displayInfo_==true || settings_.displayShortSummary_==true;

	// finding the final optimal learningRate and getting the optimal trajectories and controller
	maxLearningRate_ = settings_.maxLearningRateILQR_;
	lineSearch(computeISEs);

#ifdef BENCHMARK
	BENCHMARK_end_ = std::chrono::steady_clock::now();
	BENCHMARK_diff_ = BENCHMARK_end_ - BENCHMARK_start_;
	BENCHMARK_tAvgFP_ = ((1.0 - 1.0/BENCHMARK_nIterationsFP_)* BENCHMARK_tAvgFP_) +
			(1.0/BENCHMARK_nIterationsFP_)*std::chrono::duration_cast<std::chrono::milliseconds>(BENCHMARK_diff_).count();
#endif

	/*
	 * adds the deleted controller parts
	 */
	runExit();

	// display
	if (settings_.displayInfo_  || settings_.displayShortSummary_)  {

		std::cerr << "\n++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
		std::cerr <<   "+++++++++++++++ ILQR solver is terminated ++++++++++++++++" << std::endl;
		std::cerr <<   "++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
		std::cerr << "Number of Iterations:      " <<  iteration_+1 << " out of " << settings_.maxNumIterationsILQR_ << std::endl;

		printRolloutInfo();

		if (isOptimizationConverged==true) {
			if (isLearningRateStarZero==true)
				std::cerr << "ILQR successfully terminates as learningRate reduced to zero." << std::endl;
			else
				std::cerr << "ILQR successfully terminates as cost relative change (relCost=" << relCost <<") reached to the minimum value." << std::endl;

			if (nominalConstraint1ISE_<=settings_.minAbsConstraint1ISE_)
				std::cerr << "Type-1 constraint absolute ISE (absConstraint1ISE=" << nominalConstraint1ISE_ << ") reached to the minimum value." << std::endl;
			else
				std::cerr << "Type-1 constraint relative ISE (relConstraint1ISE=" << relConstraint1ISE << ") reached to the minimum value." << std::endl;
		} else {
			std::cerr << "Maximum number of iterations has reached." << std::endl;
		}
		std::cerr << std::endl;
	}

}

}  // ocs2 namespace

