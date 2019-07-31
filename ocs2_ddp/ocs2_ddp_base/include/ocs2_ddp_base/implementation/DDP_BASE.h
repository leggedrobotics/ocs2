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

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
DDP_BASE<STATE_DIM, INPUT_DIM>::DDP_BASE(const controlled_system_base_t* systemDynamicsPtr, const derivatives_base_t* systemDerivativesPtr,
                                         const constraint_base_t* systemConstraintsPtr, const cost_function_base_t* costFunctionPtr,
                                         const operating_trajectories_base_t* operatingTrajectoriesPtr, const DDP_Settings& ddpSettings,
                                         const Rollout_Settings& rolloutSettings, const cost_function_base_t* heuristicsFunctionPtr,
                                         const char* algorithmName, std::shared_ptr<HybridLogicRules> logicRulesPtr)
    : BASE(std::move(logicRulesPtr)),
      ddpSettings_(ddpSettings),
      rolloutSettings_(rolloutSettings),
      algorithmName_(algorithmName),
      costDesiredTrajectories_(),
      costDesiredTrajectoriesBuffer_(),
      costDesiredTrajectoriesUpdated_(false),
      rewindCounter_(0),
      iteration_(0) {
  // Dynamics, Constraints, derivatives, and cost
  linearQuadraticApproximatorPtrStock_.clear();
  linearQuadraticApproximatorPtrStock_.reserve(ddpSettings_.nThreads_);
  heuristicsFunctionsPtrStock_.clear();
  heuristicsFunctionsPtrStock_.reserve(ddpSettings_.nThreads_);
  penaltyPtrStock_.clear();
  penaltyPtrStock_.reserve(ddpSettings_.nThreads_);

  dynamicsForwardRolloutPtrStock_.resize(ddpSettings_.nThreads_);
  operatingTrajectoriesRolloutPtrStock_.resize(ddpSettings_.nThreads_);

  // initialize all subsystems, etc.
  for (size_t i = 0; i < ddpSettings_.nThreads_; i++) {
    // initialize rollout
    dynamicsForwardRolloutPtrStock_[i].reset(new time_triggered_rollout_t(*systemDynamicsPtr, rolloutSettings_, algorithmName_.c_str()));

    // initialize operating points
    operatingTrajectoriesRolloutPtrStock_[i].reset(
        new operating_trajectorie_rollout_t(*operatingTrajectoriesPtr, rolloutSettings_, algorithmName_.c_str()));

    // initialize LQ approximator
    linearQuadraticApproximatorPtrStock_.emplace_back(
        new linear_quadratic_approximator_t(*systemDerivativesPtr, *systemConstraintsPtr, *costFunctionPtr, algorithmName_.c_str(),
                                            ddpSettings_.checkNumericalStability_, ddpSettings_.useMakePSD_));

    // initialize operating trajectories
    operatingTrajectoriesPtrStock_.emplace_back(operatingTrajectoriesPtr->clone());

    // initialize heuristics functions
    if (heuristicsFunctionPtr != nullptr) {
      heuristicsFunctionsPtrStock_.emplace_back(heuristicsFunctionPtr->clone());
    } else {  // use the cost function if no heuristics function is defined
      heuristicsFunctionsPtrStock_.emplace_back(costFunctionPtr->clone());
    }

    // initialize penalty functions
    penaltyPtrStock_.emplace_back(std::shared_ptr<PenaltyBase<STATE_DIM, INPUT_DIM>>(
        new RelaxedBarrierPenalty<STATE_DIM, INPUT_DIM>(ddpSettings_.inequalityConstraintMu_, ddpSettings_.inequalityConstraintDelta_)));

  }  // end of i loop

  nominalStateFunc_.resize(ddpSettings_.nThreads_);
  nominalInputFunc_.resize(ddpSettings_.nThreads_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
DDP_BASE<STATE_DIM, INPUT_DIM>::~DDP_BASE() {
#ifdef BENCHMARK
  auto BENCHMARK_total = BENCHMARK_tAvgFP_ + BENCHMARK_tAvgBP_ + BENCHMARK_tAvgLQ_;
  if (BENCHMARK_total > 0 && (ddpSettings_.displayInfo_ || ddpSettings_.displayShortSummary_)) {
    std::cerr << std::endl << "#####################################################" << std::endl;
    std::cerr << "Benchmarking over " << BENCHMARK_nIterationsBP_ << " samples." << std::endl;
    std::cerr << "Average time for Forward Pass:      " << BENCHMARK_tAvgFP_ / 1000.0 << " [ms] \t("
              << BENCHMARK_tAvgFP_ / BENCHMARK_total * 100 << "%)" << std::endl;
    std::cerr << "Average time for Backward Pass:     " << BENCHMARK_tAvgBP_ / 1000.0 << " [ms] \t("
              << BENCHMARK_tAvgBP_ / BENCHMARK_total * 100 << "%)" << std::endl;
    std::cerr << "Average time for LQ Approximation:  " << BENCHMARK_tAvgLQ_ / 1000.0 << " [ms] \t("
              << BENCHMARK_tAvgLQ_ / BENCHMARK_total * 100 << "%)" << std::endl;
  }
#endif
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
void DDP_BASE<STATE_DIM, INPUT_DIM>::reset() {
  iteration_ = 0;
  rewindCounter_ = 0;

  learningRateStar_ = 1.0;
  maxLearningRate_ = 1.0;
  constraintStepSize_ = 1.0;

  blockwiseMovingHorizon_ = false;
  useParallelRiccatiSolverFromInitItr_ = false;

  costDesiredTrajectories_.clear();
  {
    std::lock_guard<std::mutex> lock(costDesiredTrajectoriesBufferMutex_);
    costDesiredTrajectoriesBuffer_.clear();
    costDesiredTrajectoriesUpdated_ = false;
  }

  for (size_t i = 0; i < numPartitions_; i++) {
    // very important :)
    nominalControllersStock_[i].clear();

    // for Riccati equation parallel computation
    SmFinalStock_[i] = state_matrix_t::Zero();
    SvFinalStock_[i] = state_vector_t::Zero();
    SveFinalStock_[i] = state_vector_t::Zero();
    sFinalStock_[i] = eigen_scalar_t::Zero();
    xFinalStock_[i] = state_vector_t::Zero();
  }  // end of i loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
typename DDP_BASE<STATE_DIM, INPUT_DIM>::scalar_t DDP_BASE<STATE_DIM, INPUT_DIM>::rolloutTrajectory(
    const scalar_t& initTime, const state_vector_t& initState, const scalar_t& finalTime, const scalar_array_t& partitioningTimes,
    linear_controller_array_t& controllersStock, scalar_array2_t& timeTrajectoriesStock, size_array2_t& eventsPastTheEndIndecesStock,
    state_vector_array2_t& stateTrajectoriesStock, input_vector_array2_t& inputTrajectoriesStock, size_t threadId /*= 0*/) {
  size_t numPartitions = partitioningTimes.size() - 1;

  if (controllersStock.size() != numPartitions) {
    throw std::runtime_error("controllersStock has less controllers then the number of subsystems");
  }

  timeTrajectoriesStock.resize(numPartitions);
  eventsPastTheEndIndecesStock.resize(numPartitions);
  stateTrajectoriesStock.resize(numPartitions);
  inputTrajectoriesStock.resize(numPartitions);

  // finding the active subsystem index at initTime
  size_t initActivePartition = lookup::findBoundedActiveIntervalInTimeArray(partitioningTimes, initTime);
  // finding the active subsystem index at initTime
  size_t finalActivePartition = lookup::findBoundedActiveIntervalInTimeArray(partitioningTimes, finalTime);

  scalar_t t0 = initTime;
  state_vector_t x0 = initState;
  scalar_t tf;
  size_t numSteps = 0;
  for (size_t i = 0; i < numPartitions; i++) {
    // for subsystems before the initial time
    if (i < initActivePartition || i > finalActivePartition) {
      timeTrajectoriesStock[i].clear();
      eventsPastTheEndIndecesStock[i].clear();
      stateTrajectoriesStock[i].clear();
      inputTrajectoriesStock[i].clear();
      continue;
    }

    // final time
    tf = (i != finalActivePartition) ? partitioningTimes[i + 1] : finalTime;

    // if blockwiseMovingHorizon_ is not set, use the previous partition's controller for
    // the first rollout of the partition. However for the very first run of the algorithm,
    // it will still use operating trajectories if an initial controller is not provided.
    linear_controller_t* controllerPtrTemp = &controllersStock[i];
    if (blockwiseMovingHorizon_ == false) {
      if (controllerPtrTemp->empty() && i > 0 && !controllersStock[i - 1].empty()) {
        controllerPtrTemp = &controllersStock[i - 1];
      }
    }

    // call rollout worker for the partition 'i' on the thread 'threadId'
    state_vector_t x0Temp;
    if (!controllerPtrTemp->empty()) {
      x0Temp = dynamicsForwardRolloutPtrStock_[threadId]->run(i, t0, x0, tf, controllerPtrTemp, *BASE::getLogicRulesMachinePtr(),
                                                              timeTrajectoriesStock[i], eventsPastTheEndIndecesStock[i],
                                                              stateTrajectoriesStock[i], inputTrajectoriesStock[i]);

    } else {
      x0Temp = operatingTrajectoriesRolloutPtrStock_[threadId]->run(i, t0, x0, tf, nullptr, *BASE::getLogicRulesMachinePtr(),
                                                                    timeTrajectoriesStock[i], eventsPastTheEndIndecesStock[i],
                                                                    stateTrajectoriesStock[i], inputTrajectoriesStock[i]);
    }
    // if there was an event time at the end of the previous partition
    if (initActivePartition < i && eventsPastTheEndIndecesStock[i - 1].size() > 0) {
      if (eventsPastTheEndIndecesStock[i - 1].back() == stateTrajectoriesStock[i - 1].size()) {
        timeTrajectoriesStock[i - 1].push_back(t0);
        stateTrajectoriesStock[i - 1].push_back(x0);
        inputTrajectoriesStock[i - 1].push_back(inputTrajectoriesStock[i].front());
      }
    }

    // reset the initial time and state
    t0 = timeTrajectoriesStock[i].back();
    x0.swap(x0Temp);

    // total number of steps
    numSteps += timeTrajectoriesStock[i].size();

  }  // end of i loop

  // if there is an active event at the finalTime, we remove it.
  if (eventsPastTheEndIndecesStock[finalActivePartition].size() > 0) {
    if (eventsPastTheEndIndecesStock[finalActivePartition].back() == stateTrajectoriesStock[finalActivePartition].size()) {
      eventsPastTheEndIndecesStock[finalActivePartition].pop_back();
    }
  }

  if (x0 != x0) {
    throw std::runtime_error("System became unstable during the rollout.");
  }

  // debug print
  if (ddpSettings_.debugPrintRollout_) {
    for (size_t i = 0; i < numPartitions; i++) {
      rollout_base_t::display(i, timeTrajectoriesStock[i], eventsPastTheEndIndecesStock[i], stateTrajectoriesStock[i],
                              inputTrajectoriesStock[i]);
    }
  }

  // average time step
  return (finalTime - initTime) / (scalar_t)numSteps;
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
void DDP_BASE<STATE_DIM, INPUT_DIM>::rolloutFinalState(const scalar_t& initTime, const state_vector_t& initState, const scalar_t& finalTime,
                                                       const scalar_array_t& partitioningTimes,
                                                       const linear_controller_array_t& controllersStock, state_vector_t& finalState,
                                                       input_vector_t& finalInput, size_t& finalActivePartition, size_t threadId /*= 0*/) {
  size_t numPartitions = partitioningTimes.size() - 1;

  if (controllersStock.size() != numPartitions) {
    throw std::runtime_error("controllersStock has less controllers then the number of subsystems");
  }

  scalar_array_t timeTrajectory;
  size_array_t eventsPastTheEndIndeces;
  state_vector_array_t stateTrajectory;
  input_vector_t inputTrajectory;

  // finding the active subsystem index at initTime and final time
  size_t initActivePartition = lookup::findBoundedActiveIntervalInTimeArray(partitioningTimes, initTime);
  finalActivePartition = lookup::findBoundedActiveIntervalInTimeArray(partitioningTimes, finalTime);

  scalar_t t0 = initTime, tf;
  state_vector_t x0 = initState;
  for (size_t i = initActivePartition; i <= finalActivePartition; i++) {
    timeTrajectory.clear();
    stateTrajectory.clear();
    inputTrajectory.clear();

    // final time
    tf = (i != finalActivePartition) ? partitioningTimes[i + 1] : finalTime;

    // if blockwiseMovingHorizon_ is not set, use the previous partition's controller for
    // the first rollout of the partition. However for the very first run of the algorithm,
    // it will still use operating trajectories if an initial controller is not provided.
    const controller_t* controllerPtrTemp = &controllersStock[i];
    if (blockwiseMovingHorizon_) {
      if (controllerPtrTemp->empty() && i > 0 && !controllersStock[i - 1].empty()) {
        controllerPtrTemp = &controllersStock[i - 1];
      }
    }

    // call rollout worker for the partition 'i' on the thread 'threadId'
    if (!controllerPtrTemp->empty()) {
      x0 = dynamicsForwardRolloutPtrStock_[threadId]->run(i, t0, x0, tf, *controllerPtrTemp, *BASE::getLogicRulesMachinePtr(),
                                                          timeTrajectory, eventsPastTheEndIndeces, stateTrajectory, inputTrajectory);

    } else {
      x0 = operatingTrajectoriesRolloutPtrStock_[threadId]->run(i, t0, x0, tf, *controllerPtrTemp, *BASE::getLogicRulesMachinePtr(),
                                                                timeTrajectory, eventsPastTheEndIndeces, stateTrajectory, inputTrajectory);
    }

    // reset the initial time
    t0 = timeTrajectory.back();
  }

  if (x0 != x0) {
    throw std::runtime_error("System became unstable during the rollout.");
  }

  // final state and input
  finalState = stateTrajectory.back();
  finalInput = inputTrajectory.back();
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
void DDP_BASE<STATE_DIM, INPUT_DIM>::calculateConstraintsWorker(
    size_t workerIndex, const size_t& partitionIndex, const scalar_array_t& timeTrajectory, const size_array_t& eventsPastTheEndIndeces,
    const state_vector_array_t& stateTrajectory, const input_vector_array_t& inputTrajectory, size_array_t& nc1Trajectory,
    constraint1_vector_array_t& EvTrajectory, size_array_t& nc2Trajectory, constraint2_vector_array_t& HvTrajectory,
    size_array_t& ncIneqTrajectory, scalar_array2_t& hTrajectory, size_array_t& nc2Finals, constraint2_vector_array_t& HvFinals) {
  constraint_base_t& systemConstraints = linearQuadraticApproximatorPtrStock_[workerIndex]->systemConstraints();

  size_t N = timeTrajectory.size();

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
  for (size_t k = 0; k < N; k++) {
    // set data
    systemConstraints.setCurrentStateAndControl(timeTrajectory[k], stateTrajectory[k], inputTrajectory[k]);

    // constraint 1 type
    nc1Trajectory[k] = systemConstraints.numStateInputConstraint(timeTrajectory[k]);
    systemConstraints.getConstraint1(EvTrajectory[k]);
    if (nc1Trajectory[k] > INPUT_DIM) {
      throw std::runtime_error("Number of active type-1 constraints should be less-equal to the number of input dimension.");
    }

    // constraint type 2
    nc2Trajectory[k] = systemConstraints.numStateOnlyConstraint(timeTrajectory[k]);
    systemConstraints.getConstraint2(HvTrajectory[k]);
    if (nc2Trajectory[k] > INPUT_DIM) {
      throw std::runtime_error("Number of active type-2 constraints should be less-equal to the number of input dimension.");
    }

    // inequality constraints
    ncIneqTrajectory[k] = systemConstraints.numInequalityConstraint(timeTrajectory[k]);
    if (ncIneqTrajectory[k] > 0) {
      systemConstraints.getInequalityConstraint(hTrajectory[k]);
    }

    // switching time state-constraints
    if (eventsPastTheEndItr != eventsPastTheEndIndeces.end() && k + 1 == *eventsPastTheEndItr) {
      size_t nc2Final;
      constraint2_vector_t HvFinal;
      nc2Final = systemConstraints.numStateOnlyFinalConstraint(timeTrajectory[k]);
      systemConstraints.getFinalConstraint2(HvFinal);
      if (nc2Final > INPUT_DIM) {
        throw std::runtime_error(
            "Number of active type-2 constraints at final time should be less-equal to the number of input dimension.");
      }

      nc2Finals.push_back(std::move(nc2Final));
      HvFinals.push_back(std::move(HvFinal));
      eventsPastTheEndItr++;
    }

  }  // end of k loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
void DDP_BASE<STATE_DIM, INPUT_DIM>::calculateRolloutConstraints(
    const scalar_array2_t& timeTrajectoriesStock, const size_array2_t& eventsPastTheEndIndecesStock,
    const state_vector_array2_t& stateTrajectoriesStock, const input_vector_array2_t& inputTrajectoriesStock,
    size_array2_t& nc1TrajectoriesStock, constraint1_vector_array2_t& EvTrajectoryStock, size_array2_t& nc2TrajectoriesStock,
    constraint2_vector_array2_t& HvTrajectoryStock, size_array2_t& ncIneqTrajectoriesStock, scalar_array3_t& hTrajectoryStock,
    size_array2_t& nc2FinalStock, constraint2_vector_array2_t& HvFinalStock, size_t threadId /*= 0*/) {
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

  for (size_t i = 0; i < numPartitions_; i++) {
    calculateConstraintsWorker(threadId, i, timeTrajectoriesStock[i], eventsPastTheEndIndecesStock[i], stateTrajectoriesStock[i],
                               inputTrajectoriesStock[i], nc1TrajectoriesStock[i], EvTrajectoryStock[i], nc2TrajectoriesStock[i],
                               HvTrajectoryStock[i], ncIneqTrajectoriesStock[i], hTrajectoryStock[i], nc2FinalStock[i], HvFinalStock[i]);
  }  // end of i loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
void DDP_BASE<STATE_DIM, INPUT_DIM>::calculateCostWorker(size_t workerIndex, const size_t& partitionIndex,
                                                         const scalar_array_t& timeTrajectory, const size_array_t& eventsPastTheEndIndeces,
                                                         const state_vector_array_t& stateTrajectory,
                                                         const input_vector_array_t& inputTrajectory, scalar_t& totalCost) {
  cost_function_base_t& costFunction = linearQuadraticApproximatorPtrStock_[workerIndex]->costFunction();

  // set desired trajectories
  costFunction.setCostDesiredTrajectories(costDesiredTrajectories_);

  totalCost = 0.0;
  auto eventsPastTheEndItr = eventsPastTheEndIndeces.begin();

  // integrates the intermediate cost using the trapezoidal approximation method
  scalar_t prevIntermediateCost = 0.0;
  scalar_t currIntermediateCost = 0.0;
  for (size_t k = 0; k < timeTrajectory.size(); k++) {
    if (k > 0) {
      prevIntermediateCost = currIntermediateCost;
    }

    // feed state and control to cost function
    costFunction.setCurrentStateAndControl(timeTrajectory[k], stateTrajectory[k], inputTrajectory[k]);
    // getIntermediateCost intermediate cost for next time step
    costFunction.getIntermediateCost(currIntermediateCost);

    if (k > 0) {
      totalCost += 0.5 * (prevIntermediateCost + currIntermediateCost) * (timeTrajectory[k] - timeTrajectory[k - 1]);
    }

    // terminal cost at switching times
    if (eventsPastTheEndItr != eventsPastTheEndIndeces.end() && k + 1 == *eventsPastTheEndItr) {
      scalar_t finalCost;
      costFunction.getTerminalCost(finalCost);
      totalCost += finalCost;

      eventsPastTheEndItr++;
    }

  }  // end of k loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
void DDP_BASE<STATE_DIM, INPUT_DIM>::calculateRolloutCost(const scalar_array2_t& timeTrajectoriesStock,
                                                          const size_array2_t& eventsPastTheEndIndecesStock,
                                                          const state_vector_array2_t& stateTrajectoriesStock,
                                                          const input_vector_array2_t& inputTrajectoriesStock, scalar_t& totalCost,
                                                          size_t threadId /*= 0*/) {
  totalCost = 0.0;

  for (size_t i = 0; i < numPartitions_; i++) {
    scalar_t cost;
    calculateCostWorker(threadId, i, timeTrajectoriesStock[i], eventsPastTheEndIndecesStock[i], stateTrajectoriesStock[i],
                        inputTrajectoriesStock[i], cost);
    totalCost += cost;
  }  // end of i loop

  // calculate the Heuristics function at the final time
  // set desired trajectories
  heuristicsFunctionsPtrStock_[threadId]->setCostDesiredTrajectories(costDesiredTrajectories_);
  // set state-input
  heuristicsFunctionsPtrStock_[threadId]->setCurrentStateAndControl(timeTrajectoriesStock[finalActivePartition_].back(),
                                                                    stateTrajectoriesStock[finalActivePartition_].back(),
                                                                    inputTrajectoriesStock[finalActivePartition_].back());
  // compute
  scalar_t sHeuristics;
  heuristicsFunctionsPtrStock_[threadId]->getTerminalCost(sHeuristics);
  totalCost += sHeuristics;
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
void DDP_BASE<STATE_DIM, INPUT_DIM>::calculateRolloutCost(
    const scalar_array2_t& timeTrajectoriesStock, const size_array2_t& eventsPastTheEndIndecesStock,
    const state_vector_array2_t& stateTrajectoriesStock, const input_vector_array2_t& inputTrajectoriesStock,
    const scalar_t& constraint2ISE, const scalar_t& inequalityConstraintPenalty, const size_array2_t& nc2FinalStock,
    const constraint2_vector_array2_t& HvFinalStock, scalar_t& totalCost, size_t threadId /*= 0*/) {
  calculateRolloutCost(timeTrajectoriesStock, eventsPastTheEndIndecesStock, stateTrajectoriesStock, inputTrajectoriesStock, totalCost,
                       threadId);

  const scalar_t stateConstraintPenalty =
      ddpSettings_.stateConstraintPenaltyCoeff_ * std::pow(ddpSettings_.stateConstraintPenaltyBase_, iteration_);

  // ISE of type-2 constraint
  totalCost += 0.5 * stateConstraintPenalty * constraint2ISE;

  // Inequality constraints
  totalCost += inequalityConstraintPenalty;

  // final constraint type 2
  if (!ddpSettings_.noStateConstraints_) {
    for (size_t i = 0; i < numPartitions_; i++) {
      for (size_t k = 0; k < nc2FinalStock[i].size(); k++) {
        const size_t& nc2Final = nc2FinalStock[i][k];
        totalCost += 0.5 * stateConstraintPenalty * HvFinalStock[i][k].head(nc2Final).squaredNorm();
      }  // end of k loop
    }    // end of i loop
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
void DDP_BASE<STATE_DIM, INPUT_DIM>::approximateOptimalControlProblem() {
  for (size_t i = 0; i < numPartitions_; i++) {
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
      for (size_t j = 0; j < ddpSettings_.nThreads_; j++) {
        // set desired trajectories
        linearQuadraticApproximatorPtrStock_[j]->costFunction().setCostDesiredTrajectories(costDesiredTrajectories_);
      }  // end of j loop

      // perform the approximateSubsystemLQ for partition i
      approximatePartitionLQ(i);
    }

  }  // end of i loop

  // calculate the Heuristics function at the final time
  heuristicsFunctionsPtrStock_[0]->setCostDesiredTrajectories(costDesiredTrajectories_);
  heuristicsFunctionsPtrStock_[0]->setCurrentStateAndControl(nominalTimeTrajectoriesStock_[finalActivePartition_].back(),
                                                             nominalStateTrajectoriesStock_[finalActivePartition_].back(),
                                                             nominalInputTrajectoriesStock_[finalActivePartition_].back());
  heuristicsFunctionsPtrStock_[0]->getTerminalCost(sHeuristics_(0));
  heuristicsFunctionsPtrStock_[0]->getTerminalCostDerivativeState(SvHeuristics_);
  heuristicsFunctionsPtrStock_[0]->getTerminalCostSecondDerivativeState(SmHeuristics_);
  if (ddpSettings_.useMakePSD_) {
    LinearAlgebra::makePSD(SmHeuristics_);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
void DDP_BASE<STATE_DIM, INPUT_DIM>::approximateUnconstrainedLQWorker(size_t workerIndex, const size_t& i, const size_t& k) {
  linearQuadraticApproximatorPtrStock_[workerIndex]->approximateUnconstrainedLQProblem(
      nominalTimeTrajectoriesStock_[i][k], nominalStateTrajectoriesStock_[i][k], nominalInputTrajectoriesStock_[i][k],
      AmTrajectoryStock_[i][k], BmTrajectoryStock_[i][k], nc1TrajectoriesStock_[i][k], EvTrajectoryStock_[i][k], CmTrajectoryStock_[i][k],
      DmTrajectoryStock_[i][k], nc2TrajectoriesStock_[i][k], HvTrajectoryStock_[i][k], FmTrajectoryStock_[i][k],
      ncIneqTrajectoriesStock_[i][k], hTrajectoryStock_[i][k], dhdxTrajectoryStock_[i][k], dhduTrajectoryStock_[i][k],
      ddhdxdxTrajectoryStock_[i][k], ddhduduTrajectoryStock_[i][k], ddhdudxTrajectoryStock_[i][k], qTrajectoryStock_[i][k],
      QvTrajectoryStock_[i][k], QmTrajectoryStock_[i][k], RvTrajectoryStock_[i][k], RmTrajectoryStock_[i][k], PmTrajectoryStock_[i][k]);

  // making sure that constrained Qm is PSD
  if (ddpSettings_.useMakePSD_) {
    LinearAlgebra::makePSD(QmTrajectoryStock_[i][k]);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
void DDP_BASE<STATE_DIM, INPUT_DIM>::approximateEventsLQWorker(size_t workerIndex, const size_t& i, const size_t& k,
                                                               const scalar_t& stateConstraintPenalty) {
  // if a switch took place calculate switch related variables
  size_t NE = nominalEventsPastTheEndIndecesStock_[i].size();
  for (size_t ke = 0; ke < NE; ke++) {
    if (nominalEventsPastTheEndIndecesStock_[i][ke] == k + 1) {
      linearQuadraticApproximatorPtrStock_[workerIndex]->approximateUnconstrainedLQProblemAtEventTime(
          nominalTimeTrajectoriesStock_[i][k], nominalStateTrajectoriesStock_[i][k], nominalInputTrajectoriesStock_[i][k]);

      // Final state-only equality constraint
      nc2FinalStock_[i][ke] = linearQuadraticApproximatorPtrStock_[workerIndex]->ncFinalEqStateOnly_;
      HvFinalStock_[i][ke].swap(linearQuadraticApproximatorPtrStock_[workerIndex]->HvFinal_);
      FmFinalStock_[i][ke].swap(linearQuadraticApproximatorPtrStock_[workerIndex]->FmFinal_);

      // Final cost
      qFinalStock_[i][ke].swap(linearQuadraticApproximatorPtrStock_[workerIndex]->qFinal_);
      QvFinalStock_[i][ke].swap(linearQuadraticApproximatorPtrStock_[workerIndex]->QvFinal_);
      QmFinalStock_[i][ke].swap(linearQuadraticApproximatorPtrStock_[workerIndex]->QmFinal_);

      /*
       * Modify the unconstrained LQ coefficients to constrained ones
       */
      // final constraint type 2 coefficients
      const size_t& nc2 = nc2FinalStock_[i][ke];
      if (nc2 > 0) {
        qFinalStock_[i][ke] += 0.5 * stateConstraintPenalty * HvFinalStock_[i][ke].head(nc2).transpose() * HvFinalStock_[i][ke].head(nc2);
        QvFinalStock_[i][ke] += stateConstraintPenalty * FmFinalStock_[i][ke].topRows(nc2).transpose() * HvFinalStock_[i][ke].head(nc2);
        QmFinalStock_[i][ke] += stateConstraintPenalty * FmFinalStock_[i][ke].topRows(nc2).transpose() * FmFinalStock_[i][ke].topRows(nc2);
      }

      // making sure that Qm remains PSD
      if (ddpSettings_.useMakePSD_) {
        LinearAlgebra::makePSD(QmFinalStock_[i][ke]);
      }

      break;
    }
  }  // end of ke loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
void DDP_BASE<STATE_DIM, INPUT_DIM>::lineSearchBase(bool computeISEs) {
  // display
  if (ddpSettings_.displayInfo_) {
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
  avgTimeStepFP_ =
      rolloutTrajectory(initTime_, initState_, finalTime_, partitioningTimes_, nominalControllersStock_, nominalTimeTrajectoriesStock_,
                        nominalEventsPastTheEndIndecesStock_, nominalStateTrajectoriesStock_, nominalInputTrajectoriesStock_);

  if (computeISEs) {
    // calculate constraint
    calculateRolloutConstraints(nominalTimeTrajectoriesStock_, nominalEventsPastTheEndIndecesStock_, nominalStateTrajectoriesStock_,
                                nominalInputTrajectoriesStock_, nc1TrajectoriesStock_, EvTrajectoryStock_, nc2TrajectoriesStock_,
                                HvTrajectoryStock_, ncIneqTrajectoriesStock_, hTrajectoryStock_, nc2FinalStock_, HvFinalStock_);
    // calculate constraint type-1 ISE and maximum norm
    nominalConstraint1MaxNorm_ =
        calculateConstraintISE(nominalTimeTrajectoriesStock_, nc1TrajectoriesStock_, EvTrajectoryStock_, nominalConstraint1ISE_);
    // calculates type-2 constraint ISE and maximum norm
    nominalConstraint2MaxNorm_ =
        calculateConstraintISE(nominalTimeTrajectoriesStock_, nc2TrajectoriesStock_, HvTrajectoryStock_, nominalConstraint2ISE_);
    // calculate penalty
    nominalInequalityConstraintPenalty_ = calculateInequalityConstraintPenalty(nominalTimeTrajectoriesStock_, ncIneqTrajectoriesStock_,
                                                                               hTrajectoryStock_, nominalInequalityConstraintISE_);
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
  calculateRolloutCost(nominalTimeTrajectoriesStock_, nominalEventsPastTheEndIndecesStock_, nominalStateTrajectoriesStock_,
                       nominalInputTrajectoriesStock_, nominalConstraint2ISE_, nominalInequalityConstraintPenalty_, nc2FinalStock_,
                       HvFinalStock_, nominalTotalCost_);

  // display
  if (ddpSettings_.displayInfo_) {
    std::cerr << "\t learningRate 0.0 \t cost: " << nominalTotalCost_ << " \t constraint ISE: " << nominalConstraint1ISE_
              << " \t inequality penalty: " << nominalInequalityConstraintPenalty_
              << " \t inequality ISE: " << nominalInequalityConstraintISE_ << std::endl;
    std::cerr << "\t final constraint type-2:  ";
    size_t itr = 0;
    for (size_t i = initActivePartition_; i <= finalActivePartition_; i++) {
      for (size_t k = 0; k < nc2FinalStock_[i].size(); k++) {
        std::cerr << "[" << itr << "]: " << HvFinalStock_[i][k].head(nc2FinalStock_[i][k]).transpose() << ",  ";
        itr++;
      }
    }
    std::cerr << std::endl;
    std::cerr << "\t forward pass average time step: " << avgTimeStepFP_ * 1e+3 << " [ms]." << std::endl;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
void DDP_BASE<STATE_DIM, INPUT_DIM>::lineSearchWorker(
    size_t workerIndex, scalar_t learningRate, scalar_t& lsTotalCost, scalar_t& lsConstraint1ISE, scalar_t& lsConstraint1MaxNorm,
    scalar_t& lsConstraint2ISE, scalar_t& lsConstraint2MaxNorm, scalar_t& lsInequalityConstraintPenalty,
    scalar_t& lsInequalityConstraintISE, linear_controller_array_t& lsControllersStock, scalar_array2_t& lsTimeTrajectoriesStock,
    size_array2_t& lsEventsPastTheEndIndecesStock, state_vector_array2_t& lsStateTrajectoriesStock,
    input_vector_array2_t& lsInputTrajectoriesStock) {
  // modifying uff by local increments
  for (size_t i = 0; i < numPartitions_; i++) {
    for (size_t k = 0; k < lsControllersStock[i].timeStamp_.size(); k++) {
      lsControllersStock[i].biasArray_[k] += learningRate * lsControllersStock[i].deltaBiasArray_[k];
    }
  }

  try {
    // perform a rollout
    scalar_t avgTimeStepFP =
        rolloutTrajectory(initTime_, initState_, finalTime_, partitioningTimes_, lsControllersStock, lsTimeTrajectoriesStock,
                          lsEventsPastTheEndIndecesStock, lsStateTrajectoriesStock, lsInputTrajectoriesStock, workerIndex);

    // calculate rollout constraints
    size_array2_t lsNc1TrajectoriesStock(numPartitions_);
    constraint1_vector_array2_t lsEvTrajectoryStock(numPartitions_);
    size_array2_t lsNc2TrajectoriesStock(numPartitions_);
    constraint2_vector_array2_t lsHvTrajectoryStock(numPartitions_);
    size_array2_t lsNcIneqTrajectoriesStock(numPartitions_);
    scalar_array3_t lshTrajectoryStock(numPartitions_);
    size_array2_t lsNc2FinalStock(numPartitions_);
    constraint2_vector_array2_t lsHvFinalStock(numPartitions_);

    if (lsComputeISEs_) {
      // calculate rollout constraints
      calculateRolloutConstraints(lsTimeTrajectoriesStock, lsEventsPastTheEndIndecesStock, lsStateTrajectoriesStock,
                                  lsInputTrajectoriesStock, lsNc1TrajectoriesStock, lsEvTrajectoryStock, lsNc2TrajectoriesStock,
                                  lsHvTrajectoryStock, lsNcIneqTrajectoriesStock, lshTrajectoryStock, lsNc2FinalStock, lsHvFinalStock,
                                  workerIndex);
      // calculate constraint type-1 ISE and maximum norm
      lsConstraint1MaxNorm = calculateConstraintISE(lsTimeTrajectoriesStock, lsNc1TrajectoriesStock, lsEvTrajectoryStock, lsConstraint1ISE);
      // calculates type-2 constraint ISE and maximum norm
      lsConstraint2MaxNorm = calculateConstraintISE(lsTimeTrajectoriesStock, lsNc2TrajectoriesStock, lsHvTrajectoryStock, lsConstraint2ISE);
      // inequalityConstraints
      lsInequalityConstraintPenalty = calculateInequalityConstraintPenalty(lsTimeTrajectoriesStock, lsNcIneqTrajectoriesStock,
                                                                           lshTrajectoryStock, lsInequalityConstraintISE, workerIndex);
    } else {
      lsConstraint1ISE = lsConstraint1MaxNorm = 0.0;
      lsConstraint2ISE = lsConstraint2MaxNorm = 0.0;
      lsInequalityConstraintPenalty = 0.0;
      lsInequalityConstraintISE = 0.0;
    }

    // calculate rollout cost
    calculateRolloutCost(lsTimeTrajectoriesStock, lsEventsPastTheEndIndecesStock, lsStateTrajectoriesStock, lsInputTrajectoriesStock,
                         lsConstraint2ISE, lsInequalityConstraintPenalty, lsNc2FinalStock, lsHvFinalStock, lsTotalCost, workerIndex);

    // display
    if (ddpSettings_.displayInfo_) {
      std::string finalConstraintDisplay;
      finalConstraintDisplay = "\t [Thread" + std::to_string(workerIndex) + "] - learningRate " + std::to_string(learningRate) +
                               " \t cost: " + std::to_string(lsTotalCost) + " \t constraint ISE: " + std::to_string(lsConstraint1ISE) +
                               " \t inequality penalty: " + std::to_string(lsInequalityConstraintPenalty) +
                               " \t inequality ISE: " + std::to_string(lsInequalityConstraintISE) + "\n";
      finalConstraintDisplay += "\t final constraint type-2:   ";
      for (size_t i = 0; i < numPartitions_; i++) {
        finalConstraintDisplay += "[" + std::to_string(i) + "]: ";
        for (size_t j = 0; j < lsNc2FinalStock[i].size(); j++) {
          for (size_t m = 0; m < lsNc2FinalStock[i][j]; m++) {
            finalConstraintDisplay += std::to_string(lsHvFinalStock[i][j](m)) + ", ";
          }
        }
        finalConstraintDisplay += "  ";
      }  // end of i loop
      finalConstraintDisplay += "\n\t forward pass average time step: " + std::to_string(avgTimeStepFP * 1e+3) + " [ms].";
      BASE::printString(finalConstraintDisplay);
    }

  } catch (const std::exception& error) {
    lsTotalCost = std::numeric_limits<scalar_t>::max();
    if (ddpSettings_.displayInfo_) {
      BASE::printString("\t [Thread" + std::to_string(workerIndex) + "] rollout with learningRate " + std::to_string(learningRate) +
                        " is terminated: " + error.what());
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
void DDP_BASE<STATE_DIM, INPUT_DIM>::calculateMeritFunction(const scalar_array2_t& timeTrajectoriesStock,
                                                            const size_array2_t& nc1TrajectoriesStock,
                                                            const constraint1_vector_array2_t& EvTrajectoryStock,
                                                            const std::vector<std::vector<Eigen::VectorXd>>& lagrangeTrajectoriesStock,
                                                            const scalar_t& totalCost, scalar_t& meritFunctionValue,
                                                            scalar_t& constraintISE) {
  // add cost function
  meritFunctionValue = totalCost;

  // add the L2 penalty for constraint violation
  calculateConstraintISE(timeTrajectoriesStock, nc1TrajectoriesStock, EvTrajectoryStock, constraintISE);
  double pho = 1.0;
  if (ddpSettings_.maxNumIterations_ > 1) {
    pho = (iteration_ - 1) / (ddpSettings_.maxNumIterations_ - 1) * ddpSettings_.meritFunctionRho_;
  }

  meritFunctionValue += 0.5 * pho * constraintISE;

  // add the the lagrangian term for the constraint
  scalar_t currentIntermediateMerit;
  scalar_t nextIntermediateMerit;

  for (size_t i = 0; i < numPartitions_; i++) {
    // integrates the intermediate merit using the trapezoidal approximation method
    currentIntermediateMerit = 0.0;
    nextIntermediateMerit = 0.0;
    for (size_t k = 0; k + 1 < timeTrajectoriesStock[i].size(); k++) {
      if (k == 0) {
        currentIntermediateMerit = EvTrajectoryStock[i][k].head(nc1TrajectoriesStock[i][k]).transpose() * lagrangeTrajectoriesStock[i][k];
      } else {
        currentIntermediateMerit = nextIntermediateMerit;
      }

      nextIntermediateMerit =
          EvTrajectoryStock[i][k + 1].head(nc1TrajectoriesStock[i][k + 1]).transpose() * lagrangeTrajectoriesStock[i][k + 1];

      meritFunctionValue +=
          0.5 * (currentIntermediateMerit + nextIntermediateMerit) * (timeTrajectoriesStock[i][k + 1] - timeTrajectoriesStock[i][k]);
    }  // end of k loop
  }    // end of i loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
typename DDP_BASE<STATE_DIM, INPUT_DIM>::scalar_t DDP_BASE<STATE_DIM, INPUT_DIM>::calculateConstraintISE(
    const scalar_array2_t& timeTrajectoriesStock, const std::vector<std::vector<size_t>>& nc1TrajectoriesStock,
    const constraint1_vector_array2_t& EvTrajectoriesStock, scalar_t& constraintISE) {
  constraintISE = 0.0;
  double maxConstraintNorm = 0.0;

  scalar_t currentSquaredNormError;
  scalar_t nextSquaredNormError;

  for (size_t i = 0; i < numPartitions_; i++) {
    currentSquaredNormError = 0.0;
    nextSquaredNormError = 0.0;

    for (size_t k = 0; k + 1 < timeTrajectoriesStock[i].size(); k++) {
      if (k == 0) {
        const size_t& nc1 = nc1TrajectoriesStock[i][0];
        if (nc1 > 0) {
          currentSquaredNormError = EvTrajectoriesStock[i][0].head(nc1).squaredNorm();
        } else {
          currentSquaredNormError = 0.0;
        }
      } else {
        currentSquaredNormError = nextSquaredNormError;
      }

      maxConstraintNorm = ((maxConstraintNorm < currentSquaredNormError) ? currentSquaredNormError : maxConstraintNorm);

      const size_t& nc1 = nc1TrajectoriesStock[i][k + 1];
      if (nc1 > 0) {
        nextSquaredNormError = EvTrajectoriesStock[i][k + 1].head(nc1).squaredNorm();
      } else {
        nextSquaredNormError = 0.0;
      }

      constraintISE +=
          0.5 * (currentSquaredNormError + nextSquaredNormError) * (timeTrajectoriesStock[i][k + 1] - timeTrajectoriesStock[i][k]);

    }  // end of k loop
  }    // end of i loop

  return sqrt(maxConstraintNorm);
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
typename DDP_BASE<STATE_DIM, INPUT_DIM>::scalar_t DDP_BASE<STATE_DIM, INPUT_DIM>::calculateInequalityConstraintPenalty(
    const scalar_array2_t& timeTrajectoriesStock, const size_array2_t& ncIneqTrajectoriesStock, const scalar_array3_t& hTrajectoriesStock,
    scalar_t& inequalityISE, size_t workerIndex /* = 0 */) {
  scalar_t constraintPenalty(0.0);
  scalar_t currentPenalty(0.0);
  scalar_t nextPenalty(0.0);

  inequalityISE = 0.0;
  scalar_t currentInequalityViolationSquaredNorm(0.0);
  scalar_t nextInequalityViolationSquaredNorm(0.0);

  for (size_t i = 0; i < numPartitions_; i++) {
    for (size_t k = 0; k + 1 < timeTrajectoriesStock[i].size(); k++) {
      if (k == 0) {
        if (ncIneqTrajectoriesStock[i][0] > 0) {
          penaltyPtrStock_[workerIndex]->getPenaltyCost(hTrajectoriesStock[i][k], currentPenalty);
          penaltyPtrStock_[workerIndex]->getConstraintViolationSquaredNorm(hTrajectoriesStock[i][k], currentInequalityViolationSquaredNorm);
        } else {
          currentPenalty = 0.0;
          currentInequalityViolationSquaredNorm = 0.0;
        }
      } else {
        currentPenalty = nextPenalty;
        currentInequalityViolationSquaredNorm = nextInequalityViolationSquaredNorm;
      }

      if (ncIneqTrajectoriesStock[i][k + 1] > 0) {
        penaltyPtrStock_[workerIndex]->getPenaltyCost(hTrajectoriesStock[i][k + 1], nextPenalty);
        penaltyPtrStock_[workerIndex]->getConstraintViolationSquaredNorm(hTrajectoriesStock[i][k + 1], nextInequalityViolationSquaredNorm);
      } else {
        nextPenalty = 0.0;
        nextInequalityViolationSquaredNorm = 0.0;
      }

      constraintPenalty += 0.5 * (currentPenalty + nextPenalty) * (timeTrajectoriesStock[i][k + 1] - timeTrajectoriesStock[i][k]);
      inequalityISE += 0.5 * (currentInequalityViolationSquaredNorm + nextInequalityViolationSquaredNorm) *
                       (timeTrajectoriesStock[i][k + 1] - timeTrajectoriesStock[i][k]);

    }  // end of k loop
  }    // end of i loop

  return constraintPenalty;
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
void DDP_BASE<STATE_DIM, INPUT_DIM>::truncateConterller(const scalar_array_t& partitioningTimes, const double& initTime,
                                                        linear_controller_array_t& controllersStock, size_t& initActivePartition,
                                                        linear_controller_array_t& deletedcontrollersStock) {
  deletedcontrollersStock.resize(numPartitions_);
  for (size_t i = 0; i < numPartitions_; i++) {
    deletedcontrollersStock[i].clear();
  }

  // finding the active subsystem index at initTime_
  initActivePartition = lookup::findBoundedActiveIntervalInTimeArray(partitioningTimes, initTime);

  // saving the deleting part and clearing controllersStock
  for (size_t i = 0; i < initActivePartition; i++) {
    deletedcontrollersStock[i].swap(controllersStock[i]);
  }

  if (controllersStock[initActivePartition].empty()) {
    return;
  }

  // interpolating uff
  LinearInterpolation<input_vector_t, Eigen::aligned_allocator<input_vector_t>> uffFunc;
  uffFunc.setData(&controllersStock[initActivePartition].timeStamp_, &controllersStock[initActivePartition].biasArray_);
  input_vector_t uffInit;
  const auto indexAlpha = uffFunc.interpolate(initTime, uffInit);
  auto greatestLessTimeStampIndex = indexAlpha.first;

  // interpolating k
  LinearInterpolation<input_state_matrix_t, Eigen::aligned_allocator<input_state_matrix_t>> kFunc;
  kFunc.setData(&controllersStock[initActivePartition].timeStamp_, &controllersStock[initActivePartition].gainArray_);
  input_state_matrix_t kInit;
  kFunc.interpolate(indexAlpha, kInit);

  // deleting the controller in the active subsystem for the subsystems before initTime
  if (greatestLessTimeStampIndex > 0) {
    deletedcontrollersStock[initActivePartition].timeStamp_.resize(greatestLessTimeStampIndex + 1);
    deletedcontrollersStock[initActivePartition].biasArray_.resize(greatestLessTimeStampIndex + 1);
    deletedcontrollersStock[initActivePartition].gainArray_.resize(greatestLessTimeStampIndex + 1);
    for (size_t k = 0; k <= greatestLessTimeStampIndex; k++) {
      deletedcontrollersStock[initActivePartition].timeStamp_[k] = controllersStock[initActivePartition].timeStamp_[k];
      deletedcontrollersStock[initActivePartition].biasArray_[k] = controllersStock[initActivePartition].biasArray_[k];
      deletedcontrollersStock[initActivePartition].gainArray_[k] = controllersStock[initActivePartition].gainArray_[k];
    }

    controllersStock[initActivePartition].timeStamp_.erase(
        controllersStock[initActivePartition].timeStamp_.begin(),
        controllersStock[initActivePartition].timeStamp_.begin() + greatestLessTimeStampIndex);
    controllersStock[initActivePartition].biasArray_.erase(
        controllersStock[initActivePartition].biasArray_.begin(),
        controllersStock[initActivePartition].biasArray_.begin() + greatestLessTimeStampIndex);
    controllersStock[initActivePartition].gainArray_.erase(
        controllersStock[initActivePartition].gainArray_.begin(),
        controllersStock[initActivePartition].gainArray_.begin() + greatestLessTimeStampIndex);
  }

  controllersStock[initActivePartition].timeStamp_[0] = initTime;
  controllersStock[initActivePartition].biasArray_[0] = uffInit;
  controllersStock[initActivePartition].gainArray_[0] = kInit;
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
void DDP_BASE<STATE_DIM, INPUT_DIM>::calculateControllerUpdateMaxNorm(scalar_t& maxDeltaUffNorm, scalar_t& maxDeltaUeeNorm) {
  maxDeltaUffNorm = 0.0;
  maxDeltaUeeNorm = 0.0;
  for (size_t i = initActivePartition_; i <= finalActivePartition_; i++) {
    nominalStateFunc_[0].setData(&(nominalTimeTrajectoriesStock_[i]), &(nominalStateTrajectoriesStock_[i]));
    nominalInputFunc_[0].setData(&(nominalTimeTrajectoriesStock_[i]), &(nominalInputTrajectoriesStock_[i]));

    for (size_t k = 0; k < nominalControllersStock_[i].timeStamp_.size(); k++) {
      maxDeltaUffNorm = std::max(maxDeltaUffNorm, nominalControllersStock_[i].deltaBiasArray_[k].norm());

      state_vector_t nominalState;
      const auto indexAlpha = nominalStateFunc_[0].interpolate(nominalControllersStock_[i].timeStamp_[k], nominalState);
      input_vector_t nominalInput;
      nominalInputFunc_[0].interpolate(indexAlpha, nominalInput);
      input_vector_t deltaUee =
          nominalInput - nominalControllersStock_[i].gainArray_[k] * nominalState - nominalControllersStock_[i].biasArray_[k];
      maxDeltaUeeNorm = std::max(maxDeltaUeeNorm, deltaUee.norm());

    }  // end of k loop
  }    // end of i loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
void DDP_BASE<STATE_DIM, INPUT_DIM>::updateNominalControllerPtrStock() {
  nominalControllerPtrStock_.clear();
  nominalControllerPtrStock_.reserve(nominalControllersStock_.size());

  for (linear_controller_t& controller : nominalControllersStock_) {
    nominalControllerPtrStock_.push_back(&controller);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
void DDP_BASE<STATE_DIM, INPUT_DIM>::printRolloutInfo() {
  std::cerr << "optimization cost:         " << nominalTotalCost_ << std::endl;
  std::cerr << "constraint type-1 ISE:     " << nominalConstraint1ISE_ << std::endl;
  std::cerr << "constraint type-1 MaxNorm: " << nominalConstraint1MaxNorm_ << std::endl;
  std::cerr << "constraint type-2 ISE:     " << nominalConstraint2ISE_ << std::endl;
  std::cerr << "constraint type-2 MaxNorm: " << nominalConstraint2MaxNorm_ << std::endl;
  std::cerr << "inequality Penalty:        " << nominalInequalityConstraintPenalty_ << std::endl;
  std::cerr << "inequality ISE:            " << nominalInequalityConstraintISE_ << std::endl;
  std::cerr << "final constraint type-2: 	 ";
  size_t itr = 0;
  for (size_t i = initActivePartition_; i <= finalActivePartition_; i++) {
    for (size_t k = 0; k < nc2FinalStock_[i].size(); k++) {
      std::cerr << "[" << itr << "]: " << HvFinalStock_[i][k].head(nc2FinalStock_[i][k]).transpose() << ",  ";
      itr++;
    }
  }
  std::cerr << std::endl;
  std::cerr << "forward pass average time step:  " << avgTimeStepFP_ * 1e+3 << " [ms]." << std::endl;
  std::cerr << "backward pass average time step: " << avgTimeStepBP_ * 1e+3 << " [ms]." << std::endl;
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
void DDP_BASE<STATE_DIM, INPUT_DIM>::adjustController(const scalar_array_t& newEventTimes, const scalar_array_t& controllerEventTimes) {
  // adjust the nominal controllerStock using trajectory spreading
  if (nominalControllersStock_.size() > 0) {
    trajectorySpreadingController_.adjustController(newEventTimes, controllerEventTimes, nominalControllersStock_);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
void DDP_BASE<STATE_DIM, INPUT_DIM>::getValueFuntion(const scalar_t& time, const state_vector_t& state, scalar_t& valueFuntion) {
  size_t activeSubsystem = lookup::findBoundedActiveIntervalInTimeArray(partitioningTimes_, time);

  state_matrix_t Sm;
  LinearInterpolation<state_matrix_t, Eigen::aligned_allocator<state_matrix_t>> SmFunc(&SsTimeTrajectoryStock_[activeSubsystem],
                                                                                       &SmTrajectoryStock_[activeSubsystem]);
  const auto indexAlpha = SmFunc.interpolate(time, Sm);

  state_vector_t Sv;
  LinearInterpolation<state_vector_t, Eigen::aligned_allocator<state_vector_t>> SvFunc(&SsTimeTrajectoryStock_[activeSubsystem],
                                                                                       &SvTrajectoryStock_[activeSubsystem]);
  SvFunc.interpolate(indexAlpha, Sv);

  eigen_scalar_t s;
  LinearInterpolation<eigen_scalar_t, Eigen::aligned_allocator<eigen_scalar_t>> sFunc(&SsTimeTrajectoryStock_[activeSubsystem],
                                                                                      &sTrajectoryStock_[activeSubsystem]);
  sFunc.interpolate(indexAlpha, s);

  state_vector_t xNominal;
  LinearInterpolation<state_vector_t, Eigen::aligned_allocator<state_vector_t>> xNominalFunc(
      &nominalTimeTrajectoriesStock_[activeSubsystem], &nominalStateTrajectoriesStock_[activeSubsystem]);
  xNominalFunc.interpolate(time, xNominal);

  state_vector_t deltaX = state - xNominal;

  valueFuntion = (s + deltaX.transpose() * Sv + 0.5 * deltaX.transpose() * Sm * deltaX).eval()(0);
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
void DDP_BASE<STATE_DIM, INPUT_DIM>::useParallelRiccatiSolverFromInitItr(bool flag) {
  useParallelRiccatiSolverFromInitItr_ = flag;
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
void DDP_BASE<STATE_DIM, INPUT_DIM>::blockwiseMovingHorizon(bool flag) {
  blockwiseMovingHorizon_ = flag;
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
void DDP_BASE<STATE_DIM, INPUT_DIM>::getPerformanceIndeces(scalar_t& costFunction, scalar_t& constraint1ISE,
                                                           scalar_t& constraint2ISE) const {
  costFunction = nominalTotalCost_;
  constraint1ISE = nominalConstraint1ISE_;
  constraint2ISE = nominalConstraint2ISE_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
size_t DDP_BASE<STATE_DIM, INPUT_DIM>::getNumIterations() const {
  return iteration_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
void DDP_BASE<STATE_DIM, INPUT_DIM>::getIterationsLog(eigen_scalar_array_t& iterationCost, eigen_scalar_array_t& iterationISE1,
                                                      eigen_scalar_array_t& iterationISE2) const {
  iterationCost = iterationCost_;
  iterationISE1 = iterationISE1_;
  iterationISE2 = iterationISE2_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
void DDP_BASE<STATE_DIM, INPUT_DIM>::getIterationsLogPtr(const eigen_scalar_array_t*& iterationCostPtr,
                                                         const eigen_scalar_array_t*& iterationISE1Ptr,
                                                         const eigen_scalar_array_t*& iterationISE2Ptr) const {
  iterationCostPtr = &iterationCost_;
  iterationISE1Ptr = &iterationISE1_;
  iterationISE2Ptr = &iterationISE2_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
DDP_Settings& DDP_BASE<STATE_DIM, INPUT_DIM>::ddpSettings() {
  return ddpSettings_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
const typename DDP_BASE<STATE_DIM, INPUT_DIM>::controller_ptr_array_t& DDP_BASE<STATE_DIM, INPUT_DIM>::getController() const {
  // updateNominalControllerPtrStock(); // cannot be done in const member
  return nominalControllerPtrStock_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
void DDP_BASE<STATE_DIM, INPUT_DIM>::getControllerPtr(const controller_ptr_array_t*& controllersPtrStock) const {
  // updateNominalControllerPtrStock(); // cannot be done in const member
  controllersPtrStock = &nominalControllerPtrStock_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
const typename DDP_BASE<STATE_DIM, INPUT_DIM>::scalar_array2_t& DDP_BASE<STATE_DIM, INPUT_DIM>::getNominalTimeTrajectories() const {
  return nominalTimeTrajectoriesStock_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
const typename DDP_BASE<STATE_DIM, INPUT_DIM>::state_vector_array2_t& DDP_BASE<STATE_DIM, INPUT_DIM>::getNominalStateTrajectories() const {
  return nominalStateTrajectoriesStock_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
const typename DDP_BASE<STATE_DIM, INPUT_DIM>::input_vector_array2_t& DDP_BASE<STATE_DIM, INPUT_DIM>::getNominalInputTrajectories() const {
  return nominalInputTrajectoriesStock_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
void DDP_BASE<STATE_DIM, INPUT_DIM>::getNominalTrajectoriesPtr(const scalar_array2_t*& nominalTimeTrajectoriesStockPtr,
                                                               const state_vector_array2_t*& nominalStateTrajectoriesStockPtr,
                                                               const input_vector_array2_t*& nominalInputTrajectoriesStockPtr) const {
  nominalTimeTrajectoriesStockPtr = &nominalTimeTrajectoriesStock_;
  nominalStateTrajectoriesStockPtr = &nominalStateTrajectoriesStock_;
  nominalInputTrajectoriesStockPtr = &nominalInputTrajectoriesStock_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
void DDP_BASE<STATE_DIM, INPUT_DIM>::swapNominalTrajectories(scalar_array2_t& nominalTimeTrajectoriesStock,
                                                             state_vector_array2_t& nominalStateTrajectoriesStock,
                                                             input_vector_array2_t& nominalInputTrajectoriesStock) {
  nominalTimeTrajectoriesStock.swap(nominalTimeTrajectoriesStock_);
  nominalStateTrajectoriesStock.swap(nominalStateTrajectoriesStock_);
  nominalInputTrajectoriesStock.swap(nominalInputTrajectoriesStock_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
const typename DDP_BASE<STATE_DIM, INPUT_DIM>::scalar_t& DDP_BASE<STATE_DIM, INPUT_DIM>::getFinalTime() const {
  return finalTime_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
const typename DDP_BASE<STATE_DIM, INPUT_DIM>::scalar_array_t& DDP_BASE<STATE_DIM, INPUT_DIM>::getPartitioningTimes() const {
  return partitioningTimes_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
void DDP_BASE<STATE_DIM, INPUT_DIM>::getCostDesiredTrajectoriesPtr(const cost_desired_trajectories_t*& costDesiredTrajectoriesPtr) const {
  costDesiredTrajectoriesPtr = &costDesiredTrajectories_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
void DDP_BASE<STATE_DIM, INPUT_DIM>::setCostDesiredTrajectories(const cost_desired_trajectories_t& costDesiredTrajectories) {
  std::lock_guard<std::mutex> lock(costDesiredTrajectoriesBufferMutex_);
  costDesiredTrajectoriesUpdated_ = true;
  costDesiredTrajectoriesBuffer_ = costDesiredTrajectories;
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
void DDP_BASE<STATE_DIM, INPUT_DIM>::setCostDesiredTrajectories(const scalar_array_t& desiredTimeTrajectory,
                                                                const dynamic_vector_array_t& desiredStateTrajectory,
                                                                const dynamic_vector_array_t& desiredInputTrajectory) {
  std::lock_guard<std::mutex> lock(costDesiredTrajectoriesBufferMutex_);
  costDesiredTrajectoriesUpdated_ = true;
  costDesiredTrajectoriesBuffer_.desiredTimeTrajectory() = desiredTimeTrajectory;
  costDesiredTrajectoriesBuffer_.desiredStateTrajectory() = desiredStateTrajectory;
  costDesiredTrajectoriesBuffer_.desiredInputTrajectory() = desiredInputTrajectory;
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
void DDP_BASE<STATE_DIM, INPUT_DIM>::swapCostDesiredTrajectories(cost_desired_trajectories_t& costDesiredTrajectories) {
  std::lock_guard<std::mutex> lock(costDesiredTrajectoriesBufferMutex_);
  costDesiredTrajectoriesUpdated_ = true;
  costDesiredTrajectoriesBuffer_.swap(costDesiredTrajectories);
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
void DDP_BASE<STATE_DIM, INPUT_DIM>::swapCostDesiredTrajectories(scalar_array_t& desiredTimeTrajectory,
                                                                 dynamic_vector_array_t& desiredStateTrajectory,
                                                                 dynamic_vector_array_t& desiredInputTrajectory) {
  std::lock_guard<std::mutex> lock(costDesiredTrajectoriesBufferMutex_);
  costDesiredTrajectoriesUpdated_ = true;
  costDesiredTrajectoriesBuffer_.desiredTimeTrajectory().swap(desiredTimeTrajectory);
  costDesiredTrajectoriesBuffer_.desiredStateTrajectory().swap(desiredStateTrajectory);
  costDesiredTrajectoriesBuffer_.desiredInputTrajectory().swap(desiredInputTrajectory);
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
bool DDP_BASE<STATE_DIM, INPUT_DIM>::costDesiredTrajectoriesUpdated() const {
  return costDesiredTrajectoriesUpdated_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
void DDP_BASE<STATE_DIM, INPUT_DIM>::rewindOptimizer(const size_t& firstIndex) {
  // No rewind is needed
  if (firstIndex == 0) {
    return;
  }

  // increment rewindCounter_
  rewindCounter_ += firstIndex;

  if (firstIndex > numPartitions_) {
    throw std::runtime_error("Index for rewinding is greater than the current size.");
  }

  const size_t preservedLength = numPartitions_ - firstIndex;
  for (size_t i = 0; i < numPartitions_; i++) {
    if (i < preservedLength) {
      nominalControllersStock_[i].swap(nominalControllersStock_[firstIndex + i]);
      SmFinalStock_[i] = SmFinalStock_[firstIndex + i];
      SvFinalStock_[i] = SvFinalStock_[firstIndex + i];
      SveFinalStock_[i] = SveFinalStock_[firstIndex + i];
      sFinalStock_[i] = sFinalStock_[firstIndex + i];
      xFinalStock_[i] = xFinalStock_[firstIndex + i];
    } else {
      nominalControllersStock_[i].clear();
      SmFinalStock_[i].setZero();
      SvFinalStock_[i].setZero();
      SveFinalStock_[i].setZero();
      sFinalStock_[i].setZero();
      xFinalStock_[i].setZero();
    }
  }

  updateNominalControllerPtrStock();
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
const unsigned long long int& DDP_BASE<STATE_DIM, INPUT_DIM>::getRewindCounter() const {
  return rewindCounter_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
void DDP_BASE<STATE_DIM, INPUT_DIM>::setupOptimizer(const size_t& numPartitions) {
  if (numPartitions == 0) {
    throw std::runtime_error("Number of partitions cannot be zero!");
  }

  /*
   * nominal trajectories
   */
  nominalControllersStock_.resize(numPartitions);
  updateNominalControllerPtrStock();
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
  SmFinalStock_ = state_matrix_array_t(numPartitions, state_matrix_t::Zero());
  SvFinalStock_ = state_vector_array_t(numPartitions, state_vector_t::Zero());
  SveFinalStock_ = state_vector_array_t(numPartitions, state_vector_t::Zero());
  sFinalStock_ = eigen_scalar_array_t(numPartitions, eigen_scalar_t::Zero());
  xFinalStock_ = state_vector_array_t(numPartitions, state_vector_t::Zero());

  SsTimeTrajectoryStock_.resize(numPartitions);
  SsNormalizedTimeTrajectoryStock_.resize(numPartitions);
  SsNormalizedEventsPastTheEndIndecesStock_.resize(numPartitions);
  sTrajectoryStock_.resize(numPartitions);
  SvTrajectoryStock_.resize(numPartitions);
  SveTrajectoryStock_.resize(numPartitions);
  SmTrajectoryStock_.resize(numPartitions);

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
  PmTrajectoryStock_.resize(numPartitions);
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
void DDP_BASE<STATE_DIM, INPUT_DIM>::runInit() {
#ifdef BENCHMARK
  // Benchmarking
  BENCHMARK_nIterationsLQ_++;
  BENCHMARK_nIterationsBP_++;
  BENCHMARK_nIterationsFP_++;
  BENCHMARK_start_ = std::chrono::steady_clock::now();
#endif

  // initial controller rollout
  avgTimeStepFP_ =
      rolloutTrajectory(initTime_, initState_, finalTime_, partitioningTimes_, nominalControllersStock_, nominalTimeTrajectoriesStock_,
                        nominalEventsPastTheEndIndecesStock_, nominalStateTrajectoriesStock_, nominalInputTrajectoriesStock_);

#ifdef BENCHMARK
  BENCHMARK_end_ = std::chrono::steady_clock::now();
  auto BENCHMARK_diff_ = BENCHMARK_end_ - BENCHMARK_start_;
  BENCHMARK_tAvgFP_ = ((1.0 - 1.0 / BENCHMARK_nIterationsFP_) * BENCHMARK_tAvgFP_) +
                      (1.0 / BENCHMARK_nIterationsFP_) * std::chrono::duration_cast<std::chrono::microseconds>(BENCHMARK_diff_).count();
  BENCHMARK_start_ = std::chrono::steady_clock::now();
#endif

  // linearizing the dynamics and quadratizing the cost function along nominal trajectories
  approximateOptimalControlProblem();

  // to check convergence of the main loop, we need to compute the total cost and ISEs
  bool computePerformanceIndex = ddpSettings_.displayInfo_ || ddpSettings_.maxNumIterations_ > 1;
  if (computePerformanceIndex) {
    // calculate rollout constraint type-1 ISE
    nominalConstraint1MaxNorm_ =
        calculateConstraintISE(nominalTimeTrajectoriesStock_, nc1TrajectoriesStock_, EvTrajectoryStock_, nominalConstraint1ISE_);
    // calculate rollout constraint type-2 ISE
    if (!ddpSettings_.noStateConstraints_) {
      nominalConstraint2MaxNorm_ =
          calculateConstraintISE(nominalTimeTrajectoriesStock_, nc2TrajectoriesStock_, HvTrajectoryStock_, nominalConstraint2ISE_);
    } else {
      nominalConstraint2ISE_ = nominalConstraint2MaxNorm_ = 0.0;
    }
    // calculate rollout cost
    calculateRolloutCost(nominalTimeTrajectoriesStock_, nominalEventsPastTheEndIndecesStock_, nominalStateTrajectoriesStock_,
                         nominalInputTrajectoriesStock_, nominalTotalCost_);
  } else {
    nominalTotalCost_ = 0.0;
    nominalConstraint1ISE_ = nominalConstraint1MaxNorm_ = 0.0;
    nominalConstraint2ISE_ = nominalConstraint2MaxNorm_ = 0.0;
  }

#ifdef BENCHMARK
  BENCHMARK_end_ = std::chrono::steady_clock::now();
  BENCHMARK_diff_ = BENCHMARK_end_ - BENCHMARK_start_;
  BENCHMARK_tAvgLQ_ = ((1.0 - 1.0 / BENCHMARK_nIterationsLQ_) * BENCHMARK_tAvgLQ_) +
                      (1.0 / BENCHMARK_nIterationsLQ_) * std::chrono::duration_cast<std::chrono::microseconds>(BENCHMARK_diff_).count();
  BENCHMARK_start_ = std::chrono::steady_clock::now();
#endif

  // solve Riccati equations
  avgTimeStepBP_ = solveSequentialRiccatiEquations(SmHeuristics_, SvHeuristics_, sHeuristics_);
  // calculate controller
  if (ddpSettings_.useRiccatiSolver_) {
    calculateController();
  } else {
    throw std::runtime_error("useRiccatiSolver=false is not valid.");
  }

#ifdef BENCHMARK
  BENCHMARK_end_ = std::chrono::steady_clock::now();
  BENCHMARK_diff_ = BENCHMARK_end_ - BENCHMARK_start_;
  BENCHMARK_tAvgBP_ = ((1.0 - 1.0 / BENCHMARK_nIterationsBP_) * BENCHMARK_tAvgBP_) +
                      (1.0 / BENCHMARK_nIterationsBP_) * std::chrono::duration_cast<std::chrono::microseconds>(BENCHMARK_diff_).count();
#endif

  // display
  if (ddpSettings_.displayInfo_) {
    printRolloutInfo();
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
void DDP_BASE<STATE_DIM, INPUT_DIM>::runIteration() {
#ifdef BENCHMARK
  // Benchmarking
  BENCHMARK_nIterationsLQ_++;
  BENCHMARK_nIterationsBP_++;
  BENCHMARK_nIterationsFP_++;
  BENCHMARK_start_ = std::chrono::steady_clock::now();
#endif

  bool computeISEs = ddpSettings_.displayInfo_ || !ddpSettings_.noStateConstraints_;

  // finding the optimal learningRate
  maxLearningRate_ = ddpSettings_.maxLearningRate_;
  lineSearch(computeISEs);

#ifdef BENCHMARK
  BENCHMARK_end_ = std::chrono::steady_clock::now();
  BENCHMARK_diff_ = BENCHMARK_end_ - BENCHMARK_start_;
  BENCHMARK_tAvgFP_ = ((1.0 - 1.0 / BENCHMARK_nIterationsFP_) * BENCHMARK_tAvgFP_) +
                      (1.0 / BENCHMARK_nIterationsFP_) * std::chrono::duration_cast<std::chrono::microseconds>(BENCHMARK_diff_).count();
  BENCHMARK_start_ = std::chrono::steady_clock::now();
#endif

  // linearizing the dynamics and quadratizing the cost function along nominal trajectories
  approximateOptimalControlProblem();

  // to check convergence of the main loop, we need to compute ISEs
  if (!computeISEs) {
    // calculate constraint type-1 ISE and maximum norm
    nominalConstraint1MaxNorm_ =
        calculateConstraintISE(nominalTimeTrajectoriesStock_, nc1TrajectoriesStock_, EvTrajectoryStock_, nominalConstraint1ISE_);
    // calculates type-2 constraint ISE and maximum norm
    if (!ddpSettings_.noStateConstraints_) {
      nominalConstraint2MaxNorm_ =
          calculateConstraintISE(nominalTimeTrajectoriesStock_, nc2TrajectoriesStock_, HvTrajectoryStock_, nominalConstraint2ISE_);
    } else {
      nominalConstraint2ISE_ = nominalConstraint2MaxNorm_ = 0.0;
    }
  }

#ifdef BENCHMARK
  BENCHMARK_end_ = std::chrono::steady_clock::now();
  BENCHMARK_diff_ = BENCHMARK_end_ - BENCHMARK_start_;
  BENCHMARK_tAvgLQ_ = ((1.0 - 1.0 / BENCHMARK_nIterationsLQ_) * BENCHMARK_tAvgLQ_) +
                      (1.0 / BENCHMARK_nIterationsLQ_) * std::chrono::duration_cast<std::chrono::microseconds>(BENCHMARK_diff_).count();
  BENCHMARK_start_ = std::chrono::steady_clock::now();
#endif

  // solve Riccati equations
  avgTimeStepBP_ = solveSequentialRiccatiEquations(SmHeuristics_, SvHeuristics_, sHeuristics_);
  // calculate controller
  if (ddpSettings_.useRiccatiSolver_) {
    calculateController();
  } else {
    throw std::runtime_error("useRiccatiSolver=false is not valid.");
  }

#ifdef BENCHMARK
  BENCHMARK_end_ = std::chrono::steady_clock::now();
  BENCHMARK_diff_ = BENCHMARK_end_ - BENCHMARK_start_;
  BENCHMARK_tAvgBP_ = ((1.0 - 1.0 / BENCHMARK_nIterationsBP_) * BENCHMARK_tAvgBP_) +
                      (1.0 / BENCHMARK_nIterationsBP_) * std::chrono::duration_cast<std::chrono::microseconds>(BENCHMARK_diff_).count();
#endif

  // display
  if (ddpSettings_.displayInfo_) {
    printRolloutInfo();
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
void DDP_BASE<STATE_DIM, INPUT_DIM>::runExit() {
  //	// add the deleted parts of the controller
  //	for (size_t i=0; i<initActivePartition_; i++)
  //		nominalControllersStock_[i].swap(deletedcontrollersStock_[i]);
  //
  //	if (deletedcontrollersStock_[initActivePartition_].timeStamp_.empty()==false) {
  //
  //		nominalControllersStock_[initActivePartition_].swap(deletedcontrollersStock_[initActivePartition_]);
  //
  //		for (size_t k=0; k<deletedcontrollersStock_[initActivePartition_].timeStamp_.size(); k++) {
  //			nominalControllersStock_[initActivePartition_].timeStamp_.push_back(deletedcontrollersStock_[initActivePartition_].timeStamp_[k]);
  //			nominalControllersStock_[initActivePartition_].biasArray_.push_back(deletedcontrollersStock_[initActivePartition_].biasArray_[k]);
  //			nominalControllersStock_[initActivePartition_].gainArray_.push_back(deletedcontrollersStock_[initActivePartition_].gainArray_[k]);
  //		}  // end of k loop
  //	}
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
void DDP_BASE<STATE_DIM, INPUT_DIM>::run(const scalar_t& initTime, const state_vector_t& initState, const scalar_t& finalTime,
                                         const scalar_array_t& partitioningTimes) {
  const size_t numPartitions = partitioningTimes.size() - 1;

  linear_controller_array_t noInitialController(numPartitions, linear_controller_t());
  controller_ptr_array_t noInitialControllerPtrArray(numPartitions);
  for (size_t i = 0; i < numPartitions; i++) {
    noInitialControllerPtrArray[i] = &noInitialController[i];
  }

  // call the "run" method which uses the internal controllers stock (i.e. nominalControllersStock_)
  run(initTime, initState, finalTime, partitioningTimes, noInitialControllerPtrArray);
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
void DDP_BASE<STATE_DIM, INPUT_DIM>::run(const scalar_t& initTime, const state_vector_t& initState, const scalar_t& finalTime,
                                         const scalar_array_t& partitioningTimes, const controller_ptr_array_t& controllersPtrStock) {
  if (ddpSettings_.displayInfo_) {
    std::cerr << std::endl;
    std::cerr << "++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
    std::cerr << "+++++++++++++ " + algorithmName_ + " solver is initialized ++++++++++++++" << std::endl;
    std::cerr << "++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
  }

  // infeasible learning rate adjustment scheme
  if (ddpSettings_.maxLearningRate_ < ddpSettings_.minLearningRate_ - OCS2NumericTraits<scalar_t>::limitEpsilon()) {
    throw std::runtime_error("The maximum learning rate is smaller than the minimum learning rate.");
  }

  if (partitioningTimes.empty()) {
    throw std::runtime_error("There should be at least one time partition.");
  }

  if (!initState.allFinite()) {
    throw std::runtime_error("DDP: initial state is not finite (time: " + std::to_string(initTime) + " [sec]).");
  }

  // update numPartitions_ if it has been changed
  if (numPartitions_ + 1 != partitioningTimes.size()) {
    numPartitions_ = partitioningTimes.size() - 1;
    partitioningTimes_ = partitioningTimes;
    setupOptimizer(numPartitions_);
  }

  // update partitioningTimes_
  partitioningTimes_ = partitioningTimes;

  // Use the input controller if it is not empty otherwise use the internal controller (nominalControllersStock_).
  // In the later case 2 scenarios are possible: either the internal controller is already set (such as the MPC case
  // where the warm starting option is set true) or the internal controller is empty in which instead of performing
  // a rollout the operating trajectories will be used.
  if (!controllersPtrStock.empty()) {
    if (controllersPtrStock.size() != numPartitions_) {
      throw std::runtime_error("controllersPtrStock has less controllers than the number of partitions.");
    }

    nominalControllersStock_.clear();
    nominalControllersStock_.reserve(numPartitions_);

    // ensure initial controllers are of the right type, then assign
    for (auto& controllersStock_i : controllersPtrStock) {
      auto linearCtrlPtr = dynamic_cast<linear_controller_t*>(controllersStock_i);
      if (linearCtrlPtr == nullptr) {
        throw std::runtime_error("DDP_BASE::run -- controller must be a linear_controller_t.");
      }
      nominalControllersStock_.emplace_back(*linearCtrlPtr);
    }
  } else {
    if (nominalControllersStock_.size() != numPartitions_) {
      throw std::runtime_error("The internal controller is not compatible with the number of partitions.");
    }
  }

  // set desired trajectories of cost if it is updated
  if (costDesiredTrajectoriesUpdated_) {
    std::lock_guard<std::mutex> lock(costDesiredTrajectoriesBufferMutex_);
    costDesiredTrajectoriesUpdated_ = false;
    costDesiredTrajectories_.swap(costDesiredTrajectoriesBuffer_);
  }

  // update the logic rules in the beginning of the run routine
  bool logicRulesModified = BASE::getLogicRulesMachinePtr()->updateLogicRules(partitioningTimes_);

  // display
  if (ddpSettings_.displayInfo_) {
    std::cerr << std::endl << "Rewind Counter: " << rewindCounter_ << std::endl;
    std::cerr << algorithmName_ + " solver starts from initial time " << initTime << " to final time " << finalTime << ".";
    BASE::getLogicRulesMachinePtr()->display();
    std::cerr << std::endl;
  }

  iteration_ = 0;
  initState_ = initState;
  initTime_ = initTime;
  finalTime_ = finalTime;

  iterationCost_.clear();
  iterationISE1_.clear();
  iterationISE2_.clear();

  // finding the initial active partition index and truncating the controller
  truncateConterller(partitioningTimes_, initTime_, nominalControllersStock_, initActivePartition_, deletedcontrollersStock_);

  // the final active partition index.
  finalActivePartition_ = lookup::findBoundedActiveIntervalInTimeArray(partitioningTimes_, finalTime_);

  // check if after the truncation the internal controller is empty
  bool isInitInternalControllerEmpty = false;
  for (const linear_controller_t& controller : nominalControllersStock_) {
    isInitInternalControllerEmpty = isInitInternalControllerEmpty || controller.empty();
  }

  // display
  if (ddpSettings_.displayInfo_) {
    std::cerr << "\n#### Iteration " << iteration_ << " (Dynamics might have been violated)" << std::endl;
  }

  // if a controller is not set for a partition
  for (size_t i = 0; i < numPartitions_; i++) {
    initialControllerDesignStock_[i] = nominalControllersStock_[i].empty() ? true : false;
  }

  // run DDP initializer and update the member variables
  runInit();

  // after iteration zero always allow feedforward policy update
  for (size_t i = 0; i < numPartitions_; i++) {
    initialControllerDesignStock_[i] = false;
  }

  iterationCost_.push_back((Eigen::VectorXd(1) << nominalTotalCost_).finished());
  iterationISE1_.push_back((Eigen::VectorXd(1) << nominalConstraint1ISE_).finished());
  iterationISE2_.push_back((Eigen::VectorXd(1) << nominalConstraint2ISE_).finished());

  // convergence conditions variables
  scalar_t relCost;
  scalar_t relConstraint1ISE;
  bool isLearningRateStarZero = false;
  bool isCostFunctionConverged = false;
  bool isConstraint1Satisfied = false;
  bool isOptimizationConverged = false;

  // DDP main loop
  while (iteration_ + 1 < ddpSettings_.maxNumIterations_ && !isOptimizationConverged) {
    // increment iteration counter
    iteration_++;

    scalar_t costCashed = nominalTotalCost_;
    scalar_t constraint1ISECashed = nominalConstraint1ISE_;

    // display
    if (ddpSettings_.displayInfo_) {
      std::cerr << "\n#### Iteration " << iteration_ << std::endl;
    }

    // run the an iteration of the DDP algorithm and update the member variables
    runIteration();

    iterationCost_.push_back((Eigen::VectorXd(1) << nominalTotalCost_).finished());
    iterationISE1_.push_back((Eigen::VectorXd(1) << nominalConstraint1ISE_).finished());
    iterationISE2_.push_back((Eigen::VectorXd(1) << nominalConstraint2ISE_).finished());

    // loop break variables
    relCost = std::abs(nominalTotalCost_ - costCashed);
    relConstraint1ISE = std::abs(nominalConstraint1ISE_ - constraint1ISECashed);
    isConstraint1Satisfied =
        nominalConstraint1ISE_ <= ddpSettings_.minAbsConstraint1ISE_ || relConstraint1ISE <= ddpSettings_.minRelConstraint1ISE_;
    isLearningRateStarZero = learningRateStar_ == 0 && !isInitInternalControllerEmpty;
    isCostFunctionConverged = relCost <= ddpSettings_.minRelCost_ || isLearningRateStarZero;
    isOptimizationConverged = isCostFunctionConverged && isConstraint1Satisfied;
    isInitInternalControllerEmpty = false;

  }  // end of while loop

  if (ddpSettings_.displayInfo_) {
    std::cerr << "\n#### Final rollout" << std::endl;
  }

#ifdef BENCHMARK
  BENCHMARK_nIterationsFP_++;
  BENCHMARK_start_ = std::chrono::steady_clock::now();
#endif

  bool computeISEs = !ddpSettings_.noStateConstraints_ || ddpSettings_.displayInfo_ || ddpSettings_.displayShortSummary_;

  // finding the final optimal learningRate and getting the optimal trajectories and controller
  maxLearningRate_ = ddpSettings_.maxLearningRate_;
  lineSearch(computeISEs);

#ifdef BENCHMARK
  BENCHMARK_end_ = std::chrono::steady_clock::now();
  BENCHMARK_diff_ = BENCHMARK_end_ - BENCHMARK_start_;
  BENCHMARK_tAvgFP_ = ((1.0 - 1.0 / BENCHMARK_nIterationsFP_) * BENCHMARK_tAvgFP_) +
                      (1.0 / BENCHMARK_nIterationsFP_) * std::chrono::duration_cast<std::chrono::microseconds>(BENCHMARK_diff_).count();
#endif

  updateNominalControllerPtrStock();

  /*
   * adds the deleted controller parts
   */
  runExit();

  // display
  if (ddpSettings_.displayInfo_ || ddpSettings_.displayShortSummary_) {
    std::cerr << "\n++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
    std::cerr << "++++++++++++++ " + algorithmName_ + " solver is terminated ++++++++++++++" << std::endl;
    std::cerr << "++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
    std::cerr << "Number of Iterations:      " << iteration_ + 1 << " out of " << ddpSettings_.maxNumIterations_ << std::endl;

    printRolloutInfo();

    if (isOptimizationConverged) {
      if (isLearningRateStarZero) {
        std::cerr << algorithmName_ + " successfully terminates as learningRate reduced to zero." << std::endl;
      } else {
        std::cerr << algorithmName_ + " successfully terminates as cost relative change (relCost=" << relCost
                  << ") reached to the minimum value." << std::endl;
      }

      if (nominalConstraint1ISE_ <= ddpSettings_.minAbsConstraint1ISE_) {
        std::cerr << "Type-1 constraint absolute ISE (absConstraint1ISE=" << nominalConstraint1ISE_ << ") reached to the minimum value."
                  << std::endl;
      } else {
        std::cerr << "Type-1 constraint relative ISE (relConstraint1ISE=" << relConstraint1ISE << ") reached to the minimum value."
                  << std::endl;
      }
    } else {
      std::cerr << "Maximum number of iterations has reached." << std::endl;
    }
    std::cerr << std::endl;
  }
}

}  // namespace ocs2
