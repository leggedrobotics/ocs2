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

#include <ocs2_ddp/DDP_BASE.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
DDP_BASE<STATE_DIM, INPUT_DIM>::DDP_BASE(const rollout_base_t* rolloutPtr, const derivatives_base_t* systemDerivativesPtr,
                                         const constraint_base_t* systemConstraintsPtr, const cost_function_base_t* costFunctionPtr,
                                         const operating_trajectories_base_t* operatingTrajectoriesPtr, const DDP_Settings& ddpSettings,
                                         const cost_function_base_t* heuristicsFunctionPtr, const char* algorithmName)
    : ddpSettings_(ddpSettings),
      threadPool_(ddpSettings.nThreads_, ddpSettings.threadPriority_),
      algorithmName_(algorithmName),
      rewindCounter_(0),
      iteration_(0),
      learningRateStar_(1.0) {
  // Dynamics, Constraints, derivatives, and cost
  linearQuadraticApproximatorPtrStock_.clear();
  linearQuadraticApproximatorPtrStock_.reserve(ddpSettings_.nThreads_);
  heuristicsFunctionsPtrStock_.clear();
  heuristicsFunctionsPtrStock_.reserve(ddpSettings_.nThreads_);
  penaltyPtrStock_.clear();
  penaltyPtrStock_.reserve(ddpSettings_.nThreads_);
  dynamicsForwardRolloutPtrStock_.clear();
  dynamicsForwardRolloutPtrStock_.reserve(ddpSettings_.nThreads_);
  operatingTrajectoriesRolloutPtrStock_.clear();
  operatingTrajectoriesRolloutPtrStock_.reserve(ddpSettings_.nThreads_);

  // initialize all subsystems, etc.
  for (size_t i = 0; i < ddpSettings_.nThreads_; i++) {
    // initialize rollout
    dynamicsForwardRolloutPtrStock_.emplace_back(rolloutPtr->clone());

    // initialize operating points
    operatingTrajectoriesRolloutPtrStock_.emplace_back(
        new operating_trajectorie_rollout_t(*operatingTrajectoriesPtr, rolloutPtr->settings()));

    // initialize LQ approximator
    linearQuadraticApproximatorPtrStock_.emplace_back(
        new linear_quadratic_approximator_t(*systemDerivativesPtr, *systemConstraintsPtr, *costFunctionPtr, algorithmName_.c_str(),
                                            ddpSettings_.checkNumericalStability_, ddpSettings_.useMakePSD_));

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
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
DDP_BASE<STATE_DIM, INPUT_DIM>::~DDP_BASE() {
  auto forwardPassTotal = forwardPassTimer_.getTotalInMilliseconds();
  auto linearQuadraticApproximationTotal = linearQuadraticApproximationTimer_.getTotalInMilliseconds();
  auto backwardPassTotal = backwardPassTimer_.getTotalInMilliseconds();
  auto computeControllerTotal = computeControllerTimer_.getTotalInMilliseconds();
  auto finalRolloutTotal = linesearchTimer_.getTotalInMilliseconds();

  auto benchmarkTotal =
      forwardPassTotal + linearQuadraticApproximationTotal + backwardPassTotal + computeControllerTotal + finalRolloutTotal;

  if (benchmarkTotal > 0 && (ddpSettings_.displayInfo_ || ddpSettings_.displayShortSummary_)) {
    std::cerr << "\n########################################################################\n";
    std::cerr << "Benchmarking\t           :\tAverage time [ms]   (% of total runtime)\n";
    std::cerr << "\tForward Pass       :\t" << forwardPassTimer_.getAverageInMilliseconds() << " [ms] \t\t("
              << forwardPassTotal / benchmarkTotal * 100 << "%)\n";
    std::cerr << "\tLQ Approximation   :\t" << linearQuadraticApproximationTimer_.getAverageInMilliseconds() << " [ms] \t\t("
              << linearQuadraticApproximationTotal / benchmarkTotal * 100 << "%)\n";
    std::cerr << "\tBackward Pass      :\t" << backwardPassTimer_.getAverageInMilliseconds() << " [ms] \t\t("
              << backwardPassTotal / benchmarkTotal * 100 << "%)\n";
    std::cerr << "\tCompute Controller :\t" << computeControllerTimer_.getAverageInMilliseconds() << " [ms] \t\t("
              << computeControllerTotal / benchmarkTotal * 100 << "%)\n";
    std::cerr << "\tLinesearch         :\t" << linesearchTimer_.getAverageInMilliseconds() << " [ms] \t\t("
              << finalRolloutTotal / benchmarkTotal * 100 << "%)" << std::endl;
  }
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

  useParallelRiccatiSolverFromInitItr_ = false;

  performanceIndexHistory_.clear();

  for (size_t i = 0; i < numPartitions_; i++) {
    // very important, these are variables that are carried in between iterations
    nominalControllersStock_[i].clear();
    nominalTimeTrajectoriesStock_[i].clear();
    nominalPostEventIndicesStock_[i].clear();
    nominalStateTrajectoriesStock_[i].clear();
    nominalInputTrajectoriesStock_[i].clear();

    cachedTimeTrajectoriesStock_[i].clear();
    cachedPostEventIndicesStock_[i].clear();
    cachedStateTrajectoriesStock_[i].clear();
    cachedInputTrajectoriesStock_[i].clear();

    // for Riccati equation parallel computation
    SmFinalStock_[i] = state_matrix_t::Zero();
    SvFinalStock_[i] = state_vector_t::Zero();
    SveFinalStock_[i] = state_vector_t::Zero();
    sFinalStock_[i] = eigen_scalar_t::Zero();
    xFinalStock_[i] = state_vector_t::Zero();
  }  // end of i loop

  // reset timers
  forwardPassTimer_.reset();
  linearQuadraticApproximationTimer_.reset();
  backwardPassTimer_.reset();
  computeControllerTimer_.reset();
  linesearchTimer_.reset();
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
typename DDP_BASE<STATE_DIM, INPUT_DIM>::scalar_t DDP_BASE<STATE_DIM, INPUT_DIM>::rolloutTrajectory(
    linear_controller_array_t& controllersStock, scalar_array2_t& timeTrajectoriesStock, size_array2_t& postEventIndicesStock,
    state_vector_array2_t& stateTrajectoriesStock, input_vector_array2_t& inputTrajectoriesStock, size_t threadId /*= 0*/) {
  const scalar_array_t& eventTimes = this->getModeSchedule().eventTimes;

  if (controllersStock.size() != numPartitions_) {
    throw std::runtime_error("controllersStock has less controllers then the number of subsystems");
  }

  // Prepare outputs
  timeTrajectoriesStock.resize(numPartitions_);
  postEventIndicesStock.resize(numPartitions_);
  stateTrajectoriesStock.resize(numPartitions_);
  inputTrajectoriesStock.resize(numPartitions_);
  for (size_t i = 0; i < numPartitions_; i++) {
    timeTrajectoriesStock[i].clear();
    postEventIndicesStock[i].clear();
    stateTrajectoriesStock[i].clear();
    inputTrajectoriesStock[i].clear();
  }

  // Find until where we have a controller available for the rollout
  scalar_t controllerAvailableTill = initTime_;
  size_t partitionOfLastController = initActivePartition_;
  for (size_t i = initActivePartition_; i < finalActivePartition_ + 1; i++) {
    if (!controllersStock[i].empty()) {
      controllerAvailableTill = controllersStock[i].timeStamp_.back();
      partitionOfLastController = i;
    } else {
      break;  // break on the first empty controller (cannot have gaps in the controllers)
    }
  }

  /*
   * Define till where we use the controller
   * - If the first controller is empty, don't use a controller at all
   * - If we have a controller and no events, use the controller till the final time
   * - Otherwise, use the controller until the first event time after the controller has reached it's end.
   */
  scalar_t useControllerTill = initTime_;
  if (!controllersStock[initActivePartition_].empty()) {
    useControllerTill = finalTime_;
    for (const auto eventTime : eventTimes) {
      if (eventTime >= controllerAvailableTill) {
        useControllerTill = std::min(eventTime, finalTime_);
        break;
      }
    }
  }

  if (ddpSettings_.debugPrintRollout_) {
    std::cerr << "[DDP_BASE::rolloutTrajectory] for t = [" << initTime_ << ", " << finalTime_ << "]\n"
              << "\tcontroller available till t = " << controllerAvailableTill << "\n"
              << "\twill use controller until t = " << useControllerTill << std::endl;
  }

  size_t numSteps = 0;
  state_vector_t xCurrent = initState_;
  for (size_t i = initActivePartition_; i < finalActivePartition_ + 1; i++) {
    // Start and end of rollout segment
    const scalar_t t0 = (i == initActivePartition_) ? initTime_ : partitioningTimes_[i];
    const scalar_t tf = (i == finalActivePartition_) ? finalTime_ : partitioningTimes_[i + 1];

    // Divide the rollout segment in controller rollout and operating points
    const std::pair<scalar_t, scalar_t> controllerRolloutFromTo{t0, std::max(t0, std::min(useControllerTill, tf))};
    std::pair<scalar_t, scalar_t> operatingPointsFromTo{controllerRolloutFromTo.second, tf};

    if (ddpSettings_.debugPrintRollout_) {
      std::cerr << "[DDP_BASE::rolloutTrajectory] partition " << i << " for t = [" << t0 << ", " << tf << "]" << std::endl;
      if (controllerRolloutFromTo.first < controllerRolloutFromTo.second) {
        std::cerr << "\twill use controller for t = [" << controllerRolloutFromTo.first << ", " << controllerRolloutFromTo.second << "]"
                  << std::endl;
      }
      if (operatingPointsFromTo.first < operatingPointsFromTo.second) {
        std::cerr << "\twill use operating points for t = [" << operatingPointsFromTo.first << ", " << operatingPointsFromTo.second << "]"
                  << std::endl;
      }
    }

    // Rollout with controller
    if (controllerRolloutFromTo.first < controllerRolloutFromTo.second) {
      auto controllerPtr = &controllersStock[std::min(i, partitionOfLastController)];
      xCurrent = dynamicsForwardRolloutPtrStock_[threadId]->run(
          controllerRolloutFromTo.first, xCurrent, controllerRolloutFromTo.second, controllerPtr, eventTimes, timeTrajectoriesStock[i],
          postEventIndicesStock[i], stateTrajectoriesStock[i], inputTrajectoriesStock[i]);
    }

    // Finish rollout with operating points
    if (operatingPointsFromTo.first < operatingPointsFromTo.second) {
      // Remove last point of the controller rollout if it is directly past an event. Here where we want to use the operating point
      // instead. However, we do start the integration at the state after the event. i.e. the jump map remains applied.
      if (!postEventIndicesStock[i].empty() && postEventIndicesStock[i].back() == (timeTrajectoriesStock[i].size() - 1)) {
        // Start new integration at the time point after the event to remain consistent with added epsilons in the rollout. The operating
        // point rollout does not add this epsilon because it does not know about this event.
        operatingPointsFromTo.first = timeTrajectoriesStock[i].back();
        timeTrajectoriesStock[i].pop_back();
        stateTrajectoriesStock[i].pop_back();
        inputTrajectoriesStock[i].pop_back();
        // eventsPastTheEndIndeces is not removed because we need to mark the start of the operatingPointTrajectory as being after an event.
      }

      scalar_array_t timeTrajectoryTail;
      size_array_t eventsPastTheEndIndecesTail;
      state_vector_array_t stateTrajectoryTail;
      input_vector_array_t inputTrajectoryTail;
      xCurrent = operatingTrajectoriesRolloutPtrStock_[threadId]->run(operatingPointsFromTo.first, xCurrent, operatingPointsFromTo.second,
                                                                      nullptr, eventTimes, timeTrajectoryTail, eventsPastTheEndIndecesTail,
                                                                      stateTrajectoryTail, inputTrajectoryTail);

      // Add controller rollout length to event past the indeces
      for (auto& eventIndex : eventsPastTheEndIndecesTail) {
        eventIndex += stateTrajectoriesStock[i].size();  // This size of this trajectory part was missing when counting events in the tail
      }

      // Concatenate the operating points to the rollout
      timeTrajectoriesStock[i].insert(timeTrajectoriesStock[i].end(), timeTrajectoryTail.begin(), timeTrajectoryTail.end());
      postEventIndicesStock[i].insert(postEventIndicesStock[i].end(), eventsPastTheEndIndecesTail.begin(),
                                      eventsPastTheEndIndecesTail.end());
      stateTrajectoriesStock[i].insert(stateTrajectoriesStock[i].end(), stateTrajectoryTail.begin(), stateTrajectoryTail.end());
      inputTrajectoriesStock[i].insert(inputTrajectoriesStock[i].end(), inputTrajectoryTail.begin(), inputTrajectoryTail.end());
    }

    // total number of steps
    numSteps += timeTrajectoriesStock[i].size();
  }  // end of i loop

  if (!xCurrent.allFinite()) {
    throw std::runtime_error("System became unstable during the rollout.");
  }

  // debug print
  if (ddpSettings_.debugPrintRollout_) {
    for (size_t i = 0; i < numPartitions_; i++) {
      std::cerr << std::endl << "++++++++++++++++++++++++++++++" << std::endl;
      std::cerr << "Partition: " << i;
      std::cerr << std::endl << "++++++++++++++++++++++++++++++" << std::endl;
      rollout_base_t::display(timeTrajectoriesStock[i], postEventIndicesStock[i], stateTrajectoriesStock[i], &inputTrajectoriesStock[i]);
    }
  }

  // average time step
  return (finalTime_ - initTime_) / numSteps;
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
void DDP_BASE<STATE_DIM, INPUT_DIM>::calculateConstraintsWorker(
    size_t workerIndex, size_t partitionIndex, const scalar_array_t& timeTrajectory, const size_array_t& eventsPastTheEndIndeces,
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
    const scalar_array2_t& timeTrajectoriesStock, const size_array2_t& postEventIndicesStock,
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
    calculateConstraintsWorker(threadId, i, timeTrajectoriesStock[i], postEventIndicesStock[i], stateTrajectoriesStock[i],
                               inputTrajectoriesStock[i], nc1TrajectoriesStock[i], EvTrajectoryStock[i], nc2TrajectoriesStock[i],
                               HvTrajectoryStock[i], ncIneqTrajectoriesStock[i], hTrajectoryStock[i], nc2FinalStock[i], HvFinalStock[i]);
  }  // end of i loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
auto DDP_BASE<STATE_DIM, INPUT_DIM>::calculateCostWorker(size_t workerIndex, size_t partitionIndex, const scalar_array_t& timeTrajectory,
                                                         const size_array_t& eventsPastTheEndIndeces,
                                                         const state_vector_array_t& stateTrajectory,
                                                         const input_vector_array_t& inputTrajectory) -> scalar_t {
  cost_function_base_t& costFunction = linearQuadraticApproximatorPtrStock_[workerIndex]->costFunction();

  // set desired trajectories
  costFunction.setCostDesiredTrajectoriesPtr(&this->getCostDesiredTrajectories());

  scalar_t totalCost = 0.0;
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

  return totalCost;
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
auto DDP_BASE<STATE_DIM, INPUT_DIM>::calculateRolloutCost(const scalar_array2_t& timeTrajectoriesStock,
                                                          const size_array2_t& postEventIndicesStock,
                                                          const state_vector_array2_t& stateTrajectoriesStock,
                                                          const input_vector_array2_t& inputTrajectoriesStock, size_t threadId)
    -> scalar_t {
  scalar_t totalCost = 0.0;
  for (size_t i = 0; i < numPartitions_; i++) {
    totalCost += calculateCostWorker(threadId, i, timeTrajectoriesStock[i], postEventIndicesStock[i], stateTrajectoriesStock[i],
                                     inputTrajectoriesStock[i]);
  }  // end of i loop

  // calculate the Heuristics function at the final time
  // set desired trajectories
  heuristicsFunctionsPtrStock_[threadId]->setCostDesiredTrajectoriesPtr(&this->getCostDesiredTrajectories());
  // set state-input
  heuristicsFunctionsPtrStock_[threadId]->setCurrentStateAndControl(timeTrajectoriesStock[finalActivePartition_].back(),
                                                                    stateTrajectoriesStock[finalActivePartition_].back(),
                                                                    inputTrajectoriesStock[finalActivePartition_].back());
  // compute
  scalar_t sHeuristics;
  heuristicsFunctionsPtrStock_[threadId]->getTerminalCost(sHeuristics);
  totalCost += sHeuristics;

  return totalCost;
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
void DDP_BASE<STATE_DIM, INPUT_DIM>::calculateRolloutMerit(const size_array2_t& nc2FinalStock,
                                                           const constraint2_vector_array2_t& HvFinalStock,
                                                           PerformanceIndex& performanceIndex, size_t threadId /*= 0*/) const {
  const scalar_t stateConstraintPenalty =
      ddpSettings_.stateConstraintPenaltyCoeff_ * std::pow(ddpSettings_.stateConstraintPenaltyBase_, iteration_);

  // cost
  performanceIndex.merit = performanceIndex.totalCost;

  // ISE of type-2 constraint
  performanceIndex.merit += 0.5 * stateConstraintPenalty * performanceIndex.stateEqConstraintISE;

  // Inequality constraints
  performanceIndex.merit += performanceIndex.inequalityConstraintPenalty;

  // final constraint type 2
  performanceIndex.stateEqFinalConstraintSSE = 0.0;
  for (size_t i = 0; i < numPartitions_; i++) {
    for (size_t k = 0; k < nc2FinalStock[i].size(); k++) {
      auto nc2Final = nc2FinalStock[i][k];
      performanceIndex.stateEqFinalConstraintSSE += HvFinalStock[i][k].head(nc2Final).squaredNorm();
    }  // end of k loop
  }    // end of i loop

  performanceIndex.merit += 0.5 * stateConstraintPenalty * performanceIndex.stateEqFinalConstraintSSE;
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
void DDP_BASE<STATE_DIM, INPUT_DIM>::approximateOptimalControlProblem() {
  for (size_t i = 0; i < numPartitions_; i++) {
    // number of the intermediate LQ variables
    auto N = nominalTimeTrajectoriesStock_[i].size();

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
    size_t NE = nominalPostEventIndicesStock_[i].size();

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
        linearQuadraticApproximatorPtrStock_[j]->costFunction().setCostDesiredTrajectoriesPtr(&this->getCostDesiredTrajectories());
      }  // end of j loop

      // perform the approximateLQWorker for partition i
      nextTimeIndex_ = 0;
      nextTaskId_ = 0;
      std::function<void(void)> task = [this, i] {
        int N = nominalTimeTrajectoriesStock_[i].size();
        int timeIndex;
        size_t taskId = nextTaskId_++;  // assign task ID (atomic)

        // get next time index is atomic
        while ((timeIndex = nextTimeIndex_++) < N) {
          // execute approximateLQ for the given partition and time node index
          approximateLQWorker(taskId, i, timeIndex);
        }
      };
      runParallel(task, ddpSettings_.nThreads_);
    }

  }  // end of i loop

  // calculate the Heuristics function at the final time
  heuristicsFunctionsPtrStock_[0]->setCostDesiredTrajectoriesPtr(&this->getCostDesiredTrajectories());
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
void DDP_BASE<STATE_DIM, INPUT_DIM>::approximateUnconstrainedLQWorker(size_t workerIndex, size_t i, size_t k) {
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
  } else {
    QmTrajectoryStock_[i][k].diagonal().array() += ddpSettings_.addedRiccatiDiagonal_;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
void DDP_BASE<STATE_DIM, INPUT_DIM>::approximateEventsLQWorker(size_t workerIndex, size_t i, size_t k, scalar_t stateConstraintPenalty) {
  // if a switch took place calculate switch related variables
  size_t NE = nominalPostEventIndicesStock_[i].size();
  for (size_t ke = 0; ke < NE; ke++) {
    if (nominalPostEventIndicesStock_[i][ke] == k + 1) {
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
      size_t nc2 = nc2FinalStock_[i][ke];
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
void DDP_BASE<STATE_DIM, INPUT_DIM>::calculateController() {
  for (size_t i = 0; i < numPartitions_; i++) {
    if (i < initActivePartition_ || i > finalActivePartition_) {
      nominalControllersStock_[i].clear();
      continue;
    }

    const auto N = SsTimeTrajectoryStock_[i].size();

    nominalControllersStock_[i].timeStamp_ = SsTimeTrajectoryStock_[i];
    nominalControllersStock_[i].gainArray_.resize(N);
    nominalControllersStock_[i].biasArray_.resize(N);
    nominalControllersStock_[i].deltaBiasArray_.resize(N);

    // if the partition is not active
    if (N == 0) {
      continue;
    }

    // perform the calculateControllerWorker for partition i
    nextTimeIndex_ = 0;
    nextTaskId_ = 0;
    std::function<void(void)> task = [this, i] {
      int N = SsTimeTrajectoryStock_[i].size();
      int timeIndex;
      size_t taskId = nextTaskId_++;  // assign task ID (atomic)

      // get next time index (atomic)
      while ((timeIndex = nextTimeIndex_++) < N) {
        calculateControllerWorker(taskId, i, timeIndex);
      }
    };
    runParallel(task, ddpSettings_.nThreads_);

  }  // end of i loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void DDP_BASE<STATE_DIM, INPUT_DIM>::lineSearch() {
  // perform one rollout while the input correction for the type-1 constraint is considered.
  baselineRollout();

  baselineMerit_ = performanceIndex_.merit;
  learningRateStar_ = 0.0;                             // input correction learning rate is zero
  initLScontrollersStock_ = nominalControllersStock_;  // this will serve to init the workers

  // if no line search
  if (ddpSettings_.maxLearningRate_ < OCS2NumericTraits<scalar_t>::limitEpsilon()) {
    // clear the feedforward increments
    for (size_t i = 0; i < numPartitions_; i++) {
      nominalControllersStock_[i].deltaBiasArray_.clear();
    }
    // display
    if (ddpSettings_.displayInfo_) {
      std::cerr << "The chosen learningRate is: " << learningRateStar_ << std::endl;
    }

    return;
  }

  const auto maxNumOfLineSearches = static_cast<size_t>(
      std::log(ddpSettings_.minLearningRate_ / ddpSettings_.maxLearningRate_) / std::log(ddpSettings_.lineSearchContractionRate_) + 1);

  alphaExpNext_ = 0;
  alphaProcessed_ = std::vector<bool>(maxNumOfLineSearches, false);

  nextTaskId_ = 0;
  std::function<void(void)> task = [this] { lineSearchTask(); };
  runParallel(task, ddpSettings_.nThreads_);

  // revitalize all integrators
  event_handler_t::deactivateKillIntegration();

  // clear the feedforward increments
  for (size_t i = 0; i < numPartitions_; i++) {
    nominalControllersStock_[i].deltaBiasArray_.clear();
  }

  // display
  if (ddpSettings_.displayInfo_) {
    std::cerr << "The chosen learningRate is: " + std::to_string(learningRateStar_) << std::endl;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
void DDP_BASE<STATE_DIM, INPUT_DIM>::baselineRollout() {
  // perform one rollout while the input correction for the type-1 constraint is considered.
  avgTimeStepFP_ = rolloutTrajectory(nominalControllersStock_, nominalTimeTrajectoriesStock_, nominalPostEventIndicesStock_,
                                     nominalStateTrajectoriesStock_, nominalInputTrajectoriesStock_);

  // calculate constraint
  calculateRolloutConstraints(nominalTimeTrajectoriesStock_, nominalPostEventIndicesStock_, nominalStateTrajectoriesStock_,
                              nominalInputTrajectoriesStock_, nc1TrajectoriesStock_, EvTrajectoryStock_, nc2TrajectoriesStock_,
                              HvTrajectoryStock_, ncIneqTrajectoriesStock_, hTrajectoryStock_, nc2FinalStock_, HvFinalStock_);
  // calculate constraint type-1 ISE and maximum norm
  nominalConstraint1MaxNorm_ = calculateConstraintISE(nominalTimeTrajectoriesStock_, nc1TrajectoriesStock_, EvTrajectoryStock_,
                                                      performanceIndex_.stateInputEqConstraintISE);
  // calculates type-2 constraint ISE and maximum norm
  nominalConstraint2MaxNorm_ = calculateConstraintISE(nominalTimeTrajectoriesStock_, nc2TrajectoriesStock_, HvTrajectoryStock_,
                                                      performanceIndex_.stateEqConstraintISE);
  // calculate penalty
  performanceIndex_.inequalityConstraintPenalty = calculateInequalityConstraintPenalty(
      nominalTimeTrajectoriesStock_, ncIneqTrajectoriesStock_, hTrajectoryStock_, performanceIndex_.inequalityConstraintISE);

  // calculates cost
  performanceIndex_.totalCost = calculateRolloutCost(nominalTimeTrajectoriesStock_, nominalPostEventIndicesStock_,
                                                     nominalStateTrajectoriesStock_, nominalInputTrajectoriesStock_);
  // calculates merit
  calculateRolloutMerit(nc2FinalStock_, HvFinalStock_, performanceIndex_);

  // display
  if (ddpSettings_.displayInfo_) {
    std::stringstream linesearchDisplay;
    linesearchDisplay << "    [Thread 0] - learningRate 0.0" << '\n';
    linesearchDisplay << std::setw(4) << performanceIndex_ << '\n';
    linesearchDisplay << "    final state constraints norm: ";
    for (size_t i = initActivePartition_; i <= finalActivePartition_; i++) {
      for (size_t k = 0; k < nc2FinalStock_[i].size(); k++) {
        linesearchDisplay << HvFinalStock_[i][k].head(nc2FinalStock_[i][k]).norm() << ",  ";
      }
    }
    linesearchDisplay << "\n    forward pass average time step: " << avgTimeStepFP_ * 1e+3 << " [ms].\n";
    BASE::printString(linesearchDisplay.str());
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void DDP_BASE<STATE_DIM, INPUT_DIM>::lineSearchTask() {
  size_t taskId = nextTaskId_++;  // assign task ID (atomic)

  // local search forward simulation's variables
  PerformanceIndex lsPerformanceIndex;
  scalar_t lsConstraint1MaxNorm, lsConstraint2MaxNorm;
  linear_controller_array_t lsControllersStock(numPartitions_);
  scalar_array2_t lsTimeTrajectoriesStock(numPartitions_);
  size_array2_t lsPostEventIndicesStock(numPartitions_);
  state_vector_array2_t lsStateTrajectoriesStock(numPartitions_);
  input_vector_array2_t lsInputTrajectoriesStock(numPartitions_);

  while (true) {
    size_t alphaExp = alphaExpNext_++;
    scalar_t learningRate = maxLearningRate_ * std::pow(ddpSettings_.lineSearchContractionRate_, alphaExp);

    /*
     * finish this thread's task since the learning rate is less than the minimum learning rate.
     * This means that the all the line search tasks are already processed or they are under
     * process in other threads.
     */
    if (!numerics::almost_ge(learningRate, ddpSettings_.minLearningRate_)) {
      break;
    }

    // skip if the current learning rate is less than the best candidate
    if (learningRate < learningRateStar_) {
      // display
      if (ddpSettings_.displayInfo_) {
        std::string linesearchDisplay;
        linesearchDisplay = "    [Thread " + std::to_string(taskId) + "] rollout with learningRate " + std::to_string(learningRate) +
                            " is skipped: A larger learning rate is already found!";
        BASE::printString(linesearchDisplay);
      }
      break;
    }

    // do a line search
    lsControllersStock = initLScontrollersStock_;
    lineSearchWorker(taskId, learningRate, lsPerformanceIndex, lsConstraint1MaxNorm, lsConstraint2MaxNorm, lsControllersStock,
                     lsTimeTrajectoriesStock, lsPostEventIndicesStock, lsStateTrajectoriesStock, lsInputTrajectoriesStock);

    bool terminateLinesearchTasks = false;
    {
      std::lock_guard<std::mutex> lock(lineSearchResultMutex_);

      /*
       * based on the "greedy learning rate selection" policy:
       * cost should be better than the baseline cost but learning rate should
       * be as high as possible. This is equivalent to a single core line search.
       */
      if (lsPerformanceIndex.merit < (baselineMerit_ * (1 - 1e-3 * learningRate)) && learningRate > learningRateStar_) {
        learningRateStar_ = learningRate;
        nominalConstraint1MaxNorm_ = lsConstraint1MaxNorm;
        nominalConstraint2MaxNorm_ = lsConstraint2MaxNorm;
        performanceIndex_ = lsPerformanceIndex;

        nominalControllersStock_.swap(lsControllersStock);
        nominalTimeTrajectoriesStock_.swap(lsTimeTrajectoriesStock);
        nominalPostEventIndicesStock_.swap(lsPostEventIndicesStock);
        nominalStateTrajectoriesStock_.swap(lsStateTrajectoriesStock);
        nominalInputTrajectoriesStock_.swap(lsInputTrajectoriesStock);

        // whether to stop all other thread.
        terminateLinesearchTasks = true;
        for (size_t i = 0; i < alphaExp; i++) {
          if (!alphaProcessed_[i]) {
            terminateLinesearchTasks = false;
            break;
          }
        }  // end of i loop

      }  // end of if

      alphaProcessed_[alphaExp] = true;

    }  // end lock

    // kill other ongoing line search tasks
    if (terminateLinesearchTasks) {
      event_handler_t::activateKillIntegration();  // kill all integrators
      if (ddpSettings_.displayInfo_) {
        BASE::printString("    LS: interrupt other rollout's integrations.");
      }
      break;
    }

  }  // end of while loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
void DDP_BASE<STATE_DIM, INPUT_DIM>::lineSearchWorker(size_t workerIndex, scalar_t learningRate, PerformanceIndex& lsPerformanceIndex,
                                                      scalar_t& lsConstraint1MaxNorm, scalar_t& lsConstraint2MaxNorm,
                                                      linear_controller_array_t& lsControllersStock,
                                                      scalar_array2_t& lsTimeTrajectoriesStock, size_array2_t& lsPostEventIndicesStock,
                                                      state_vector_array2_t& lsStateTrajectoriesStock,
                                                      input_vector_array2_t& lsInputTrajectoriesStock) {
  // modifying uff by local increments
  for (size_t i = 0; i < numPartitions_; i++) {
    for (size_t k = 0; k < lsControllersStock[i].timeStamp_.size(); k++) {
      lsControllersStock[i].biasArray_[k] += learningRate * lsControllersStock[i].deltaBiasArray_[k];
    }
  }

  try {
    // perform a rollout
    scalar_t avgTimeStepFP = rolloutTrajectory(lsControllersStock, lsTimeTrajectoriesStock, lsPostEventIndicesStock,
                                               lsStateTrajectoriesStock, lsInputTrajectoriesStock, workerIndex);

    // calculate rollout constraints
    size_array2_t lsNc1TrajectoriesStock(numPartitions_);
    constraint1_vector_array2_t lsEvTrajectoryStock(numPartitions_);
    size_array2_t lsNc2TrajectoriesStock(numPartitions_);
    constraint2_vector_array2_t lsHvTrajectoryStock(numPartitions_);
    size_array2_t lsNcIneqTrajectoriesStock(numPartitions_);
    scalar_array3_t lshTrajectoryStock(numPartitions_);
    size_array2_t lsNc2FinalStock(numPartitions_);
    constraint2_vector_array2_t lsHvFinalStock(numPartitions_);

    // calculate rollout constraints
    calculateRolloutConstraints(lsTimeTrajectoriesStock, lsPostEventIndicesStock, lsStateTrajectoriesStock, lsInputTrajectoriesStock,
                                lsNc1TrajectoriesStock, lsEvTrajectoryStock, lsNc2TrajectoriesStock, lsHvTrajectoryStock,
                                lsNcIneqTrajectoriesStock, lshTrajectoryStock, lsNc2FinalStock, lsHvFinalStock, workerIndex);
    // calculate constraint type-1 ISE and maximum norm
    lsConstraint1MaxNorm = calculateConstraintISE(lsTimeTrajectoriesStock, lsNc1TrajectoriesStock, lsEvTrajectoryStock,
                                                  lsPerformanceIndex.stateInputEqConstraintISE);
    // calculates type-2 constraint ISE and maximum norm
    lsConstraint2MaxNorm = calculateConstraintISE(lsTimeTrajectoriesStock, lsNc2TrajectoriesStock, lsHvTrajectoryStock,
                                                  lsPerformanceIndex.stateEqConstraintISE);
    // inequalityConstraints
    lsPerformanceIndex.inequalityConstraintPenalty = calculateInequalityConstraintPenalty(
        lsTimeTrajectoriesStock, lsNcIneqTrajectoriesStock, lshTrajectoryStock, lsPerformanceIndex.inequalityConstraintISE, workerIndex);

    // calculate rollout cost
    lsPerformanceIndex.totalCost = calculateRolloutCost(lsTimeTrajectoriesStock, lsPostEventIndicesStock, lsStateTrajectoriesStock,
                                                        lsInputTrajectoriesStock, workerIndex);
    // calculates merit
    calculateRolloutMerit(lsNc2FinalStock, lsHvFinalStock, lsPerformanceIndex, workerIndex);

    // display
    if (ddpSettings_.displayInfo_) {
      std::stringstream linesearchDisplay;
      linesearchDisplay << "    [Thread " << workerIndex << "] - learningRate " << learningRate << '\n';
      linesearchDisplay << std::setw(4) << lsPerformanceIndex << '\n';
      linesearchDisplay << "    final state constraints norm: ";
      for (size_t i = initActivePartition_; i <= finalActivePartition_; i++) {
        for (size_t k = 0; k < lsNc2FinalStock[i].size(); k++) {
          linesearchDisplay << lsHvFinalStock[i][k].head(lsNc2FinalStock[i][k]).norm() << ",  ";
        }
      }
      linesearchDisplay << "\n    forward pass average time step: " << avgTimeStepFP * 1e+3 << " [ms].\n";
      BASE::printString(linesearchDisplay.str());
    }

  } catch (const std::exception& error) {
    lsPerformanceIndex.merit = std::numeric_limits<scalar_t>::max();
    lsPerformanceIndex.totalCost = std::numeric_limits<scalar_t>::max();
    if (ddpSettings_.displayInfo_) {
      BASE::printString("    [Thread " + std::to_string(workerIndex) + "] rollout with learningRate " + std::to_string(learningRate) +
                        " is terminated: " + error.what());
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
typename DDP_BASE<STATE_DIM, INPUT_DIM>::scalar_t DDP_BASE<STATE_DIM, INPUT_DIM>::solveSequentialRiccatiEquations(
    const state_matrix_t& SmFinal, const state_vector_t& SvFinal, const eigen_scalar_t& sFinal) {
  SmFinalStock_[finalActivePartition_] = SmFinal;
  SvFinalStock_[finalActivePartition_] = SvFinal;
  SveFinalStock_[finalActivePartition_].setZero();
  sFinalStock_[finalActivePartition_] = sFinal;

  // solve it sequentially for the first time when useParallelRiccatiSolverFromInitItr_ is false
  if (iteration_ == 0 && !useParallelRiccatiSolverFromInitItr_) {
    nextTaskId_ = 0;
    for (int i = 0; i < ddpSettings_.nThreads_; i++) {
      riccatiSolverTask();
    }
  }
  // solve it in parallel if useParallelRiccatiSolverFromInitItr_ is true
  else {
    nextTaskId_ = 0;
    std::function<void(void)> task = [this] { riccatiSolverTask(); };
    runParallel(task, ddpSettings_.nThreads_);
  }

  // total number of call
  size_t numSteps = 0;
  for (size_t i = initActivePartition_; i <= finalActivePartition_; i++) {
    numSteps += SsTimeTrajectoryStock_[i].size();
  }

  // average time step
  return (finalTime_ - initTime_) / numSteps;
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
void DDP_BASE<STATE_DIM, INPUT_DIM>::calculateMeritFunction(const scalar_array2_t& timeTrajectoriesStock,
                                                            const size_array2_t& nc1TrajectoriesStock,
                                                            const constraint1_vector_array2_t& EvTrajectoryStock,
                                                            const std::vector<std::vector<Eigen::VectorXd>>& lagrangeTrajectoriesStock,
                                                            scalar_t totalCost, scalar_t& meritFunctionValue, scalar_t& constraintISE) {
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
        size_t nc1 = nc1TrajectoriesStock[i][0];
        if (nc1 > 0) {
          currentSquaredNormError = EvTrajectoriesStock[i][0].head(nc1).squaredNorm();
        } else {
          currentSquaredNormError = 0.0;
        }
      } else {
        currentSquaredNormError = nextSquaredNormError;
      }

      maxConstraintNorm = ((maxConstraintNorm < currentSquaredNormError) ? currentSquaredNormError : maxConstraintNorm);

      size_t nc1 = nc1TrajectoriesStock[i][k + 1];
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
void DDP_BASE<STATE_DIM, INPUT_DIM>::calculateControllerUpdateMaxNorm(scalar_t& maxDeltaUffNorm, scalar_t& maxDeltaUeeNorm) {
  maxDeltaUffNorm = 0.0;
  maxDeltaUeeNorm = 0.0;
  for (size_t i = initActivePartition_; i <= finalActivePartition_; i++) {
    for (size_t k = 0; k < nominalControllersStock_[i].timeStamp_.size(); k++) {
      maxDeltaUffNorm = std::max(maxDeltaUffNorm, nominalControllersStock_[i].deltaBiasArray_[k].norm());

      const auto& time = nominalControllersStock_[i].timeStamp_[k];
      const auto indexAlpha = EigenLinearInterpolation<state_vector_t>::timeSegment(time, &(nominalTimeTrajectoriesStock_[i]));
      state_vector_t nominalState;
      EigenLinearInterpolation<state_vector_t>::interpolate(indexAlpha, nominalState, &(nominalStateTrajectoriesStock_[i]));
      input_vector_t nominalInput;
      EigenLinearInterpolation<input_vector_t>::interpolate(indexAlpha, nominalInput, &(nominalInputTrajectoriesStock_[i]));
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
void DDP_BASE<STATE_DIM, INPUT_DIM>::swapNominalTrajectoriesToCache() {
  cachedTimeTrajectoriesStock_.swap(nominalTimeTrajectoriesStock_);
  cachedPostEventIndicesStock_.swap(nominalPostEventIndicesStock_);
  cachedStateTrajectoriesStock_.swap(nominalStateTrajectoriesStock_);
  cachedInputTrajectoriesStock_.swap(nominalInputTrajectoriesStock_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
void DDP_BASE<STATE_DIM, INPUT_DIM>::correctInitcachedNominalTrajectories() {
  // for each partition
  for (size_t i = initActivePartition_; i <= finalActivePartition_; i++) {
    if (cachedTimeTrajectoriesStock_[i].empty()) {
      cachedPostEventIndicesStock_[i] = nominalPostEventIndicesStock_[i];
      cachedTimeTrajectoriesStock_[i] = nominalTimeTrajectoriesStock_[i];
      cachedStateTrajectoriesStock_[i] = nominalStateTrajectoriesStock_[i];
      cachedInputTrajectoriesStock_[i] = nominalInputTrajectoriesStock_[i];

    } else if (cachedTimeTrajectoriesStock_[i].back() < nominalTimeTrajectoriesStock_[i].back()) {
      // find the time segment
      const scalar_t finalTime = cachedTimeTrajectoriesStock_[i].back() + OCS2NumericTraits<scalar_t>::weakEpsilon();
      const auto timeSegment = LinearInterpolation<scalar_t>::timeSegment(finalTime, &nominalTimeTrajectoriesStock_[i]);

      // post event index
      const int sizeBeforeCorrection = cachedTimeTrajectoriesStock_[i].size();
      for (auto ind : nominalPostEventIndicesStock_[i]) {
        if (ind > timeSegment.first) {
          cachedPostEventIndicesStock_[i].push_back(ind - timeSegment.first + sizeBeforeCorrection);
        }
      }

      // time
      correctcachedTrajectoryTail(timeSegment, nominalTimeTrajectoriesStock_[i], cachedTimeTrajectoriesStock_[i]);
      // state
      correctcachedTrajectoryTail(timeSegment, nominalStateTrajectoriesStock_[i], cachedStateTrajectoriesStock_[i]);
      // input
      correctcachedTrajectoryTail(timeSegment, nominalInputTrajectoriesStock_[i], cachedInputTrajectoriesStock_[i]);

      // debugging checks for the added tail
      if (ddpSettings_.debugCaching_) {
        for (int k = timeSegment.first + 1; k < nominalTimeTrajectoriesStock_[i].size(); k++) {
          auto indexAlpha =
              LinearInterpolation<scalar_t>::timeSegment(nominalTimeTrajectoriesStock_[i][k], &cachedTimeTrajectoriesStock_[i]);

          state_vector_t stateCached;
          EigenLinearInterpolation<state_vector_t>::interpolate(indexAlpha, stateCached, &cachedStateTrajectoriesStock_[i]);
          if (!stateCached.isApprox(nominalStateTrajectoriesStock_[i][k])) {
            throw std::runtime_error("The tail of the cached state trajectory is not correctly set.");
          }

          input_vector_t inputCached;
          EigenLinearInterpolation<input_vector_t>::interpolate(indexAlpha, inputCached, &cachedInputTrajectoriesStock_[i]);
          if (!inputCached.isApprox(nominalInputTrajectoriesStock_[i][k])) {
            throw std::runtime_error("The tail of the cached input trajectory is not correctly set.");
          }
        }  // end of k loop
      }
    }

    // check for the event time indices
    if (ddpSettings_.debugCaching_) {
      auto postEvent = nominalPostEventIndicesStock_[i].rbegin();
      auto cachedPostEvent = cachedPostEventIndicesStock_[i].rbegin();
      for (; postEvent != nominalPostEventIndicesStock_[i].rend(); ++postEvent) {
        // nominal trajectory should have less event since it spans a shorter time period
        if (nominalTimeTrajectoriesStock_[i][*postEvent] != cachedTimeTrajectoriesStock_[i][*cachedPostEvent]) {
          throw std::runtime_error("Cached post event indexes are in correct.");
        }
        // check for the repeated time
        if (nominalTimeTrajectoriesStock_[i][*postEvent - 1] != cachedTimeTrajectoriesStock_[i][*cachedPostEvent - 1]) {
          throw std::runtime_error("Cached post event indexes are biased by -1.");
        }
        ++cachedPostEvent;
      }  // end of postEvent loop
    }

  }  // end of i loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
template <typename Data_T, class Alloc>
void DDP_BASE<STATE_DIM, INPUT_DIM>::correctcachedTrajectoryTail(std::pair<int, scalar_t> timeSegment,
                                                                 const std::vector<Data_T, Alloc>& currentTrajectory,
                                                                 std::vector<Data_T, Alloc>& cachedTrajectory) {
  // adding the fist cashed value
  Data_T firstCachedValue;
  LinearInterpolation<Data_T, Alloc>::interpolate(timeSegment, firstCachedValue, &currentTrajectory);
  cachedTrajectory.emplace_back(firstCachedValue);

  // Concatenate the rest
  const int ignoredSizeOfNominal = timeSegment.first + 1;

  cachedTrajectory.insert(cachedTrajectory.end(), currentTrajectory.begin() + ignoredSizeOfNominal, currentTrajectory.end());
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
void DDP_BASE<STATE_DIM, INPUT_DIM>::printRolloutInfo() {
  std::cerr << performanceIndex_ << '\n';
  std::cerr << "final constraint type-2:   ";
  size_t itr = 0;
  for (size_t i = initActivePartition_; i <= finalActivePartition_; i++) {
    for (size_t k = 0; k < nc2FinalStock_[i].size(); k++) {
      std::cerr << "[" << itr << "]: " << HvFinalStock_[i][k].head(nc2FinalStock_[i][k]).transpose() << ",  ";
      itr++;
    }
  }
  std::cerr << "\nforward pass average time step:  " << avgTimeStepFP_ * 1e+3 << " [ms].";
  std::cerr << "\nbackward pass average time step: " << avgTimeStepBP_ * 1e+3 << " [ms].\n";
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
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
typename DDP_BASE<STATE_DIM, INPUT_DIM>::scalar_t DDP_BASE<STATE_DIM, INPUT_DIM>::getValueFunction(scalar_t time,
                                                                                                   const state_vector_t& state) const {
  const auto partition = lookup::findBoundedActiveIntervalInTimeArray(partitioningTimes_, time);

  state_matrix_t Sm;
  const auto indexAlpha =
      EigenLinearInterpolation<state_matrix_t>::interpolate(time, Sm, &SsTimeTrajectoryStock_[partition], &SmTrajectoryStock_[partition]);

  state_vector_t Sv;
  EigenLinearInterpolation<state_vector_t>::interpolate(indexAlpha, Sv, &SvTrajectoryStock_[partition]);

  state_vector_t Sve;
  if (SveTrajectoryStock_[partition].empty()) {
    Sve.setZero();
  } else {
    EigenLinearInterpolation<state_vector_t>::interpolate(indexAlpha, Sve, &SveTrajectoryStock_[partition]);
  }

  eigen_scalar_t s;
  EigenLinearInterpolation<eigen_scalar_t>::interpolate(indexAlpha, s, &sTrajectoryStock_[partition]);

  state_vector_t xNominal;
  EigenLinearInterpolation<state_vector_t>::interpolate(time, xNominal, &nominalTimeTrajectoriesStock_[partition],
                                                        &nominalStateTrajectoriesStock_[partition]);

  state_vector_t deltaX = state - xNominal;

  return s(0) + deltaX.dot(Sv + Sve) + 0.5 * deltaX.dot(Sm * deltaX);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void DDP_BASE<STATE_DIM, INPUT_DIM>::getValueFunctionStateDerivative(scalar_t time, const state_vector_t& state, state_vector_t& Vx) const {
  const auto partition = lookup::findBoundedActiveIntervalInTimeArray(partitioningTimes_, time);

  state_matrix_t Sm;
  const auto indexAlpha =
      EigenLinearInterpolation<state_matrix_t>::interpolate(time, Sm, &SsTimeTrajectoryStock_[partition], &SmTrajectoryStock_[partition]);

  state_vector_t Sv;
  EigenLinearInterpolation<state_vector_t>::interpolate(indexAlpha, Sv, &SvTrajectoryStock_[partition]);

  state_vector_t Sve;
  if (SveTrajectoryStock_[partition].empty()) {
    Sve.setZero();
  } else {
    EigenLinearInterpolation<state_vector_t>::interpolate(indexAlpha, Sve, &SveTrajectoryStock_[partition]);
  }

  state_vector_t xNominal;
  EigenLinearInterpolation<state_vector_t>::interpolate(time, xNominal, &nominalTimeTrajectoriesStock_[partition],
                                                        &nominalStateTrajectoriesStock_[partition]);

  state_vector_t deltaX = state - xNominal;

  Vx = Sm * deltaX + Sv + Sve;
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
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
const PerformanceIndex& DDP_BASE<STATE_DIM, INPUT_DIM>::getPerformanceIndeces() const {
  return performanceIndex_;
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
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
const std::vector<PerformanceIndex>& DDP_BASE<STATE_DIM, INPUT_DIM>::getIterationsLog() const {
  return performanceIndexHistory_;
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
const DDP_Settings& DDP_BASE<STATE_DIM, INPUT_DIM>::ddpSettings() const {
  return ddpSettings_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
void DDP_BASE<STATE_DIM, INPUT_DIM>::getPrimalSolution(scalar_t finalTime, primal_solution_t* primalSolutionPtr) const {
  // total number of nodes
  int N = 0;
  for (const scalar_array_t& timeTrajectory_i : nominalTimeTrajectoriesStock_) {
    N += timeTrajectory_i.size();
  }

  auto upperBound = [](const scalar_array_t& array, scalar_t value) {
    auto firstLargerValueIterator = std::upper_bound(array.begin(), array.end(), value);
    return static_cast<int>(firstLargerValueIterator - array.begin());
  };

  // fill trajectories
  primalSolutionPtr->timeTrajectory_.clear();
  primalSolutionPtr->timeTrajectory_.reserve(N);
  primalSolutionPtr->stateTrajectory_.clear();
  primalSolutionPtr->stateTrajectory_.reserve(N);
  primalSolutionPtr->inputTrajectory_.clear();
  primalSolutionPtr->inputTrajectory_.reserve(N);
  for (size_t i = initActivePartition_; i <= finalActivePartition_; i++) {
    // break if the start time of the partition is greater than the final time
    if (nominalTimeTrajectoriesStock_[i].front() > finalTime) {
      break;
    }
    // length of the copy
    const int length = upperBound(nominalTimeTrajectoriesStock_[i], finalTime);

    primalSolutionPtr->timeTrajectory_.insert(primalSolutionPtr->timeTrajectory_.end(), nominalTimeTrajectoriesStock_[i].begin(),
                                              nominalTimeTrajectoriesStock_[i].begin() + length);
    primalSolutionPtr->stateTrajectory_.insert(primalSolutionPtr->stateTrajectory_.end(), nominalStateTrajectoriesStock_[i].begin(),
                                               nominalStateTrajectoriesStock_[i].begin() + length);
    primalSolutionPtr->inputTrajectory_.insert(primalSolutionPtr->inputTrajectory_.end(), nominalInputTrajectoriesStock_[i].begin(),
                                               nominalInputTrajectoriesStock_[i].begin() + length);
  }

  // fill controller
  if (ddpSettings_.useFeedbackPolicy_) {
    primalSolutionPtr->controllerPtr_.reset(new linear_controller_t);
    // concatenate controller stock into a single controller
    for (size_t i = initActivePartition_; i <= finalActivePartition_; i++) {
      // break if the start time of the partition is greater than the final time
      if (nominalControllersStock_[i].timeStamp_.front() > finalTime) {
        break;
      }
      // length of the copy
      const int length = upperBound(nominalControllersStock_[i].timeStamp_, finalTime);
      primalSolutionPtr->controllerPtr_->concatenate(&(nominalControllersStock_[i]), 0, length);
    }
  } else {
    primalSolutionPtr->controllerPtr_.reset(
        new feedforward_controller_t(primalSolutionPtr->timeTrajectory_, primalSolutionPtr->inputTrajectory_));
  }

  // fill mode schedule
  primalSolutionPtr->modeSchedule_ = this->getModeSchedule();
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
typename DDP_BASE<STATE_DIM, INPUT_DIM>::scalar_t DDP_BASE<STATE_DIM, INPUT_DIM>::getFinalTime() const {
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
void DDP_BASE<STATE_DIM, INPUT_DIM>::rewindOptimizer(size_t firstIndex) {
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
void DDP_BASE<STATE_DIM, INPUT_DIM>::distributeWork() {
  const int N = ddpSettings_.nThreads_;
  startingIndicesRiccatiWorker_.resize(N);
  endingIndicesRiccatiWorker_.resize(N);

  int subsystemsPerThread = (finalActivePartition_ - initActivePartition_ + 1) / N;
  int remainingSubsystems = (finalActivePartition_ - initActivePartition_ + 1) % N;

  int startingId, endingId = finalActivePartition_;
  for (size_t i = 0; i < N; i++) {
    endingIndicesRiccatiWorker_[i] = endingId;
    if (remainingSubsystems > 0) {
      startingId = endingId - subsystemsPerThread;
      remainingSubsystems--;
    } else {
      startingId = endingId - subsystemsPerThread + 1;
    }
    startingIndicesRiccatiWorker_[i] = startingId;
    endingId = startingId - 1;
  }

  // adding the inactive subsystems
  endingIndicesRiccatiWorker_.front() = numPartitions_ - 1;
  startingIndicesRiccatiWorker_.back() = 0;

  if (ddpSettings_.displayInfo_) {
    std::cerr << "Initial Active Subsystem: " << initActivePartition_ << std::endl;
    std::cerr << "Final Active Subsystem:   " << finalActivePartition_ << std::endl;
    std::cerr << "Backward path work distribution:" << std::endl;
    for (size_t i = 0; i < N; i++) {
      std::cerr << "start: " << startingIndicesRiccatiWorker_[i] << "\t";
      std::cerr << "end: " << endingIndicesRiccatiWorker_[i] << "\t";
      std::cerr << "num: " << endingIndicesRiccatiWorker_[i] - startingIndicesRiccatiWorker_[i] + 1 << std::endl;
    }
    std::cerr << std::endl;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
void DDP_BASE<STATE_DIM, INPUT_DIM>::runParallel(std::function<void(void)> taskFunction, size_t N) {
  threadPool_.runParallel([&](int) { taskFunction(); }, N);
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
void DDP_BASE<STATE_DIM, INPUT_DIM>::setupOptimizer(size_t numPartitions) {
  if (numPartitions == 0) {
    throw std::runtime_error("Number of partitions cannot be zero!");
  }

  /*
   * nominal trajectories
   */
  nominalControllersStock_.resize(numPartitions);

  nominalTimeTrajectoriesStock_.resize(numPartitions);
  nominalPostEventIndicesStock_.resize(numPartitions);
  nominalStateTrajectoriesStock_.resize(numPartitions);
  nominalInputTrajectoriesStock_.resize(numPartitions);

  cachedTimeTrajectoriesStock_.resize(numPartitions);
  cachedPostEventIndicesStock_.resize(numPartitions);
  cachedStateTrajectoriesStock_.resize(numPartitions);
  cachedInputTrajectoriesStock_.resize(numPartitions);

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
  // disable Eigen multi-threading
  Eigen::setNbThreads(1);

  // cache the nominal trajectories before the new rollout (time, state, input, ...)
  swapNominalTrajectoriesToCache();

  // initial controller rollout
  forwardPassTimer_.startTimer();
  avgTimeStepFP_ = rolloutTrajectory(nominalControllersStock_, nominalTimeTrajectoriesStock_, nominalPostEventIndicesStock_,
                                     nominalStateTrajectoriesStock_, nominalInputTrajectoriesStock_);
  forwardPassTimer_.endTimer();

  // This is necessary for:
  // + The moving horizon (MPC) application
  // + The very first call of the algorithm where there is no previous nominal trajectories.
  correctInitcachedNominalTrajectories();

  // linearizing the dynamics and quadratizing the cost function along nominal trajectories
  linearQuadraticApproximationTimer_.startTimer();
  approximateOptimalControlProblem();

  // to check convergence of the main loop, we need to compute the total cost and ISEs
  // calculate rollout constraint type-1 ISE
  nominalConstraint1MaxNorm_ = calculateConstraintISE(nominalTimeTrajectoriesStock_, nc1TrajectoriesStock_, EvTrajectoryStock_,
                                                      performanceIndex_.stateInputEqConstraintISE);
  // calculate rollout constraint type-2 ISE
  nominalConstraint2MaxNorm_ = calculateConstraintISE(nominalTimeTrajectoriesStock_, nc2TrajectoriesStock_, HvTrajectoryStock_,
                                                      performanceIndex_.stateEqConstraintISE);
  // calculate penalty
  performanceIndex_.inequalityConstraintPenalty = calculateInequalityConstraintPenalty(
      nominalTimeTrajectoriesStock_, ncIneqTrajectoriesStock_, hTrajectoryStock_, performanceIndex_.inequalityConstraintISE);

  // calculate rollout cost
  performanceIndex_.totalCost = calculateRolloutCost(nominalTimeTrajectoriesStock_, nominalPostEventIndicesStock_,
                                                     nominalStateTrajectoriesStock_, nominalInputTrajectoriesStock_);
  // calculates merit
  calculateRolloutMerit(nc2FinalStock_, HvFinalStock_, performanceIndex_);

  linearQuadraticApproximationTimer_.endTimer();

  // solve Riccati equations
  backwardPassTimer_.startTimer();
  avgTimeStepBP_ = solveSequentialRiccatiEquations(SmHeuristics_, SvHeuristics_, sHeuristics_);
  backwardPassTimer_.endTimer();

  // calculate controller
  computeControllerTimer_.startTimer();
  if (ddpSettings_.useRiccatiSolver_) {
    calculateController();
  } else {
    throw std::runtime_error("useRiccatiSolver=false is not valid.");
  }
  computeControllerTimer_.endTimer();

  // display
  if (ddpSettings_.displayInfo_) {
    printRolloutInfo();
  }

  // TODO(mspieler): this is not exception safe
  // restore default Eigen thread number
  Eigen::setNbThreads(0);
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
void DDP_BASE<STATE_DIM, INPUT_DIM>::runIteration() {
  // disable Eigen multi-threading
  Eigen::setNbThreads(1);

  // finding the optimal learningRate
  maxLearningRate_ = ddpSettings_.maxLearningRate_;
  linesearchTimer_.startTimer();
  lineSearch();
  linesearchTimer_.endTimer();

  // linearizing the dynamics and quadratizing the cost function along nominal trajectories
  linearQuadraticApproximationTimer_.startTimer();
  approximateOptimalControlProblem();

  linearQuadraticApproximationTimer_.endTimer();

  // solve Riccati equations
  backwardPassTimer_.startTimer();
  avgTimeStepBP_ = solveSequentialRiccatiEquations(SmHeuristics_, SvHeuristics_, sHeuristics_);
  backwardPassTimer_.endTimer();

  // calculate controller
  computeControllerTimer_.startTimer();
  if (ddpSettings_.useRiccatiSolver_) {
    calculateController();
  } else {
    throw std::runtime_error("useRiccatiSolver=false is not valid.");
  }
  computeControllerTimer_.endTimer();

  // display
  if (ddpSettings_.displayInfo_) {
    printRolloutInfo();
  }

  // TODO(mspieler): this is not exception safe
  // restore default Eigen thread number
  Eigen::setNbThreads(0);
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
void DDP_BASE<STATE_DIM, INPUT_DIM>::runImpl(scalar_t initTime, const state_vector_t& initState, scalar_t finalTime,
                                             const scalar_array_t& partitioningTimes) {
  const size_t numPartitions = partitioningTimes.size() - 1;

  linear_controller_array_t noInitialController(numPartitions, linear_controller_t());
  controller_ptr_array_t noInitialControllerPtrArray(numPartitions);
  for (size_t i = 0; i < numPartitions; i++) {
    noInitialControllerPtrArray[i] = &noInitialController[i];
  }

  // call the "run" method which uses the internal controllers stock (i.e. nominalControllersStock_)
  runImpl(initTime, initState, finalTime, partitioningTimes, noInitialControllerPtrArray);
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
void DDP_BASE<STATE_DIM, INPUT_DIM>::runImpl(scalar_t initTime, const state_vector_t& initState, scalar_t finalTime,
                                             const scalar_array_t& partitioningTimes, const controller_ptr_array_t& controllersPtrStock) {
  if (ddpSettings_.displayInfo_) {
    std::cerr << std::endl;
    std::cerr << "++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
    std::cerr << "+++++++++++++ " + algorithmName_ + " solver is initialized ++++++++++++++" << std::endl;
    std::cerr << "++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
  }

  // infeasible learning rate adjustment scheme
  if (!numerics::almost_ge(ddpSettings_.maxLearningRate_, ddpSettings_.minLearningRate_)) {
    throw std::runtime_error("The maximum learning rate is smaller than the minimum learning rate.");
  }

  if (partitioningTimes.empty()) {
    throw std::runtime_error("There should be at least one time partition.");
  }

  if (!initState.allFinite()) {
    throw std::runtime_error("DDP: initial state is not finite (time: " + std::to_string(initTime) + " [sec]).");
  }

  // update numPartitions_ if it has been changed
  if (numPartitions_ != partitioningTimes.size() - 1) {
    numPartitions_ = partitioningTimes.size() - 1;
    setupOptimizer(numPartitions_);
  }

  // update partitioningTimes_
  partitioningTimes_ = partitioningTimes;
  initActivePartition_ = lookup::findBoundedActiveIntervalInTimeArray(partitioningTimes, initTime);
  finalActivePartition_ = lookup::findBoundedActiveIntervalInTimeArray(partitioningTimes, finalTime);

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

  // display
  if (ddpSettings_.displayInfo_) {
    std::cerr << std::endl << "Rewind Counter: " << rewindCounter_ << std::endl;
    std::cerr << algorithmName_ + " solver starts from initial time " << initTime << " to final time " << finalTime << ".";
    std::cerr << this->getModeSchedule();
    std::cerr << std::endl;
  }

  iteration_ = 0;
  initState_ = initState;
  initTime_ = initTime;
  finalTime_ = finalTime;

  performanceIndexHistory_.clear();

  // check if after the truncation the internal controller is empty
  bool isInitInternalControllerEmpty = false;
  for (const linear_controller_t& controller : nominalControllersStock_) {
    isInitInternalControllerEmpty = isInitInternalControllerEmpty || controller.empty();
  }

  // display
  if (ddpSettings_.displayInfo_) {
    std::cerr << "\n#### Iteration " << iteration_ << " (Dynamics might have been violated)" << std::endl;
  }

  // distribution of the sequential tasks (e.g. Riccati solver) in between threads
  distributeWork();

  // run DDP initializer and update the member variables
  runInit();

  performanceIndexHistory_.push_back(performanceIndex_);

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

    // display the iteration's input update norm (before caching the old nominals)
    if (ddpSettings_.displayInfo_) {
      std::cerr << "\n#### Iteration " << iteration_ << std::endl;

      scalar_t maxDeltaUffNorm, maxDeltaUeeNorm;
      calculateControllerUpdateMaxNorm(maxDeltaUffNorm, maxDeltaUeeNorm);
      std::cerr << "max feedforward update norm:                            " << maxDeltaUffNorm << std::endl;
      std::cerr << "max state-input equality constraints error update norm: " << maxDeltaUeeNorm << std::endl;
    }

    auto meritCached = performanceIndex_.merit;
    auto constraint1ISECached = performanceIndex_.stateInputEqConstraintISE;

    // cache the nominal trajectories before the new rollout (time, state, input, ...)
    swapNominalTrajectoriesToCache();

    // run the an iteration of the DDP algorithm and update the member variables
    runIteration();

    performanceIndexHistory_.push_back(performanceIndex_);

    // loop break variables
    relCost = std::abs(performanceIndex_.merit - meritCached);
    relConstraint1ISE = std::abs(performanceIndex_.stateInputEqConstraintISE - constraint1ISECached);
    isConstraint1Satisfied = performanceIndex_.stateInputEqConstraintISE <= ddpSettings_.minAbsConstraint1ISE_ ||
                             relConstraint1ISE <= ddpSettings_.minRelConstraint1ISE_;
    isLearningRateStarZero = learningRateStar_ == 0 && !isInitInternalControllerEmpty;
    isCostFunctionConverged = relCost <= ddpSettings_.minRelCost_ || isLearningRateStarZero;
    isOptimizationConverged = isCostFunctionConverged && isConstraint1Satisfied;
    isInitInternalControllerEmpty = false;

  }  // end of while loop

  // display the final iteration's input update norm (before caching the old nominals)
  if (ddpSettings_.displayInfo_) {
    std::cerr << "\n#### Final rollout" << std::endl;

    scalar_t maxDeltaUffNorm, maxDeltaUeeNorm;
    calculateControllerUpdateMaxNorm(maxDeltaUffNorm, maxDeltaUeeNorm);
    std::cerr << "max feedforward update norm:                            " << maxDeltaUffNorm << std::endl;
    std::cerr << "max state-input equality constraints error update norm: " << maxDeltaUeeNorm << std::endl;
  }

  // cache the nominal trajectories before the new rollout (time, state, input, ...)
  swapNominalTrajectoriesToCache();

  // finding the final optimal learningRate and getting the optimal trajectories and controller
  maxLearningRate_ = ddpSettings_.maxLearningRate_;
  linesearchTimer_.startTimer();
  lineSearch();
  linesearchTimer_.endTimer();

  // display
  if (ddpSettings_.displayInfo_ || ddpSettings_.displayShortSummary_) {
    std::cerr << "\n++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
    std::cerr << "++++++++++++++ " + algorithmName_ + " solver is terminated ++++++++++++++" << std::endl;
    std::cerr << "++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
    std::cerr << "Time Period:               [" << initTime_ << " ," << finalTime_ << "]" << std::endl;
    std::cerr << "Number of Iterations:      " << iteration_ + 1 << " out of " << ddpSettings_.maxNumIterations_ << std::endl;

    printRolloutInfo();

    if (isOptimizationConverged) {
      if (isLearningRateStarZero) {
        std::cerr << algorithmName_ + " successfully terminates as learningRate reduced to zero." << std::endl;
      } else {
        std::cerr << algorithmName_ + " successfully terminates as cost relative change (relCost=" << relCost
                  << ") reached to the minimum value." << std::endl;
      }

      if (performanceIndex_.stateInputEqConstraintISE <= ddpSettings_.minAbsConstraint1ISE_) {
        std::cerr << "Type-1 constraint absolute ISE (absConstraint1ISE=" << performanceIndex_.stateInputEqConstraintISE
                  << ") reached to the minimum value." << std::endl;
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
