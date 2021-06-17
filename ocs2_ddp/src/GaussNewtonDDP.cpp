/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

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

#include "ocs2_ddp/GaussNewtonDDP.h"

#include <algorithm>
#include <numeric>

#include <ocs2_core/control/FeedforwardController.h>
#include <ocs2_core/integration/TrapezoidalIntegration.h>
#include <ocs2_core/misc/LinearAlgebra.h>
#include <ocs2_core/misc/Lookup.h>
#include <ocs2_core/soft_constraint/SoftConstraintPenalty.h>
#include <ocs2_core/soft_constraint/penalties/RelaxedBarrierPenalty.h>

#include <ocs2_oc/approximate_model/ChangeOfInputVariables.h>
#include <ocs2_oc/rollout/InitializerRollout.h>

#include <ocs2_ddp/HessianCorrection.h>
#include <ocs2_ddp/riccati_equations/RiccatiModificationInterpolation.h>
#include <ocs2_ddp/search_strategy/LevenbergMarquardtStrategy.h>
#include <ocs2_ddp/search_strategy/LineSearchStrategy.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
GaussNewtonDDP::GaussNewtonDDP(const RolloutBase* rolloutPtr, const SystemDynamicsBase* systemDynamicsPtr,
                               const ConstraintBase* systemConstraintsPtr, const CostFunctionBase* costFunctionPtr,
                               const Initializer* initializerPtr, ddp::Settings ddpSettings, const CostFunctionBase* heuristicsFunctionPtr)
    : SolverBase(),
      ddpSettings_(std::move(ddpSettings)),
      threadPool_(std::max(ddpSettings_.nThreads_, size_t(1)) - 1, ddpSettings_.threadPriority_) {
  // Dynamics, Constraints, derivatives, and cost
  linearQuadraticApproximatorPtrStock_.clear();
  linearQuadraticApproximatorPtrStock_.reserve(ddpSettings_.nThreads_);
  heuristicsFunctionsPtrStock_.clear();
  heuristicsFunctionsPtrStock_.reserve(ddpSettings_.nThreads_);
  dynamicsForwardRolloutPtrStock_.clear();
  dynamicsForwardRolloutPtrStock_.reserve(ddpSettings_.nThreads_);
  initializerRolloutPtrStock_.clear();
  initializerRolloutPtrStock_.reserve(ddpSettings_.nThreads_);

  // initialize all subsystems, etc.
  for (size_t i = 0; i < ddpSettings_.nThreads_; i++) {
    // initialize rollout
    dynamicsForwardRolloutPtrStock_.emplace_back(rolloutPtr->clone());

    // initialize initializerRollout
    initializerRolloutPtrStock_.emplace_back(new InitializerRollout(*initializerPtr, rolloutPtr->settings()));

    // initialize LQ approximator
    linearQuadraticApproximatorPtrStock_.emplace_back(new LinearQuadraticApproximator(
        *systemDynamicsPtr, *systemConstraintsPtr, *costFunctionPtr, ddpSettings_.checkNumericalStability_));

    // initialize heuristics functions
    if (heuristicsFunctionPtr != nullptr) {
      heuristicsFunctionsPtrStock_.emplace_back(heuristicsFunctionPtr->clone());
    } else {  // use the cost function if no heuristics function is defined
      heuristicsFunctionsPtrStock_.emplace_back(costFunctionPtr->clone());
    }
  }  // end of i loop

  // initialize penalty functions
  std::unique_ptr<PenaltyBase> penaltyFunction(new RelaxedBarrierPenalty(
      RelaxedBarrierPenalty::Config(ddpSettings_.inequalityConstraintMu_, ddpSettings_.inequalityConstraintDelta_)));
  penaltyPtr_.reset(new SoftConstraintPenalty(std::move(penaltyFunction)));

  // initialize Augmented Lagrangian parameters
  initializeConstraintPenalties();

  // search strategy method
  const auto basicStrategySettings = [&]() {
    search_strategy::Settings s;
    s.displayInfo = ddpSettings_.displayInfo_;
    s.debugPrintRollout = ddpSettings_.debugPrintRollout_;
    s.minRelCost = ddpSettings_.minRelCost_;
    s.constraintTolerance = ddpSettings_.constraintTolerance_;
    return s;
  }();
  switch (ddpSettings_.strategy_) {
    case search_strategy::Type::LINE_SEARCH: {
      std::vector<std::reference_wrapper<RolloutBase>> rolloutRefStock;
      std::vector<std::reference_wrapper<ConstraintBase>> constraintsRefStock;
      std::vector<std::reference_wrapper<CostFunctionBase>> costFunctionRefStock;
      std::vector<std::reference_wrapper<CostFunctionBase>> heuristicsFunctionsRefStock;
      for (size_t i = 0; i < ddpSettings_.nThreads_; i++) {
        rolloutRefStock.emplace_back(*dynamicsForwardRolloutPtrStock_[i]);
        constraintsRefStock.emplace_back(linearQuadraticApproximatorPtrStock_[i]->systemConstraints());
        costFunctionRefStock.emplace_back(linearQuadraticApproximatorPtrStock_[i]->costFunction());
        heuristicsFunctionsRefStock.emplace_back(*heuristicsFunctionsPtrStock_[i]);
      }  // end of i loop
      searchStrategyPtr_.reset(new LineSearchStrategy(basicStrategySettings, ddpSettings_.lineSearch_, threadPool_,
                                                      std::move(rolloutRefStock), std::move(constraintsRefStock),
                                                      std::move(costFunctionRefStock), std::move(heuristicsFunctionsRefStock), *penaltyPtr_,
                                                      [this](const PerformanceIndex& p) { return calculateRolloutMerit(p); }));
      break;
    }
    case search_strategy::Type::LEVENBERG_MARQUARDT: {
      constexpr size_t threadID = 0;
      searchStrategyPtr_.reset(new LevenbergMarquardtStrategy(
          basicStrategySettings, ddpSettings_.levenbergMarquardt_, *dynamicsForwardRolloutPtrStock_[threadID],
          linearQuadraticApproximatorPtrStock_[threadID]->systemConstraints(),
          linearQuadraticApproximatorPtrStock_[threadID]->costFunction(), *heuristicsFunctionsPtrStock_[threadID], *penaltyPtr_,
          [this](const PerformanceIndex& p) { return calculateRolloutMerit(p); }));
      break;
    }
  }  // end of switch-case
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
GaussNewtonDDP::~GaussNewtonDDP() {
  if (ddpSettings_.displayInfo_ || ddpSettings_.displayShortSummary_) {
    std::cerr << getBenchmarkingInfo() << std::endl;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::string GaussNewtonDDP::getBenchmarkingInfo() const {
  const auto initializationTotal = initializationTimer_.getTotalInMilliseconds();
  const auto linearQuadraticApproximationTotal = linearQuadraticApproximationTimer_.getTotalInMilliseconds();
  const auto backwardPassTotal = backwardPassTimer_.getTotalInMilliseconds();
  const auto computeControllerTotal = computeControllerTimer_.getTotalInMilliseconds();
  const auto searchStrategyTotal = searchStrategyTimer_.getTotalInMilliseconds();

  const auto benchmarkTotal =
      initializationTotal + linearQuadraticApproximationTotal + backwardPassTotal + computeControllerTotal + searchStrategyTotal;

  std::stringstream infoStream;
  if (benchmarkTotal > 0.0) {
    infoStream << "\n########################################################################\n";
    infoStream << "The benchmarking is computed over " << totalNumIterations_ << " iterations. \n";
    infoStream << "DDP Benchmarking\t   :\tAverage time [ms]   (% of total runtime)\n";
    infoStream << "\tInitialization     :\t" << initializationTimer_.getAverageInMilliseconds() << " [ms] \t\t("
               << initializationTotal / benchmarkTotal * 100 << "%)\n";
    infoStream << "\tLQ Approximation   :\t" << linearQuadraticApproximationTimer_.getAverageInMilliseconds() << " [ms] \t\t("
               << linearQuadraticApproximationTotal / benchmarkTotal * 100 << "%)\n";
    infoStream << "\tBackward Pass      :\t" << backwardPassTimer_.getAverageInMilliseconds() << " [ms] \t\t("
               << backwardPassTotal / benchmarkTotal * 100 << "%)\n";
    infoStream << "\tCompute Controller :\t" << computeControllerTimer_.getAverageInMilliseconds() << " [ms] \t\t("
               << computeControllerTotal / benchmarkTotal * 100 << "%)\n";
    infoStream << "\tSearch Strategy    :\t" << searchStrategyTimer_.getAverageInMilliseconds() << " [ms] \t\t("
               << searchStrategyTotal / benchmarkTotal * 100 << "%)";
  }
  return infoStream.str();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaussNewtonDDP::reset() {
  rewindCounter_ = 0;
  totalNumIterations_ = 0;

  performanceIndexHistory_.clear();

  avgTimeStepFP_ = 0.0;
  avgTimeStepBP_ = 0.0;

  // reset search strategy
  searchStrategyPtr_->reset();

  // initialize Augmented Lagrangian parameters
  initializeConstraintPenalties();

  for (size_t i = 0; i < numPartitions_; i++) {
    // very important, these are variables that are carried in between iterations
    nominalControllersStock_[i].clear();
    nominalTimeTrajectoriesStock_[i].clear();
    nominalPostEventIndicesStock_[i].clear();
    nominalStateTrajectoriesStock_[i].clear();
    nominalInputTrajectoriesStock_[i].clear();

    cachedControllersStock_[i].clear();
    cachedTimeTrajectoriesStock_[i].clear();
    cachedPostEventIndicesStock_[i].clear();
    cachedStateTrajectoriesStock_[i].clear();
    cachedInputTrajectoriesStock_[i].clear();
    cachedModelDataTrajectoriesStock_[i].clear();
    cachedModelDataEventTimesStock_[i].clear();
    cachedProjectedModelDataTrajectoriesStock_[i].clear();
    cachedRiccatiModificationTrajectoriesStock_[i].clear();
  }  // end of i loop

  // reset timers
  initializationTimer_.reset();
  linearQuadraticApproximationTimer_.reset();
  backwardPassTimer_.reset();
  computeControllerTimer_.reset();
  searchStrategyTimer_.reset();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
size_t GaussNewtonDDP::getNumIterations() const {
  return totalNumIterations_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t GaussNewtonDDP::getFinalTime() const {
  return finalTime_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
const scalar_array_t& GaussNewtonDDP::getPartitioningTimes() const {
  return partitioningTimes_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
const PerformanceIndex& GaussNewtonDDP::getPerformanceIndeces() const {
  return performanceIndex_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
const std::vector<PerformanceIndex>& GaussNewtonDDP::getIterationsLog() const {
  return performanceIndexHistory_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaussNewtonDDP::getPrimalSolution(scalar_t finalTime, PrimalSolution* primalSolutionPtr) const {
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
    primalSolutionPtr->controllerPtr_.reset(new LinearController);
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
        new FeedforwardController(primalSolutionPtr->timeTrajectory_, primalSolutionPtr->inputTrajectory_));
  }

  // fill mode schedule
  primalSolutionPtr->modeSchedule_ = this->getReferenceManager().getModeSchedule();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation GaussNewtonDDP::getValueFunction(scalar_t time, const vector_t& state) const {
  size_t partition = lookup::findBoundedActiveIntervalInTimeArray(partitioningTimes_, time);
  partition = std::max(partition, initActivePartition_);
  partition = std::min(partition, finalActivePartition_);

  // Interpolate value function trajectory
  ScalarFunctionQuadraticApproximation valueFunction;
  const auto indexAlpha = LinearInterpolation::timeSegment(time, SsTimeTrajectoryStock_[partition]);
  valueFunction.dfdxx = LinearInterpolation::interpolate(indexAlpha, SmTrajectoryStock_[partition]);
  valueFunction.dfdx = LinearInterpolation::interpolate(indexAlpha, SvTrajectoryStock_[partition]);
  valueFunction.f = LinearInterpolation::interpolate(indexAlpha, sTrajectoryStock_[partition]);

  // Re-center around query state
  const vector_t xNominal =
      LinearInterpolation::interpolate(time, cachedTimeTrajectoriesStock_[partition], cachedStateTrajectoriesStock_[partition]);
  const vector_t deltaX = state - xNominal;
  const vector_t SmDeltaX = valueFunction.dfdxx * deltaX;
  valueFunction.f += deltaX.dot(0.5 * SmDeltaX + valueFunction.dfdx);
  valueFunction.dfdx += SmDeltaX;  // Adapt dfdx after f!

  return valueFunction;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t GaussNewtonDDP::getStateInputEqualityConstraintLagrangian(scalar_t time, const vector_t& state) const {
  size_t activeSubsystem = lookup::findBoundedActiveIntervalInTimeArray(partitioningTimes_, time);
  activeSubsystem = std::max(activeSubsystem, initActivePartition_);
  activeSubsystem = std::min(activeSubsystem, finalActivePartition_);

  const auto indexAlpha = LinearInterpolation::timeSegment(time, cachedTimeTrajectoriesStock_[activeSubsystem]);
  const vector_t xNominal = LinearInterpolation::interpolate(indexAlpha, cachedStateTrajectoriesStock_[activeSubsystem]);

  const matrix_t Bm =
      LinearInterpolation::interpolate(indexAlpha, cachedModelDataTrajectoriesStock_[activeSubsystem], model_data::dynamics_dfdu);
  const matrix_t Pm =
      LinearInterpolation::interpolate(indexAlpha, cachedModelDataTrajectoriesStock_[activeSubsystem], model_data::cost_dfdux);
  const vector_t Rv =
      LinearInterpolation::interpolate(indexAlpha, cachedModelDataTrajectoriesStock_[activeSubsystem], model_data::cost_dfdu);

  const vector_t EvProjected = LinearInterpolation::interpolate(indexAlpha, cachedProjectedModelDataTrajectoriesStock_[activeSubsystem],
                                                                model_data::stateInputEqConstr_f);
  const matrix_t CmProjected = LinearInterpolation::interpolate(indexAlpha, cachedProjectedModelDataTrajectoriesStock_[activeSubsystem],
                                                                model_data::stateInputEqConstr_dfdx);
  const matrix_t Hm = LinearInterpolation::interpolate(indexAlpha, cachedRiccatiModificationTrajectoriesStock_[activeSubsystem],
                                                       riccati_modification::hamiltonianHessian);
  const matrix_t DmDagger = LinearInterpolation::interpolate(indexAlpha, cachedRiccatiModificationTrajectoriesStock_[activeSubsystem],
                                                             riccati_modification::constraintRangeProjector);

  const vector_t deltaX = state - xNominal;
  const vector_t costate = getValueFunction(time, state).dfdx;

  vector_t err = EvProjected;
  err.noalias() += CmProjected * deltaX;

  vector_t temp = -Rv;
  temp.noalias() -= Pm * deltaX;
  temp.noalias() -= Bm.transpose() * costate;
  temp.noalias() += Hm * err;

  const vector_t nu = DmDagger.transpose() * temp;

  return nu;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaussNewtonDDP::rewindOptimizer(size_t firstIndex) {
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
      swap(nominalControllersStock_[i], nominalControllersStock_[firstIndex + i]);
      SmFinalStock_[i] = SmFinalStock_[firstIndex + i];
      SvFinalStock_[i] = SvFinalStock_[firstIndex + i];
      sFinalStock_[i] = sFinalStock_[firstIndex + i];
      xFinalStock_[i] = xFinalStock_[firstIndex + i];
    } else {
      nominalControllersStock_[i].clear();
      SmFinalStock_[i].setZero(0, 0);
      SvFinalStock_[i].setZero(0);
      sFinalStock_[i] = 0.0;
      xFinalStock_[i].setZero(0);
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaussNewtonDDP::computeNormalizedTime(const scalar_array_t& timeTrajectory, const size_array_t& postEventIndices,
                                           scalar_array_t& normalizedTimeTrajectory, size_array_t& normalizedPostEventIndices) {
  const int N = timeTrajectory.size();
  const int NE = postEventIndices.size();

  // normalized time
  normalizedTimeTrajectory.resize(N);
  for (int k = 0; k < N; k++) {
    normalizedTimeTrajectory[N - 1 - k] = -timeTrajectory[k];
  }

  // normalized event past the index
  normalizedPostEventIndices.resize(NE);
  for (int k = 0; k < NE; k++) {
    normalizedPostEventIndices[NE - 1 - k] = N - postEventIndices[k];
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaussNewtonDDP::adjustController(const scalar_array_t& newEventTimes, const scalar_array_t& controllerEventTimes) {
  // adjust the nominal controllerStock using trajectory spreading
  if (!nominalControllersStock_.empty()) {
    trajectorySpreadingController_.adjustController(newEventTimes, controllerEventTimes, nominalControllersStock_);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaussNewtonDDP::setupOptimizer(size_t numPartitions) {
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

  cachedControllersStock_.resize(numPartitions);
  cachedTimeTrajectoriesStock_.resize(numPartitions);
  cachedPostEventIndicesStock_.resize(numPartitions);
  cachedStateTrajectoriesStock_.resize(numPartitions);
  cachedInputTrajectoriesStock_.resize(numPartitions);

  /*
   * Riccati solver variables and controller update
   */
  SmFinalStock_ = matrix_array_t(numPartitions);
  SvFinalStock_ = vector_array_t(numPartitions);
  sFinalStock_ = scalar_array_t(numPartitions);
  xFinalStock_ = vector_array_t(numPartitions);

  SsTimeTrajectoryStock_.resize(numPartitions);
  SsNormalizedTimeTrajectoryStock_.resize(numPartitions);
  SsNormalizedEventsPastTheEndIndecesStock_.resize(numPartitions);
  sTrajectoryStock_.resize(numPartitions);
  SvTrajectoryStock_.resize(numPartitions);
  SmTrajectoryStock_.resize(numPartitions);

  /*
   * intermediate model data
   */
  modelDataTrajectoriesStock_.resize(numPartitions);
  cachedModelDataTrajectoriesStock_.resize(numPartitions);

  /*
   * event times model data
   */
  modelDataEventTimesStock_.resize(numPartitions);
  cachedModelDataEventTimesStock_.resize(numPartitions);

  /*
   * projected intermediate model data
   */
  projectedModelDataTrajectoriesStock_.resize(numPartitions);
  cachedProjectedModelDataTrajectoriesStock_.resize(numPartitions);

  /*
   * Riccati solver related variables
   */
  riccatiModificationTrajectoriesStock_.resize(numPartitions);
  cachedRiccatiModificationTrajectoriesStock_.resize(numPartitions);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::vector<std::pair<int, int>> GaussNewtonDDP::distributeWork(int numWorkers) const {
  const int subsystemsPerThread = (finalActivePartition_ - initActivePartition_ + 1) / numWorkers;
  int remainingSubsystems = (finalActivePartition_ - initActivePartition_ + 1) % numWorkers;

  int startingId, endingId = finalActivePartition_;
  std::vector<std::pair<int, int>> indexPeriodArray;
  for (size_t i = 0; i < numWorkers; i++) {
    if (remainingSubsystems > 0) {
      startingId = endingId - subsystemsPerThread;
      remainingSubsystems--;
    } else {
      startingId = endingId - subsystemsPerThread + 1;
    }
    if (startingId <= endingId) {
      indexPeriodArray.emplace_back(startingId, endingId);
    }
    endingId = startingId - 1;
  }

  if (ddpSettings_.displayInfo_) {
    std::cerr << "Initial Active Subsystem: " << initActivePartition_ << "\n";
    std::cerr << "Final Active Subsystem:   " << finalActivePartition_ << "\n";
    std::cerr << "Backward path work distribution:\n";
    for (size_t i = 0; i < numWorkers; i++) {
      std::cerr << "start: " << indexPeriodArray[i].first << "\t";
      std::cerr << "end: " << indexPeriodArray[i].second << "\t";
      std::cerr << "num: " << indexPeriodArray[i].second - indexPeriodArray[i].first + 1 << "\n";
    }
    std::cerr << "\n";
  }

  return indexPeriodArray;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaussNewtonDDP::runParallel(std::function<void(void)> taskFunction, size_t N) {
  threadPool_.runParallel([&](int) { taskFunction(); }, N);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t GaussNewtonDDP::rolloutInitialTrajectory(std::vector<LinearController>& controllersStock, scalar_array2_t& timeTrajectoriesStock,
                                                  size_array2_t& postEventIndicesStock, vector_array2_t& stateTrajectoriesStock,
                                                  vector_array2_t& inputTrajectoriesStock,
                                                  std::vector<std::vector<ModelData>>& modelDataTrajectoriesStock,
                                                  std::vector<std::vector<ModelData>>& modelDataEventTimesStock,
                                                  size_t workerIndex /*= 0*/) {
  const scalar_array_t& eventTimes = this->getReferenceManager().getModeSchedule().eventTimes;

  if (controllersStock.size() != numPartitions_) {
    throw std::runtime_error("controllersStock has less controllers then the number of subsystems");
  }

  // Prepare outputs
  timeTrajectoriesStock.resize(numPartitions_);
  postEventIndicesStock.resize(numPartitions_);
  stateTrajectoriesStock.resize(numPartitions_);
  inputTrajectoriesStock.resize(numPartitions_);
  modelDataTrajectoriesStock.resize(numPartitions_);
  modelDataEventTimesStock.resize(numPartitions_);
  for (size_t i = 0; i < numPartitions_; i++) {
    timeTrajectoriesStock[i].clear();
    postEventIndicesStock[i].clear();
    stateTrajectoriesStock[i].clear();
    inputTrajectoriesStock[i].clear();
    modelDataTrajectoriesStock[i].clear();
    modelDataEventTimesStock[i].clear();
  }

  // Find until where we have a controller available for the rollout
  scalar_t controllerAvailableTill = initTime_;
  for (size_t i = initActivePartition_; i < finalActivePartition_ + 1; i++) {
    if (!controllersStock[i].empty()) {
      controllerAvailableTill = controllersStock[i].timeStamp_.back();
    } else {
      break;  // break on the first empty controller (cannot have gaps in the controllers)
    }
  }

  if (ddpSettings_.debugPrintRollout_) {
    std::cerr << "[GaussNewtonDDP::rolloutInitialTrajectory] for t = [" << initTime_ << ", " << finalTime_ << "]\n"
              << "\tcontroller available till t = " << controllerAvailableTill << "\n";
  }

  size_t numSteps = 0;
  vector_t xCurrent = initState_;
  for (size_t i = initActivePartition_; i < finalActivePartition_ + 1; i++) {
    // Start and end of rollout segment
    const scalar_t t0 = (i == initActivePartition_) ? initTime_ : partitioningTimes_[i];
    const scalar_t tf = (i == finalActivePartition_) ? finalTime_ : partitioningTimes_[i + 1];

    // Divide the rollout segment in controller rollout and operating points
    const std::pair<scalar_t, scalar_t> controllerRolloutFromTo{t0, std::max(t0, std::min(controllerAvailableTill, tf))};
    const std::pair<scalar_t, scalar_t> operatingPointsFromTo{controllerRolloutFromTo.second, tf};

    if (ddpSettings_.debugPrintRollout_) {
      std::cerr << "[GaussNewtonDDP::rolloutInitialTrajectory] partition " << i << " for t = [" << t0 << ", " << tf << "]\n";
      if (controllerRolloutFromTo.first < controllerRolloutFromTo.second) {
        std::cerr << "\twill use controller for t = [" << controllerRolloutFromTo.first << ", " << controllerRolloutFromTo.second << "]\n";
      }
      if (operatingPointsFromTo.first < operatingPointsFromTo.second) {
        std::cerr << "\twill use operating points for t = [" << operatingPointsFromTo.first << ", " << operatingPointsFromTo.second
                  << "]\n";
      }
    }

    // Rollout with controller
    if (controllerRolloutFromTo.first < controllerRolloutFromTo.second) {
      xCurrent = dynamicsForwardRolloutPtrStock_[workerIndex]->run(
          controllerRolloutFromTo.first, xCurrent, controllerRolloutFromTo.second, &controllersStock[i], eventTimes,
          timeTrajectoriesStock[i], postEventIndicesStock[i], stateTrajectoriesStock[i], inputTrajectoriesStock[i]);
    }

    // Finish rollout with operating points
    if (operatingPointsFromTo.first < operatingPointsFromTo.second) {
      // Remove last point of the controller rollout if it is directly past an event. Here where we want to use the initializer
      // instead. However, we do start the integration at the state after the event. i.e. the jump map remains applied.
      if (!postEventIndicesStock[i].empty() && postEventIndicesStock[i].back() == (timeTrajectoriesStock[i].size() - 1)) {
        // Start new integration at the time point after the event
        timeTrajectoriesStock[i].pop_back();
        stateTrajectoriesStock[i].pop_back();
        inputTrajectoriesStock[i].pop_back();
        // eventsPastTheEndIndeces is not removed because we need to mark the start of the operatingPointTrajectory as being after an event.
      }

      scalar_array_t timeTrajectoryTail;
      size_array_t eventsPastTheEndIndecesTail;
      vector_array_t stateTrajectoryTail;
      vector_array_t inputTrajectoryTail;
      xCurrent = initializerRolloutPtrStock_[workerIndex]->run(operatingPointsFromTo.first, xCurrent, operatingPointsFromTo.second, nullptr,
                                                               eventTimes, timeTrajectoryTail, eventsPastTheEndIndecesTail,
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

    // update model data trajectory
    modelDataTrajectoriesStock[i].resize(timeTrajectoriesStock[i].size());
    for (size_t k = 0; k < timeTrajectoriesStock[i].size(); k++) {
      modelDataTrajectoriesStock[i][k].time_ = timeTrajectoriesStock[i][k];
      modelDataTrajectoriesStock[i][k].stateDim_ = stateTrajectoriesStock[i][k].size();
      modelDataTrajectoriesStock[i][k].inputDim_ = inputTrajectoriesStock[i][k].size();
      modelDataTrajectoriesStock[i][k].dynamicsBias_.setZero(stateTrajectoriesStock[i][k].size());
    }

    // update model data at event times
    modelDataEventTimesStock[i].resize(postEventIndicesStock[i].size());
    for (size_t ke = 0; ke < postEventIndicesStock[i].size(); ke++) {
      const auto index = postEventIndicesStock[i][ke] - 1;
      modelDataEventTimesStock[i][ke].time_ = timeTrajectoriesStock[i][index];
      modelDataEventTimesStock[i][ke].stateDim_ = stateTrajectoriesStock[i][index].size();
      modelDataEventTimesStock[i][ke].inputDim_ = inputTrajectoriesStock[i][index].size();
      modelDataEventTimesStock[i][ke].dynamicsBias_.setZero(stateTrajectoriesStock[i][index].size());
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
      std::cerr << "\n++++++++++++++++++++++++++++++\n";
      std::cerr << "Partition: " << i;
      std::cerr << "\n++++++++++++++++++++++++++++++\n";
      RolloutBase::display(timeTrajectoriesStock[i], postEventIndicesStock[i], stateTrajectoriesStock[i], &inputTrajectoriesStock[i]);
    }
  }
  // average time step
  return (controllerAvailableTill - initTime_) / numSteps;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaussNewtonDDP::printRolloutInfo() const {
  std::cerr << performanceIndex_ << '\n';
  std::cerr << "forward pass average time step:  " << avgTimeStepFP_ * 1e+3 << " [ms].\n";
  std::cerr << "backward pass average time step: " << avgTimeStepBP_ * 1e+3 << " [ms].\n";
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t GaussNewtonDDP::calculateRolloutMerit(const PerformanceIndex& performanceIndex) const {
  // total cost
  scalar_t merit = performanceIndex.totalCost;

  // intermediate state-only equality constraints
  merit += constraintPenaltyCoefficients_.stateEqConstrPenaltyCoeff * performanceIndex.stateEqConstraintISE;

  // final state-only equality constraints
  merit += constraintPenaltyCoefficients_.stateFinalEqConstrPenaltyCoeff * performanceIndex.stateEqFinalConstraintSSE;

  // intermediate state-input equality constraints
  merit += constraintPenaltyCoefficients_.stateInputEqConstrPenaltyCoeff * std::sqrt(performanceIndex.stateInputEqConstraintISE);

  // intermediate inequality constraints
  merit += performanceIndex.inequalityConstraintPenalty;

  return merit;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t GaussNewtonDDP::solveSequentialRiccatiEquationsImpl(const matrix_t& SmFinal, const vector_t& SvFinal, const scalar_t& sFinal) {
  // clear partitions
  for (size_t i = 0; i < numPartitions_; i++) {
    SsTimeTrajectoryStock_[i].clear();
    SsNormalizedTimeTrajectoryStock_[i].clear();
    SsNormalizedEventsPastTheEndIndecesStock_[i].clear();
    SmTrajectoryStock_[i].clear();
    SvTrajectoryStock_[i].clear();
    sTrajectoryStock_[i].clear();
  }  // end of i loop

  // solve it sequentially for the first iteration
  if (totalNumIterations_ == 0) {
    std::pair<int, int> indexPeriod{initActivePartition_, finalActivePartition_};
    solveRiccatiEquationsForPartitions(0, indexPeriod, SmFinal, SvFinal, sFinal);

  } else {  // solve it in parallel
    // distribution of the sequential tasks (e.g. Riccati solver) in between threads
    const std::vector<std::pair<int, int>> indexPeriodArray = distributeWork(ddpSettings_.nThreads_);

    // correct the end of parrtition's final values based on the cached values
    SmFinalStock_[finalActivePartition_] = SmFinal;
    SvFinalStock_[finalActivePartition_] = SvFinal;
    sFinalStock_[finalActivePartition_] = sFinal;
    xFinalStock_[finalActivePartition_] = nominalStateTrajectoriesStock_[finalActivePartition_].back();
    for (size_t i = initActivePartition_; i < finalActivePartition_; i++) {
      const vector_t& xFinalUpdated = nominalStateTrajectoriesStock_[i + 1].front();
      const vector_t deltaState = xFinalUpdated - xFinalStock_[i];
      const vector_t SmFinalDeltaState = SmFinalStock_[i] * deltaState;
      sFinalStock_[i] += deltaState.dot(0.5 * SmFinalDeltaState + SvFinalStock_[i]);
      SvFinalStock_[i] += SmFinalDeltaState;
    }  // end of loop

    nextTaskId_ = 0;
    std::function<void(void)> task = [&] {
      const size_t taskId = nextTaskId_++;  // assign task ID (atomic)
      const auto& indexPeriod = indexPeriodArray[taskId];
      solveRiccatiEquationsForPartitions(taskId, indexPeriod, SmFinalStock_[indexPeriod.second], SvFinalStock_[indexPeriod.second],
                                         sFinalStock_[indexPeriod.second]);
    };
    runParallel(task, indexPeriodArray.size());
  }

  // update the final values for the next iteration
  SmFinalStock_[finalActivePartition_] = SmFinal;
  SvFinalStock_[finalActivePartition_] = SvFinal;
  sFinalStock_[finalActivePartition_] = sFinal;
  xFinalStock_[finalActivePartition_] = nominalStateTrajectoriesStock_[finalActivePartition_].back();
  for (size_t i = initActivePartition_; i < finalActivePartition_; i++) {
    SmFinalStock_[i] = SmTrajectoryStock_[i + 1].front();
    SvFinalStock_[i] = SvTrajectoryStock_[i + 1].front();
    sFinalStock_[i] = sTrajectoryStock_[i + 1].front();
    xFinalStock_[i] = nominalStateTrajectoriesStock_[i + 1].front();
  }  // end of i loop

  // testing the numerical stability of the Riccati equations
  if (ddpSettings_.checkNumericalStability_) {
    for (size_t i = 0; i < numPartitions_; i++) {
      int N = SsTimeTrajectoryStock_[i].size();
      for (int k = N - 1; k >= 0; k--) {
        try {
          if (!SmTrajectoryStock_[i][k].allFinite()) {
            throw std::runtime_error("Sm is unstable.");
          }
          if (LinearAlgebra::eigenvalues(SmTrajectoryStock_[i][k]).real().minCoeff() < -Eigen::NumTraits<scalar_t>::epsilon()) {
            throw std::runtime_error("Sm matrix is not positive semi-definite. It's smallest eigenvalue is " +
                                     std::to_string(LinearAlgebra::eigenvalues(SmTrajectoryStock_[i][k]).real().minCoeff()) + ".");
          }
          if (!SvTrajectoryStock_[i][k].allFinite()) {
            throw std::runtime_error("Sv is unstable.");
          }
          if (sTrajectoryStock_[i][k] != sTrajectoryStock_[i][k]) {
            throw std::runtime_error("s is unstable");
          }
        } catch (const std::exception& error) {
          std::cerr << "what(): " << error.what() << " at time " << SsTimeTrajectoryStock_[i][k] << " [sec].\n";
          for (int kp = k; kp < k + 10; kp++) {
            if (kp >= N) {
              continue;
            }
            std::cerr << "Sm[" << SsTimeTrajectoryStock_[i][kp] << "]:\n" << SmTrajectoryStock_[i][kp].norm() << "\n";
            std::cerr << "Sv[" << SsTimeTrajectoryStock_[i][kp] << "]:\t" << SvTrajectoryStock_[i][kp].transpose().norm() << "\n";
            std::cerr << "s[" << SsTimeTrajectoryStock_[i][kp] << "]:\t" << sTrajectoryStock_[i][kp] << "\n";
          }
          throw;
        }
      }  // end of k loop
    }    // end of i loop
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
/******************************************************************************************************/
void GaussNewtonDDP::solveRiccatiEquationsForPartitions(size_t taskId, const std::pair<int, int>& indexPeriod, matrix_t SmFinal,
                                                        vector_t SvFinal, scalar_t sFinal) {
  for (int i = indexPeriod.second; i >= indexPeriod.first; i--) {
    assert(initActivePartition_ <= i && i <= finalActivePartition_);

    // solve the backward pass
    riccatiEquationsWorker(taskId, i, SmFinal, SvFinal, sFinal);

    // set the final value for next Riccati equation
    SmFinal = SmTrajectoryStock_[i].front();
    SvFinal = SvTrajectoryStock_[i].front();
    sFinal = sTrajectoryStock_[i].front();
  }  // end of i loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaussNewtonDDP::calculateController() {
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

  // if the final time is not an event time change the last control to the second to the last
  const auto& timeTrajectory = nominalTimeTrajectoriesStock_[finalActivePartition_];
  const auto& postEventIndices = nominalPostEventIndicesStock_[finalActivePartition_];
  if (postEventIndices.empty() || postEventIndices.back() != timeTrajectory.size() - 1) {
    auto& ctrl = nominalControllersStock_[finalActivePartition_];
    if (ctrl.size() > 1) {
      const auto secondToLastIndex = ctrl.size() - 2;
      ctrl.gainArray_.back() = ctrl.gainArray_[secondToLastIndex];
      ctrl.biasArray_.back() = ctrl.biasArray_[secondToLastIndex];
      ctrl.deltaBiasArray_.back() = ctrl.deltaBiasArray_[secondToLastIndex];
    } else if (finalActivePartition_ > initActivePartition_) {
      const auto secondToLastCtrl = nominalControllersStock_[finalActivePartition_ - 1];
      ctrl.gainArray_.back() = secondToLastCtrl.gainArray_.back();
      ctrl.biasArray_.back() = secondToLastCtrl.biasArray_.back();
      ctrl.deltaBiasArray_.back() = secondToLastCtrl.deltaBiasArray_.back();
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaussNewtonDDP::calculateControllerUpdateMaxNorm(scalar_t& maxDeltaUffNorm, scalar_t& maxDeltaUeeNorm) const {
  maxDeltaUffNorm = 0.0;
  maxDeltaUeeNorm = 0.0;
  for (size_t i = initActivePartition_; i <= finalActivePartition_; i++) {
    for (size_t k = 0; k < nominalControllersStock_[i].timeStamp_.size(); k++) {
      maxDeltaUffNorm = std::max(maxDeltaUffNorm, nominalControllersStock_[i].deltaBiasArray_[k].norm());

      const auto& time = nominalControllersStock_[i].timeStamp_[k];
      const auto indexAlpha = LinearInterpolation::timeSegment(time, nominalTimeTrajectoriesStock_[i]);
      const vector_t nominalState = LinearInterpolation::interpolate(indexAlpha, nominalStateTrajectoriesStock_[i]);
      const vector_t nominalInput = LinearInterpolation::interpolate(indexAlpha, nominalInputTrajectoriesStock_[i]);
      const vector_t deltaUee =
          nominalInput - nominalControllersStock_[i].gainArray_[k] * nominalState - nominalControllersStock_[i].biasArray_[k];
      maxDeltaUeeNorm = std::max(maxDeltaUeeNorm, deltaUee.norm());

    }  // end of k loop
  }    // end of i loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaussNewtonDDP::approximateOptimalControlProblem() {
  for (size_t i = 0; i < numPartitions_; i++) {
    /*
     * compute and augment the LQ approximation of intermediate times for the partition i
     */
    if (!nominalTimeTrajectoriesStock_[i].empty()) {
      // perform the LQ approximation for intermediate times at partition i
      approximateIntermediateLQ(nominalTimeTrajectoriesStock_[i], nominalPostEventIndicesStock_[i], nominalStateTrajectoriesStock_[i],
                                nominalInputTrajectoriesStock_[i], modelDataTrajectoriesStock_[i]);

      // augment the intermediate cost by performing augmentCostWorker for the partition i
      nextTimeIndex_ = 0;
      nextTaskId_ = 0;
      std::function<void(void)> task = [this, i] {
        size_t timeIndex;
        size_t taskId = nextTaskId_++;  // assign task ID (atomic)

        // get next time index is atomic
        while ((timeIndex = nextTimeIndex_++) < nominalTimeTrajectoriesStock_[i].size()) {
          // augment cost
          augmentCostWorker(taskId, constraintPenaltyCoefficients_.stateEqConstrPenaltyCoeff, 0.0,
                            modelDataTrajectoriesStock_[i][timeIndex]);
        }
      };
      runParallel(task, ddpSettings_.nThreads_);
    }

    /*
     * compute and augment the LQ approximation of the event times for the partition i.
     * also call shiftHessian on the event time's cost 2nd order derivative.
     */
    const size_t NE = nominalPostEventIndicesStock_[i].size();
    if (NE > 0) {
      // perform the approximateEventsLQWorker for partition i
      nextTimeIndex_ = 0;
      nextTaskId_ = 0;
      std::function<void(void)> task = [this, i] {
        int timeIndex;
        size_t taskId = nextTaskId_++;  // assign task ID (atomic)

        // get next time index is atomic
        while ((timeIndex = nextTimeIndex_++) < nominalPostEventIndicesStock_[i].size()) {
          // execute approximateLQ for the given partition and event time index
          const size_t k = nominalPostEventIndicesStock_[i][timeIndex] - 1;
          linearQuadraticApproximatorPtrStock_[taskId]->approximateLQProblemAtEventTime(
              nominalTimeTrajectoriesStock_[i][k], nominalStateTrajectoriesStock_[i][k], nominalInputTrajectoriesStock_[i][k],
              modelDataEventTimesStock_[i][timeIndex]);
          // augment cost
          augmentCostWorker(taskId, constraintPenaltyCoefficients_.stateFinalEqConstrPenaltyCoeff, 0.0,
                            modelDataEventTimesStock_[i][timeIndex]);
          // shift Hessian for event times
          if (ddpSettings_.strategy_ == search_strategy::Type::LINE_SEARCH) {
            hessian_correction::shiftHessian(ddpSettings_.lineSearch_.hessianCorrectionStrategy_,
                                             modelDataEventTimesStock_[i][timeIndex].cost_.dfdxx,
                                             ddpSettings_.lineSearch_.hessianCorrectionMultiple_);
          }
        }
      };
      runParallel(task, ddpSettings_.nThreads_);
    }

  }  // end of i loop

  /*
   * compute the Heuristics function at the final time. Also call shiftHessian on the Heuristics 2nd order derivative.
   */
  heuristics_ = heuristicsFunctionsPtrStock_[0]->finalCostQuadraticApproximation(
      nominalTimeTrajectoriesStock_[finalActivePartition_].back(), nominalStateTrajectoriesStock_[finalActivePartition_].back());

  // shift Hessian for final time
  if (ddpSettings_.strategy_ == search_strategy::Type::LINE_SEARCH) {
    hessian_correction::shiftHessian(ddpSettings_.lineSearch_.hessianCorrectionStrategy_, heuristics_.dfdxx,
                                     ddpSettings_.lineSearch_.hessianCorrectionMultiple_);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaussNewtonDDP::computeProjectionAndRiccatiModification(const ModelData& modelData, const matrix_t& Sm, ModelData& projectedModelData,
                                                             riccati_modification::Data& riccatiModification) const {
  // compute the Hamiltonian's Hessian
  riccatiModification.time_ = modelData.time_;
  riccatiModification.hamiltonianHessian_ = computeHamiltonianHessian(modelData, Sm);

  // compute projectors
  computeProjections(riccatiModification.hamiltonianHessian_, modelData.stateInputEqConstr_.dfdu,
                     riccatiModification.constraintRangeProjector_, riccatiModification.constraintNullProjector_);

  // project LQ
  projectLQ(modelData, riccatiModification.constraintRangeProjector_, riccatiModification.constraintNullProjector_, projectedModelData);

  // compute deltaQm, deltaGv, deltaGm
  searchStrategyPtr_->computeRiccatiModification(projectedModelData, riccatiModification.deltaQm_, riccatiModification.deltaGv_,
                                                 riccatiModification.deltaGm_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaussNewtonDDP::computeProjections(const matrix_t& Hm, const matrix_t& Dm, matrix_t& constraintRangeProjector,
                                        matrix_t& constraintNullProjector) const {
  // UUT decomposition of inv(Hm)
  matrix_t HmInvUmUmT;
  LinearAlgebra::computeInverseMatrixUUT(Hm, HmInvUmUmT);

  // compute DmDagger, DmDaggerTHmDmDaggerUUT, HmInverseConstrainedLowRank
  if (Dm.rows() == 0) {
    constraintRangeProjector.setZero(Dm.cols(), 0);
    constraintNullProjector = HmInvUmUmT;

  } else {
    // check numerics
    if (ddpSettings_.checkNumericalStability_) {
      if (LinearAlgebra::rank(Dm) != Dm.rows()) {
        std::string msg = ">>> WARNING: The state-input constraints are rank deficient!";
        this->printString(msg);
      }
    }
    // constraint projectors are obtained at once
    matrix_t DmDaggerTHmDmDaggerUUT;
    ocs2::LinearAlgebra::computeConstraintProjection(Dm, HmInvUmUmT, constraintRangeProjector, DmDaggerTHmDmDaggerUUT,
                                                     constraintNullProjector);
  }

  // check
  if (ddpSettings_.checkNumericalStability_) {
    matrix_t HmProjected = constraintNullProjector.transpose() * Hm * constraintNullProjector;
    const int nullSpaceDim = Hm.rows() - Dm.rows();
    if (!HmProjected.isApprox(matrix_t::Identity(nullSpaceDim, nullSpaceDim))) {
      std::cerr << "HmProjected:\n" << HmProjected << "\n";
      throw std::runtime_error("HmProjected should be identity!");
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaussNewtonDDP::projectLQ(const ModelData& modelData, const matrix_t& constraintRangeProjector,
                               const matrix_t& constraintNullProjector, ModelData& projectedModelData) const {
  // dimensions and time
  projectedModelData.time_ = modelData.time_;
  projectedModelData.stateDim_ = modelData.stateDim_;
  projectedModelData.inputDim_ = modelData.inputDim_ - modelData.stateInputEqConstr_.f.rows();

  // unhandled constraints
  projectedModelData.ineqConstr_.f.setZero(0);
  projectedModelData.stateEqConstr_.f.setZero(0);

  if (modelData.stateInputEqConstr_.f.rows() == 0) {
    // Change of variables u = Pu * tilde{u}
    // Pu = constraintNullProjector;

    // projected state-input equality constraints
    projectedModelData.stateInputEqConstr_.f.setZero(projectedModelData.inputDim_);
    projectedModelData.stateInputEqConstr_.dfdx.setZero(projectedModelData.inputDim_, projectedModelData.stateDim_);
    projectedModelData.stateInputEqConstr_.dfdu.setZero(modelData.inputDim_, modelData.inputDim_);

    // dynamics
    projectedModelData.dynamics_ = modelData.dynamics_;
    changeOfInputVariables(projectedModelData.dynamics_, constraintNullProjector);

    // dynamics bias
    projectedModelData.dynamicsBias_ = modelData.dynamicsBias_;

    // cost
    projectedModelData.cost_ = modelData.cost_;
    changeOfInputVariables(projectedModelData.cost_, constraintNullProjector);
  } else {
    // Change of variables u = Pu * tilde{u} + Px * x + u0
    // Pu = constraintNullProjector;
    // Px (= -CmProjected) = -constraintRangeProjector * C
    // u0 (= -EvProjected) = -constraintRangeProjector * e

    /* projected state-input equality constraints */
    projectedModelData.stateInputEqConstr_.f.noalias() = constraintRangeProjector * modelData.stateInputEqConstr_.f;
    projectedModelData.stateInputEqConstr_.dfdx.noalias() = constraintRangeProjector * modelData.stateInputEqConstr_.dfdx;
    projectedModelData.stateInputEqConstr_.dfdu.noalias() = constraintRangeProjector * modelData.stateInputEqConstr_.dfdu;

    // Change of variable matrices
    const auto& Pu = constraintNullProjector;
    const matrix_t Px = -projectedModelData.stateInputEqConstr_.dfdx;
    const matrix_t u0 = -projectedModelData.stateInputEqConstr_.f;

    // dynamics
    projectedModelData.dynamics_ = modelData.dynamics_;
    changeOfInputVariables(projectedModelData.dynamics_, Pu, Px, u0);

    // dynamics bias
    projectedModelData.dynamicsBias_ = modelData.dynamicsBias_;
    projectedModelData.dynamicsBias_.noalias() += modelData.dynamics_.dfdu * u0;

    // cost
    projectedModelData.cost_ = modelData.cost_;
    changeOfInputVariables(projectedModelData.cost_, Pu, Px, u0);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaussNewtonDDP::augmentCostWorker(size_t workerIndex, scalar_t stateEqConstrPenaltyCoeff, scalar_t stateInputEqConstrPenaltyCoeff,
                                       ModelData& modelData) const {
  // state equality constraint (type 2) coefficients
  if (modelData.stateEqConstr_.f.rows() > 0) {
    const vector_t& Hv = modelData.stateEqConstr_.f;
    const matrix_t& Fm = modelData.stateEqConstr_.dfdx;
    modelData.cost_.f += 0.5 * stateEqConstrPenaltyCoeff * Hv.dot(Hv);
    modelData.cost_.dfdx.noalias() += stateEqConstrPenaltyCoeff * Fm.transpose() * Hv;
    modelData.cost_.dfdxx.noalias() += stateEqConstrPenaltyCoeff * Fm.transpose() * Fm;
  }

  // state-input equality constraint (type 1) coefficients
  if (modelData.stateInputEqConstr_.f.rows() > 0 && !numerics::almost_eq(stateInputEqConstrPenaltyCoeff, 0.0)) {
    const vector_t& Ev = modelData.stateInputEqConstr_.f;
    const matrix_t& Cm = modelData.stateInputEqConstr_.dfdx;
    const matrix_t& Dm = modelData.stateInputEqConstr_.dfdu;
    modelData.cost_.f += 0.5 * stateInputEqConstrPenaltyCoeff * Ev.squaredNorm();
    modelData.cost_.dfdx.noalias() += stateInputEqConstrPenaltyCoeff * Cm.transpose() * Ev;
    modelData.cost_.dfdu.noalias() += stateInputEqConstrPenaltyCoeff * Dm.transpose() * Ev;
    modelData.cost_.dfdxx.noalias() += stateInputEqConstrPenaltyCoeff * Cm.transpose() * Cm;
    modelData.cost_.dfduu.noalias() += stateInputEqConstrPenaltyCoeff * Dm.transpose() * Dm;
    modelData.cost_.dfdux.noalias() += stateInputEqConstrPenaltyCoeff * Dm.transpose() * Cm;
  }

  // inequality constraints
  if (modelData.ineqConstr_.f.rows() > 0) {
    modelData.cost_ += penaltyPtr_->getQuadraticApproximation(modelData.ineqConstr_);

    // checking the numerical stability again
    if (ddpSettings_.checkNumericalStability_) {
      auto errorDescription = modelData.checkCostProperties();
      if (!errorDescription.empty()) {
        throw std::runtime_error(errorDescription);
      }
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaussNewtonDDP::initializeConstraintPenalties() {
  assert(ddpSettings_.constraintPenaltyInitialValue_ > 1.0);
  assert(ddpSettings_.constraintPenaltyIncreaseRate_ > 1.0);

  // state-only equality
  constraintPenaltyCoefficients_.stateEqConstrPenaltyCoeff = ddpSettings_.constraintPenaltyInitialValue_;
  constraintPenaltyCoefficients_.stateEqConstrPenaltyTol = ddpSettings_.constraintTolerance_;

  // final state-only equality
  constraintPenaltyCoefficients_.stateFinalEqConstrPenaltyCoeff = ddpSettings_.constraintPenaltyInitialValue_;
  constraintPenaltyCoefficients_.stateFinalEqConstrPenaltyTol = ddpSettings_.constraintTolerance_;

  // state-input equality
  constraintPenaltyCoefficients_.stateInputEqConstrPenaltyCoeff = ddpSettings_.constraintPenaltyInitialValue_;
  constraintPenaltyCoefficients_.stateInputEqConstrPenaltyTol =
      1.0 / std::pow(constraintPenaltyCoefficients_.stateInputEqConstrPenaltyCoeff, 0.1);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaussNewtonDDP::runSearchStrategy(scalar_t expectedCost) {
  auto performanceIndex = performanceIndex_;
  scalar_array2_t timeTrajectoriesStock(numPartitions_);
  size_array2_t postEventIndicesStock(numPartitions_);
  vector_array2_t stateTrajectoriesStock(numPartitions_);
  vector_array2_t inputTrajectoriesStock(numPartitions_);
  std::vector<std::vector<ModelData>> modelDataTrajectoriesStock(numPartitions_);
  std::vector<std::vector<ModelData>> modelDataEventTimesStock(numPartitions_);

  bool success = searchStrategyPtr_->run(expectedCost, this->getReferenceManager().getModeSchedule(), nominalControllersStock_,
                                         performanceIndex, timeTrajectoriesStock, postEventIndicesStock, stateTrajectoriesStock,
                                         inputTrajectoriesStock, modelDataTrajectoriesStock, modelDataEventTimesStock, avgTimeStepFP_);

  // accept or reject the search
  if (success) {
    // update nominal trajectories
    performanceIndex_ = performanceIndex;
    nominalTimeTrajectoriesStock_.swap(timeTrajectoriesStock);
    nominalPostEventIndicesStock_.swap(postEventIndicesStock);
    nominalStateTrajectoriesStock_.swap(stateTrajectoriesStock);
    nominalInputTrajectoriesStock_.swap(inputTrajectoriesStock);
    modelDataTrajectoriesStock_.swap(modelDataTrajectoriesStock);
    modelDataEventTimesStock_.swap(modelDataEventTimesStock);
    // clear the feedforward increments
    for (auto& controller : nominalControllersStock_) {
      controller.deltaBiasArray_.clear();
    }

  } else {
    // swap back the cached nominal trajectories
    swapDataToCache();
    // replace the cached controller as the nominal
    nominalControllersStock_.swap(cachedControllersStock_);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaussNewtonDDP::updateConstraintPenalties(scalar_t stateEqConstraintISE, scalar_t stateEqFinalConstraintSSE,
                                               scalar_t stateInputEqConstraintISE) {
  // state-only equality penalty
  if (stateEqConstraintISE > ddpSettings_.constraintTolerance_) {
    constraintPenaltyCoefficients_.stateEqConstrPenaltyCoeff *= ddpSettings_.constraintPenaltyIncreaseRate_;
    constraintPenaltyCoefficients_.stateEqConstrPenaltyTol = ddpSettings_.constraintTolerance_;
  }

  // final state-only equality
  if (stateEqFinalConstraintSSE > ddpSettings_.constraintTolerance_) {
    constraintPenaltyCoefficients_.stateFinalEqConstrPenaltyCoeff *= ddpSettings_.constraintPenaltyIncreaseRate_;
    constraintPenaltyCoefficients_.stateFinalEqConstrPenaltyTol = ddpSettings_.constraintTolerance_;
  }

  // state-input equality penalty
  if (stateInputEqConstraintISE < constraintPenaltyCoefficients_.stateInputEqConstrPenaltyTol) {
    // tighten tolerance
    constraintPenaltyCoefficients_.stateInputEqConstrPenaltyTol /=
        std::pow(constraintPenaltyCoefficients_.stateInputEqConstrPenaltyCoeff, 0.9);
  } else {
    // tighten tolerance & increase penalty
    constraintPenaltyCoefficients_.stateInputEqConstrPenaltyCoeff *= ddpSettings_.constraintPenaltyIncreaseRate_;
    constraintPenaltyCoefficients_.stateInputEqConstrPenaltyTol /=
        std::pow(constraintPenaltyCoefficients_.stateInputEqConstrPenaltyCoeff, 0.1);
  }
  constraintPenaltyCoefficients_.stateInputEqConstrPenaltyTol =
      std::max(constraintPenaltyCoefficients_.stateInputEqConstrPenaltyTol, ddpSettings_.constraintTolerance_);

  // display
  if (ddpSettings_.displayInfo_) {
    std::string displayText = "Augmented Lagrangian Penalty Parameters:\n";

    displayText += "    State Equality:      ";
    displayText += "    Penalty Tolerance: " + std::to_string(constraintPenaltyCoefficients_.stateEqConstrPenaltyTol);
    displayText += "    Penalty Coefficient: " + std::to_string(constraintPenaltyCoefficients_.stateEqConstrPenaltyCoeff) + '\n';

    displayText += "    Final State Equality:";
    displayText += "    Penalty Tolerance: " + std::to_string(constraintPenaltyCoefficients_.stateFinalEqConstrPenaltyTol);
    displayText += "    Penalty Coefficient: " + std::to_string(constraintPenaltyCoefficients_.stateFinalEqConstrPenaltyCoeff) + '\n';

    displayText += "    State-Input Equality:";
    displayText += "    Penalty Tolerance: " + std::to_string(constraintPenaltyCoefficients_.stateInputEqConstrPenaltyTol);
    displayText += "    Penalty Coefficient: " + std::to_string(constraintPenaltyCoefficients_.stateInputEqConstrPenaltyCoeff) + ".\n";
    this->printString(displayText);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaussNewtonDDP::swapDataToCache() {
  cachedTimeTrajectoriesStock_.swap(nominalTimeTrajectoriesStock_);
  cachedPostEventIndicesStock_.swap(nominalPostEventIndicesStock_);
  cachedStateTrajectoriesStock_.swap(nominalStateTrajectoriesStock_);
  cachedInputTrajectoriesStock_.swap(nominalInputTrajectoriesStock_);
  cachedModelDataTrajectoriesStock_.swap(modelDataTrajectoriesStock_);
  cachedModelDataEventTimesStock_.swap(modelDataEventTimesStock_);
  cachedProjectedModelDataTrajectoriesStock_.swap(projectedModelDataTrajectoriesStock_);
  cachedRiccatiModificationTrajectoriesStock_.swap(riccatiModificationTrajectoriesStock_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaussNewtonDDP::correctInitcachedNominalTrajectories() {
  // for each partition
  for (size_t i = initActivePartition_; i <= finalActivePartition_; i++) {
    if (cachedTimeTrajectoriesStock_[i].empty()) {
      cachedPostEventIndicesStock_[i] = nominalPostEventIndicesStock_[i];
      cachedTimeTrajectoriesStock_[i] = nominalTimeTrajectoriesStock_[i];
      cachedStateTrajectoriesStock_[i] = nominalStateTrajectoriesStock_[i];
      cachedInputTrajectoriesStock_[i] = nominalInputTrajectoriesStock_[i];

    } else if (cachedTimeTrajectoriesStock_[i].back() < nominalTimeTrajectoriesStock_[i].back()) {
      // find the time segment
      const scalar_t finalTime = cachedTimeTrajectoriesStock_[i].back() + numeric_traits::weakEpsilon<scalar_t>();
      const auto timeSegment = LinearInterpolation::timeSegment(finalTime, nominalTimeTrajectoriesStock_[i]);

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
          const auto indexAlpha = LinearInterpolation::timeSegment(nominalTimeTrajectoriesStock_[i][k], cachedTimeTrajectoriesStock_[i]);

          const vector_t stateCached = LinearInterpolation::interpolate(indexAlpha, cachedStateTrajectoriesStock_[i]);
          if (!stateCached.isApprox(nominalStateTrajectoriesStock_[i][k])) {
            throw std::runtime_error("The tail of the cached state trajectory is not correctly set.");
          }

          const vector_t inputCached = LinearInterpolation::interpolate(indexAlpha, cachedInputTrajectoriesStock_[i]);
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
/******************************************************************************************************/
void GaussNewtonDDP::runInit() {
  // disable Eigen multi-threading
  Eigen::setNbThreads(1);

  // initial controller rollout
  initializationTimer_.startTimer();
  try {
    constexpr size_t taskId = 0;
    constexpr scalar_t stepLength = 0.0;
    // perform a rollout
    const auto avgTimeStep = rolloutInitialTrajectory(
        nominalControllersStock_, nominalTimeTrajectoriesStock_, nominalPostEventIndicesStock_, nominalStateTrajectoriesStock_,
        nominalInputTrajectoriesStock_, modelDataTrajectoriesStock_, modelDataEventTimesStock_, taskId);
    scalar_t heuristicsValue = 0.0;
    searchStrategyPtr_->rolloutCostAndConstraints(
        linearQuadraticApproximatorPtrStock_[taskId]->systemConstraints(), linearQuadraticApproximatorPtrStock_[taskId]->costFunction(),
        *heuristicsFunctionsPtrStock_[taskId], nominalTimeTrajectoriesStock_, nominalPostEventIndicesStock_, nominalStateTrajectoriesStock_,
        nominalInputTrajectoriesStock_, modelDataTrajectoriesStock_, modelDataEventTimesStock_, heuristicsValue);

    // This is necessary for:
    // + The moving horizon (MPC) application
    // + The very first call of the algorithm where there is no previous nominal trajectories.
    correctInitcachedNominalTrajectories();

    performanceIndex_ = searchStrategyPtr_->calculateRolloutPerformanceIndex(
        *penaltyPtr_, nominalTimeTrajectoriesStock_, modelDataTrajectoriesStock_, modelDataEventTimesStock_, heuristicsValue);
    // calculates rollout merit
    performanceIndex_.merit = calculateRolloutMerit(performanceIndex_);

    // display
    if (ddpSettings_.displayInfo_) {
      std::stringstream infoDisplay;
      infoDisplay << "    [Thread " << taskId << "] - step length " << stepLength << '\n';
      infoDisplay << std::setw(4) << performanceIndex_ << '\n';
      printString(infoDisplay.str());
    }

  } catch (const std::exception& error) {
    std::string msg = "Initial controller does not generate a stable rollout.\n";
    throw std::runtime_error(msg + error.what());
  }
  initializationTimer_.endTimer();

  // update the constraint penalty coefficients
  updateConstraintPenalties(0.0, 0.0, 0.0);

  // linearizing the dynamics and quadratizing the cost function along nominal trajectories
  linearQuadraticApproximationTimer_.startTimer();
  approximateOptimalControlProblem();
  linearQuadraticApproximationTimer_.endTimer();

  // solve Riccati equations
  backwardPassTimer_.startTimer();
  avgTimeStepBP_ = solveSequentialRiccatiEquations(heuristics_.dfdxx, heuristics_.dfdx, heuristics_.f);
  backwardPassTimer_.endTimer();

  // calculate controller
  computeControllerTimer_.startTimer();
  // cache controller
  cachedControllersStock_.swap(nominalControllersStock_);
  // update nominal controller
  calculateController();
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
/******************************************************************************************************/
void GaussNewtonDDP::runIteration(bool unreliableControllerIncrement) {
  // disable Eigen multi-threading
  Eigen::setNbThreads(1);

  // finding the optimal stepLength
  searchStrategyTimer_.startTimer();
  // the controller which is designed solely based on operation trajectories possibly has invalid feedforward.
  // Therefore the expected cost/merit (calculated by the Riccati solution) is not reliable as well.
  scalar_t expectedCost = unreliableControllerIncrement ? performanceIndex_.merit : sTrajectoryStock_[initActivePartition_].front();
  runSearchStrategy(expectedCost);
  searchStrategyTimer_.endTimer();

  // update the constraint penalty coefficients
  updateConstraintPenalties(performanceIndex_.stateEqConstraintISE, performanceIndex_.stateEqFinalConstraintSSE,
                            performanceIndex_.stateInputEqConstraintISE);

  // linearizing the dynamics and quadratizing the cost function along nominal trajectories
  linearQuadraticApproximationTimer_.startTimer();
  approximateOptimalControlProblem();
  linearQuadraticApproximationTimer_.endTimer();

  // solve Riccati equations
  backwardPassTimer_.startTimer();
  avgTimeStepBP_ = solveSequentialRiccatiEquations(heuristics_.dfdxx, heuristics_.dfdx, heuristics_.f);
  backwardPassTimer_.endTimer();

  // calculate controller
  computeControllerTimer_.startTimer();
  // cache controller
  cachedControllersStock_.swap(nominalControllersStock_);
  // update nominal controller
  calculateController();
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
/******************************************************************************************************/
void GaussNewtonDDP::runImpl(scalar_t initTime, const vector_t& initState, scalar_t finalTime, const scalar_array_t& partitioningTimes) {
  const size_t numPartitions = partitioningTimes.size() - 1;

  std::vector<LinearController> noInitialController(numPartitions);
  std::vector<ControllerBase*> noInitialControllerPtrArray(numPartitions);
  for (size_t i = 0; i < numPartitions; i++) {
    noInitialControllerPtrArray[i] = &noInitialController[i];
  }

  // call the "run" method which uses the internal controllers stock (i.e. nominalControllersStock_)
  runImpl(initTime, initState, finalTime, partitioningTimes, noInitialControllerPtrArray);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaussNewtonDDP::runImpl(scalar_t initTime, const vector_t& initState, scalar_t finalTime, const scalar_array_t& partitioningTimes,
                             const std::vector<ControllerBase*>& controllersPtrStock) {
  if (ddpSettings_.displayInfo_) {
    std::cerr << "\n++++++++++++++++++++++++++++++++++++++++++++++++++++++";
    std::cerr << "\n+++++++++++++ " + ddp::toAlgorithmName(ddpSettings_.algorithm_) + " solver is initialized ++++++++++++++";
    std::cerr << "\n++++++++++++++++++++++++++++++++++++++++++++++++++++++\n";
  }

  // infeasible learning rate adjustment scheme
  if (!numerics::almost_ge(ddpSettings_.lineSearch_.maxStepLength_, ddpSettings_.lineSearch_.minStepLength_)) {
    throw std::runtime_error("The maximum learning rate is smaller than the minimum learning rate.");
  }

  if (partitioningTimes.empty()) {
    throw std::runtime_error("There should be at least one time partition.");
  }

  if (!initState.allFinite()) {
    throw std::runtime_error("DDP: Initial state is not finite (time: " + std::to_string(initTime) + " [sec]).");
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
      auto linearCtrlPtr = dynamic_cast<LinearController*>(controllersStock_i);
      if (linearCtrlPtr == nullptr) {
        throw std::runtime_error("GaussNewtonDDP::run -- controller must be a LinearController.");
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
    std::cerr << "\nRewind Counter: " << rewindCounter_ << "\n";
    std::cerr << ddp::toAlgorithmName(ddpSettings_.algorithm_) + " solver starts from initial time " << initTime << " to final time "
              << finalTime << ".\n";
    std::cerr << this->getReferenceManager().getModeSchedule() << "\n";
  }

  initState_ = initState;
  initTime_ = initTime;
  finalTime_ = finalTime;
  const auto initIteration = totalNumIterations_;

  performanceIndexHistory_.clear();

  // check if after the truncation the internal controller is empty
  bool unreliableControllerIncrement = false;
  for (const auto& controller : nominalControllersStock_) {
    unreliableControllerIncrement = unreliableControllerIncrement || controller.empty();
  }

  // initialize the search strategy
  searchStrategyPtr_->initalize(initTime_, initState_, finalTime_, partitioningTimes_, initActivePartition_, finalActivePartition_);

  // set cost desired trajectories
  for (size_t i = 0; i < ddpSettings_.nThreads_; i++) {
    const auto& targetTrajectories = this->getReferenceManager().getTargetTrajectories();
    heuristicsFunctionsPtrStock_[i]->setTargetTrajectoriesPtr(&targetTrajectories);
    linearQuadraticApproximatorPtrStock_[i]->costFunction().setTargetTrajectoriesPtr(&targetTrajectories);
  }

  // display
  if (ddpSettings_.displayInfo_) {
    std::cerr << "\n###################";
    std::cerr << "\n#### Iteration " << (totalNumIterations_ - initIteration) << " (Dynamics might have been violated)";
    std::cerr << "\n###################\n";
  }

  // cache the nominal trajectories before the new rollout (time, state, input, ...)
  swapDataToCache();

  // run DDP initializer and update the member variables
  runInit();

  // increment iteration counter
  totalNumIterations_++;

  // convergence variables of the main loop
  bool isConverged = false;
  std::string convergenceInfo;

  // DDP main loop
  while (!isConverged && (totalNumIterations_ - initIteration) < ddpSettings_.maxNumIterations_) {
    // display the iteration's input update norm (before caching the old nominals)
    if (ddpSettings_.displayInfo_) {
      std::cerr << "\n###################";
      std::cerr << "\n#### Iteration " << (totalNumIterations_ - initIteration);
      std::cerr << "\n###################\n";

      scalar_t maxDeltaUffNorm, maxDeltaUeeNorm;
      calculateControllerUpdateMaxNorm(maxDeltaUffNorm, maxDeltaUeeNorm);
      std::cerr << "max feedforward norm: " << maxDeltaUffNorm << "\n";
    }

    // cache the nominal trajectories before the new rollout (time, state, input, ...)
    swapDataToCache();
    performanceIndexHistory_.push_back(performanceIndex_);

    // run the an iteration of the DDP algorithm and update the member variables
    runIteration(unreliableControllerIncrement);

    // increment iteration counter
    totalNumIterations_++;

    // check convergence
    std::tie(isConverged, convergenceInfo) =
        searchStrategyPtr_->checkConvergence(unreliableControllerIncrement, performanceIndexHistory_.back(), performanceIndex_);
    unreliableControllerIncrement = false;
  }  // end of while loop

  // display the final iteration's input update norm (before caching the old nominals)
  if (ddpSettings_.displayInfo_) {
    std::cerr << "\n###################";
    std::cerr << "\n#### Final Rollout";
    std::cerr << "\n###################\n";

    scalar_t maxDeltaUffNorm, maxDeltaUeeNorm;
    calculateControllerUpdateMaxNorm(maxDeltaUffNorm, maxDeltaUeeNorm);
    std::cerr << "max feedforward norm: " << maxDeltaUffNorm << "\n";
  }

  // cache the nominal trajectories before the new rollout (time, state, input, ...)
  swapDataToCache();
  performanceIndexHistory_.push_back(performanceIndex_);

  // finding the final optimal stepLength and getting the optimal trajectories and controller
  searchStrategyTimer_.startTimer();
  runSearchStrategy(sTrajectoryStock_[initActivePartition_].front());
  searchStrategyTimer_.endTimer();

  performanceIndexHistory_.push_back(performanceIndex_);

  // display
  if (ddpSettings_.displayInfo_ || ddpSettings_.displayShortSummary_) {
    std::cerr << "\n++++++++++++++++++++++++++++++++++++++++++++++++++++++";
    std::cerr << "\n++++++++++++++ " + ddp::toAlgorithmName(ddpSettings_.algorithm_) + " solver has terminated +++++++++++++";
    std::cerr << "\n++++++++++++++++++++++++++++++++++++++++++++++++++++++\n";
    std::cerr << "Time Period:          [" << initTime_ << " ," << finalTime_ << "]\n";
    std::cerr << "Number of Iterations: " << (totalNumIterations_ - initIteration) << " out of " << ddpSettings_.maxNumIterations_ << "\n";

    printRolloutInfo();

    if (isConverged) {
      std::cerr << convergenceInfo << std::endl;
    } else if (totalNumIterations_ - initIteration == ddpSettings_.maxNumIterations_) {
      std::cerr << "The algorithm has terminated as: \n";
      std::cerr << "    * The maximum number of iterations (i.e., " << ddpSettings_.maxNumIterations_ << ") has reached." << std::endl;
    } else {
      std::cerr << "The algorithm has terminated for an unknown reason!" << std::endl;
    }
  }
}

}  // namespace ocs2
