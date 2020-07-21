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

#include <ocs2_core/control/FeedforwardController.h>
#include <ocs2_core/misc/LinearAlgebra.h>

#include <ocs2_ddp/GaussNewtonDDP.h>
#include <ocs2_ddp/HessianCorrection.h>
#include <ocs2_ddp/riccati_equations/RiccatiModificationInterpolation.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
GaussNewtonDDP::GaussNewtonDDP(size_t stateDim, size_t inputDim, const RolloutBase* rolloutPtr, const SystemDynamicsBase* systemDynamicsPtr,
                               const ConstraintBase* systemConstraintsPtr, const CostFunctionBase* costFunctionPtr,
                               const SystemOperatingTrajectoriesBase* operatingTrajectoriesPtr, const DDP_Settings& ddpSettings,
                               const CostFunctionBase* heuristicsFunctionPtr, const char* algorithmName)
    : Solver_BASE(),
      stateDim_(stateDim),
      inputDim_(inputDim),
      ddpSettings_(ddpSettings),
      threadPool_(ddpSettings.nThreads_, ddpSettings.threadPriority_),
      algorithmName_(algorithmName),
      rewindCounter_(0),
      iteration_(0) {
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
    operatingTrajectoriesRolloutPtrStock_.emplace_back(new OperatingTrajectoriesRollout(*operatingTrajectoriesPtr, rolloutPtr->settings()));

    // initialize LQ approximator
    bool makePsdWillBePerformedLater = ddpSettings_.lineSearch_.hessianCorrectionStrategy_ != hessian_correction::Strategy::DIAGONAL_SHIFT;
    linearQuadraticApproximatorPtrStock_.emplace_back(new LinearQuadraticApproximator(
        *systemDynamicsPtr, *systemConstraintsPtr, *costFunctionPtr, ddpSettings_.checkNumericalStability_, makePsdWillBePerformedLater));

    // initialize heuristics functions
    if (heuristicsFunctionPtr != nullptr) {
      heuristicsFunctionsPtrStock_.emplace_back(heuristicsFunctionPtr->clone());
    } else {  // use the cost function if no heuristics function is defined
      heuristicsFunctionsPtrStock_.emplace_back(costFunctionPtr->clone());
    }

    // initialize penalty functions
    penaltyPtrStock_.emplace_back(new RelaxedBarrierPenalty(ddpSettings_.inequalityConstraintMu_, ddpSettings_.inequalityConstraintDelta_));

  }  // end of i loop

  // initialize Augmented Lagrangian parameters
  initializeConstraintPenalties();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
GaussNewtonDDP::~GaussNewtonDDP() {
  auto forwardPassTotal = forwardPassTimer_.getTotalInMilliseconds();
  auto linearQuadraticApproximationTotal = linearQuadraticApproximationTimer_.getTotalInMilliseconds();
  auto backwardPassTotal = backwardPassTimer_.getTotalInMilliseconds();
  auto computeControllerTotal = computeControllerTimer_.getTotalInMilliseconds();
  auto searchStrategyTotal = searchStrategyTimer_.getTotalInMilliseconds();

  auto benchmarkTotal =
      forwardPassTotal + linearQuadraticApproximationTotal + backwardPassTotal + computeControllerTotal + searchStrategyTotal;

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
    std::cerr << "\tSearch Strategy    :\t" << searchStrategyTimer_.getAverageInMilliseconds() << " [ms] \t\t("
              << searchStrategyTotal / benchmarkTotal * 100 << "%)" << std::endl;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaussNewtonDDP::reset() {
  iteration_ = 0;
  rewindCounter_ = 0;

  performanceIndexHistory_.clear();

  // reset Levenberg_Marquardt variables
  levenbergMarquardtModule_ = LevenbergMarquardtModule();

  // initialize Augmented Lagrangian parameters
  initializeConstraintPenalties();

  useParallelRiccatiSolverFromInitItr_ = false;

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
  forwardPassTimer_.reset();
  linearQuadraticApproximationTimer_.reset();
  backwardPassTimer_.reset();
  computeControllerTimer_.reset();
  searchStrategyTimer_.reset();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
size_t GaussNewtonDDP::getNumIterations() const {
  return iteration_;
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
    primalSolutionPtr->controllerPtr_.reset(new LinearController(stateDim_, inputDim_));
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
        new FeedforwardController(stateDim_, inputDim_, primalSolutionPtr->timeTrajectory_, primalSolutionPtr->inputTrajectory_));
  }

  // fill mode schedule
  primalSolutionPtr->modeSchedule_ = this->getModeSchedule();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t GaussNewtonDDP::getValueFunction(scalar_t time, const vector_t& state) const {
  const auto partition = lookup::findBoundedActiveIntervalInTimeArray(partitioningTimes_, time);

  matrix_t Sm;
  const auto indexAlpha = LinearInterpolation::interpolate(time, Sm, &SsTimeTrajectoryStock_[partition], &SmTrajectoryStock_[partition]);

  vector_t Sv;
  LinearInterpolation::interpolate(indexAlpha, Sv, &SvTrajectoryStock_[partition]);

  scalar_t s;
  LinearInterpolation::interpolate(indexAlpha, s, &sTrajectoryStock_[partition]);

  vector_t xNominal;
  LinearInterpolation::interpolate(time, xNominal, &nominalTimeTrajectoriesStock_[partition], &nominalStateTrajectoriesStock_[partition]);

  vector_t deltaX = state - xNominal;

  return s + deltaX.dot(Sv) + 0.5 * deltaX.dot(Sm * deltaX);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaussNewtonDDP::getValueFunctionStateDerivative(scalar_t time, const vector_t& state, vector_t& Vx) const {
  const auto partition = lookup::findBoundedActiveIntervalInTimeArray(partitioningTimes_, time);

  matrix_t Sm;
  const auto indexAlpha = LinearInterpolation::interpolate(time, Sm, &SsTimeTrajectoryStock_[partition], &SmTrajectoryStock_[partition]);

  // Sv
  LinearInterpolation::interpolate(indexAlpha, Vx, &SvTrajectoryStock_[partition]);

  vector_t xNominal;
  LinearInterpolation::interpolate(time, xNominal, &nominalTimeTrajectoriesStock_[partition], &nominalStateTrajectoriesStock_[partition]);

  vector_t deltaX = state - xNominal;
  Vx += Sm * deltaX;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaussNewtonDDP::getStateInputEqualityConstraintLagrangian(scalar_t time, const vector_t& state, vector_t& nu) const {
  const auto activeSubsystem = lookup::findBoundedActiveIntervalInTimeArray(partitioningTimes_, time);

  const auto indexAlpha = LinearInterpolation::timeSegment(time, &nominalTimeTrajectoriesStock_[activeSubsystem]);

  vector_t xNominal;
  LinearInterpolation::interpolate(indexAlpha, xNominal, &nominalStateTrajectoriesStock_[activeSubsystem]);

  matrix_t Bm;
  ModelData::interpolate(indexAlpha, Bm, &modelDataTrajectoriesStock_[activeSubsystem], ModelData::dynamics_dfdu);

  matrix_t Pm;
  ModelData::interpolate(indexAlpha, Pm, &modelDataTrajectoriesStock_[activeSubsystem], ModelData::cost_dfdux);

  vector_t Rv;
  ModelData::interpolate(indexAlpha, Rv, &modelDataTrajectoriesStock_[activeSubsystem], ModelData::cost_dfdu);

  vector_t EvProjected;
  ModelData::interpolate(indexAlpha, EvProjected, &projectedModelDataTrajectoriesStock_[activeSubsystem], ModelData::stateInputEqConstr_f);

  matrix_t CmProjected;
  ModelData::interpolate(indexAlpha, CmProjected, &projectedModelDataTrajectoriesStock_[activeSubsystem],
                         ModelData::stateInputEqConstr_dfdx);

  matrix_t Hm;
  riccati_modification::interpolate(indexAlpha, Hm, &riccatiModificationTrajectoriesStock_[activeSubsystem],
                                    riccati_modification::hamiltonianHessian);

  matrix_t DmDagger;
  riccati_modification::interpolate(indexAlpha, DmDagger, &riccatiModificationTrajectoriesStock_[activeSubsystem],
                                    riccati_modification::constraintRangeProjector);

  vector_t costate;
  getValueFunctionStateDerivative(time, state, costate);

  vector_t deltaX = state - xNominal;
  matrix_t DmDaggerTransHm = DmDagger.transpose() * Hm;

  nu = DmDaggerTransHm * (CmProjected * deltaX + EvProjected) - DmDagger.transpose() * (Rv + Pm * deltaX + Bm.transpose() * costate);

  //  alternative computation
  //  nu = DmDagger.transpose() * (Hm * DmDagger.transpose() * CmProjected * deltaX - Rv - Bm.transpose() * costate);
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
      SmFinalStock_[i].setZero(stateDim_, stateDim_);
      SvFinalStock_[i].setZero(stateDim_);
      sFinalStock_[i] = 0.0;
      xFinalStock_[i].setZero(stateDim_);
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
const unsigned long long int& GaussNewtonDDP::getRewindCounter() const {
  return rewindCounter_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
DDP_Settings& GaussNewtonDDP::ddpSettings() {
  return ddpSettings_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
const DDP_Settings& GaussNewtonDDP::ddpSettings() const {
  return ddpSettings_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaussNewtonDDP::useParallelRiccatiSolverFromInitItr(bool flag) {
  useParallelRiccatiSolverFromInitItr_ = flag;
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
  if (nominalControllersStock_.size() > 0) {
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
  nominalControllersStock_.resize(numPartitions, LinearController(stateDim_, inputDim_));

  nominalTimeTrajectoriesStock_.resize(numPartitions);
  nominalPostEventIndicesStock_.resize(numPartitions);
  nominalStateTrajectoriesStock_.resize(numPartitions);
  nominalInputTrajectoriesStock_.resize(numPartitions);

  cachedControllersStock_.resize(numPartitions, LinearController(stateDim_, inputDim_));
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
void GaussNewtonDDP::distributeWork() {
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
/******************************************************************************************************/
void GaussNewtonDDP::runParallel(std::function<void(void)> taskFunction, size_t N) {
  threadPool_.runParallel([&](int) { taskFunction(); }, N);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t GaussNewtonDDP::rolloutTrajectory(std::vector<LinearController>& controllersStock, scalar_array2_t& timeTrajectoriesStock,
                                           size_array2_t& postEventIndicesStock, vector_array2_t& stateTrajectoriesStock,
                                           vector_array2_t& inputTrajectoriesStock,
                                           std::vector<std::vector<ModelDataBase>>& modelDataTrajectoriesStock,
                                           size_t workerIndex /*= 0*/) {
  const scalar_array_t& eventTimes = this->getModeSchedule().eventTimes;

  if (controllersStock.size() != numPartitions_) {
    throw std::runtime_error("controllersStock has less controllers then the number of subsystems");
  }

  // Prepare outputs
  timeTrajectoriesStock.resize(numPartitions_);
  postEventIndicesStock.resize(numPartitions_);
  stateTrajectoriesStock.resize(numPartitions_);
  inputTrajectoriesStock.resize(numPartitions_);
  modelDataTrajectoriesStock.resize(numPartitions_);
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
    std::cerr << "[GaussNewtonDDP::rolloutTrajectory] for t = [" << initTime_ << ", " << finalTime_ << "]\n"
              << "\tcontroller available till t = " << controllerAvailableTill << "\n"
              << "\twill use controller until t = " << useControllerTill << std::endl;
  }

  size_t numSteps = 0;
  vector_t xCurrent = initState_;
  for (size_t i = initActivePartition_; i < finalActivePartition_ + 1; i++) {
    // Start and end of rollout segment
    const scalar_t t0 = (i == initActivePartition_) ? initTime_ : partitioningTimes_[i];
    const scalar_t tf = (i == finalActivePartition_) ? finalTime_ : partitioningTimes_[i + 1];

    // Divide the rollout segment in controller rollout and operating points
    const std::pair<scalar_t, scalar_t> controllerRolloutFromTo{t0, std::max(t0, std::min(useControllerTill, tf))};
    std::pair<scalar_t, scalar_t> operatingPointsFromTo{controllerRolloutFromTo.second, tf};

    if (ddpSettings_.debugPrintRollout_) {
      std::cerr << "[GaussNewtonDDP::rolloutTrajectory] partition " << i << " for t = [" << t0 << ", " << tf << "]" << std::endl;
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
      xCurrent = dynamicsForwardRolloutPtrStock_[workerIndex]->run(
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
      vector_array_t stateTrajectoryTail;
      vector_array_t inputTrajectoryTail;
      xCurrent = operatingTrajectoriesRolloutPtrStock_[workerIndex]->run(
          operatingPointsFromTo.first, xCurrent, operatingPointsFromTo.second, nullptr, eventTimes, timeTrajectoryTail,
          eventsPastTheEndIndecesTail, stateTrajectoryTail, inputTrajectoryTail);

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
      modelDataTrajectoriesStock[i][k].stateDim_ = stateDim_;
      modelDataTrajectoriesStock[i][k].inputDim_ = inputDim_;
      modelDataTrajectoriesStock[i][k].dynamicsBias_.setZero(stateDim_);
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
      RolloutBase::display(timeTrajectoriesStock[i], postEventIndicesStock[i], stateTrajectoriesStock[i], &inputTrajectoriesStock[i]);
    }
  }
  // average time step
  return (finalTime_ - initTime_) / numSteps;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaussNewtonDDP::printRolloutInfo() const {
  std::cerr << performanceIndex_ << '\n';
  std::cerr << "forward pass average time step:  " << avgTimeStepFP_ * 1e+3 << " [ms]." << std::endl;
  std::cerr << "backward pass average time step: " << avgTimeStepBP_ * 1e+3 << " [ms]." << std::endl;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaussNewtonDDP::calculateRolloutConstraintsISE(const scalar_array2_t& timeTrajectoriesStock,
                                                    const size_array2_t& postEventIndicesStock,
                                                    const vector_array2_t& stateTrajectoriesStock,
                                                    const vector_array2_t& inputTrajectoriesStock, scalar_t& stateInputEqConstraintISE,
                                                    scalar_t& stateEqConstraintISE, scalar_t& stateEqFinalConstraintSSE,
                                                    scalar_t& inequalityConstraintISE, scalar_t& inequalityConstraintPenalty,
                                                    size_t workerIndex /*= 0*/) {
  size_t maxSize = 0;
  for (size_t i = 0; i < numPartitions_; i++) {
    maxSize = std::max(maxSize, timeTrajectoriesStock[i].size());
  }

  scalar_array_t stateEqualityNorm2Trajectory(maxSize);
  scalar_array_t stateFinalEqualityNorm2Trajectory(1);
  scalar_array_t stateInputEqualityNorm2Trajectory(maxSize);
  scalar_array_t inequalityNorm2Trajectory(maxSize);
  scalar_array_t inequalityPenaltyTrajectory(maxSize);

  stateEqConstraintISE = 0.0;
  stateInputEqConstraintISE = 0.0;
  inequalityConstraintPenalty = 0.0;
  inequalityConstraintISE = 0.0;
  stateEqFinalConstraintSSE = 0.0;
  for (size_t i = 0; i < numPartitions_; i++) {
    stateEqualityNorm2Trajectory.clear();
    stateFinalEqualityNorm2Trajectory.clear();
    stateInputEqualityNorm2Trajectory.clear();
    inequalityNorm2Trajectory.clear();
    inequalityPenaltyTrajectory.clear();

    auto eventsPastTheEndItr = postEventIndicesStock[i].begin();
    // compute constraint1 trajectory for subsystem i
    for (size_t k = 0; k < timeTrajectoriesStock[i].size(); k++) {
      const auto t = timeTrajectoriesStock[i][k];
      const auto& x = stateTrajectoriesStock[i][k];
      const auto& u = inputTrajectoriesStock[i][k];
      auto& systemConstraints = linearQuadraticApproximatorPtrStock_[workerIndex]->systemConstraints();

      // state equality constraint
      stateEqualityNorm2Trajectory.emplace_back(systemConstraints.stateEqualityConstraint(t, x).squaredNorm());

      // state-input equality constraint
      stateInputEqualityNorm2Trajectory.emplace_back(systemConstraints.stateInputEqualityConstraint(t, x, u).squaredNorm());

      // inequality constraints
      vector_t HvIneq = systemConstraints.inequalityConstraint(t, x, u);
      inequalityPenaltyTrajectory.emplace_back(penaltyPtrStock_[workerIndex]->penaltyCost(HvIneq));
      inequalityNorm2Trajectory.emplace_back(penaltyPtrStock_[workerIndex]->constraintViolationSquaredNorm(HvIneq));

      // switching time constraints
      if (eventsPastTheEndItr != postEventIndicesStock[i].end() && k + 1 == *eventsPastTheEndItr) {
        stateFinalEqualityNorm2Trajectory.emplace_back(systemConstraints.finalStateEqualityConstraint(t, x).squaredNorm());
        eventsPastTheEndItr++;
      }

    }  // end of k loop

    // state equality constraint's ISE
    stateEqConstraintISE += trapezoidalIntegration(timeTrajectoriesStock[i], stateEqualityNorm2Trajectory);

    // calculate state-input equality constraint's ISE
    stateInputEqConstraintISE += trapezoidalIntegration(timeTrajectoriesStock[i], stateInputEqualityNorm2Trajectory);

    // inequality constraint
    inequalityConstraintPenalty += trapezoidalIntegration(timeTrajectoriesStock[i], inequalityPenaltyTrajectory);
    inequalityConstraintISE += trapezoidalIntegration(timeTrajectoriesStock[i], inequalityNorm2Trajectory);

    // final state equality constraint's SSE
    for (size_t ke = 0; ke < postEventIndicesStock[i].size(); ke++) {
      stateEqFinalConstraintSSE += stateFinalEqualityNorm2Trajectory[ke];
    }  // end of k loop

  }  // end of i loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t GaussNewtonDDP::calculateRolloutCost(const scalar_array2_t& timeTrajectoriesStock, const size_array2_t& postEventIndicesStock,
                                              const vector_array2_t& stateTrajectoriesStock, const vector_array2_t& inputTrajectoriesStock,
                                              size_t workerIndex /*= 0*/) {
  // set desired trajectories
  auto& costFunction = linearQuadraticApproximatorPtrStock_[workerIndex]->costFunction();
  costFunction.setCostDesiredTrajectoriesPtr(&this->getCostDesiredTrajectories());

  scalar_array_t finalCosts;
  scalar_array_t costTrajectory;
  scalar_t totalCost = 0.0;
  for (size_t i = 0; i < numPartitions_; i++) {
    finalCosts.clear();
    costTrajectory.resize(timeTrajectoriesStock[i].size());

    auto eventsPastTheEndItr = postEventIndicesStock[i].begin();
    for (size_t k = 0; k < timeTrajectoriesStock[i].size(); k++) {
      // get intermediate cost for next time step
      costTrajectory[k] = costFunction.cost(timeTrajectoriesStock[i][k], stateTrajectoriesStock[i][k], inputTrajectoriesStock[i][k]);

      // final cost at switching times
      if (eventsPastTheEndItr != postEventIndicesStock[i].end() && k + 1 == *eventsPastTheEndItr) {
        finalCosts.emplace_back(costFunction.finalCost(timeTrajectoriesStock[i][k], stateTrajectoriesStock[i][k]));
        eventsPastTheEndItr++;
      }
    }  // end of k loop

    // total cost
    totalCost += trapezoidalIntegration(timeTrajectoriesStock[i], costTrajectory);

    // final cost
    for (size_t ke = 0; ke < postEventIndicesStock[i].size(); ke++) {
      totalCost += finalCosts[ke];
    }  // end of k loop
  }    // end of i loop

  // calculate the Heuristics function at the final time
  // set desired trajectories
  auto& heuristicsFunction = heuristicsFunctionsPtrStock_[workerIndex];
  heuristicsFunction->setCostDesiredTrajectoriesPtr(&this->getCostDesiredTrajectories());
  // set state-input
  totalCost += heuristicsFunction->finalCost(timeTrajectoriesStock[finalActivePartition_].back(),
                                             stateTrajectoriesStock[finalActivePartition_].back());

  return totalCost;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t GaussNewtonDDP::performFullRollout(size_t workerIndex, scalar_t stepLength, std::vector<LinearController>& controllersStock,
                                            scalar_array2_t& timeTrajectoriesStock, size_array2_t& postEventIndicesStock,
                                            vector_array2_t& stateTrajectoriesStock, vector_array2_t& inputTrajectoriesStock,
                                            std::vector<std::vector<ModelDataBase>>& modelDataTrajectoriesStock,
                                            PerformanceIndex& performanceIndex) {
  // modifying uff by local increments
  if (!numerics::almost_eq(stepLength, 0.0)) {
    for (auto& controller : controllersStock) {
      for (size_t k = 0; k < controller.size(); k++) {
        controller.biasArray_[k] += stepLength * controller.deltaBiasArray_[k];
      }
    }
  }

  scalar_t avgTimeStepFP = 0.0;
  try {
    // perform a rollout
    avgTimeStepFP = rolloutTrajectory(controllersStock, timeTrajectoriesStock, postEventIndicesStock, stateTrajectoriesStock,
                                      inputTrajectoriesStock, modelDataTrajectoriesStock, workerIndex);

    // calculate rollout constraints
    calculateRolloutConstraintsISE(timeTrajectoriesStock, postEventIndicesStock, stateTrajectoriesStock, inputTrajectoriesStock,
                                   performanceIndex.stateInputEqConstraintISE, performanceIndex.stateEqConstraintISE,
                                   performanceIndex.stateEqFinalConstraintSSE, performanceIndex.inequalityConstraintISE,
                                   performanceIndex.inequalityConstraintPenalty, workerIndex);
    // calculate rollout cost
    performanceIndex.totalCost =
        calculateRolloutCost(timeTrajectoriesStock, postEventIndicesStock, stateTrajectoriesStock, inputTrajectoriesStock, workerIndex);

    // calculates rollout merit
    calculateRolloutMerit(performanceIndex);

    // display
    if (ddpSettings_.displayInfo_) {
      std::stringstream linesearchDisplay;
      linesearchDisplay << "    [Thread " << workerIndex << "] - step length " << stepLength << '\n';
      linesearchDisplay << std::setw(4) << performanceIndex << '\n';
      linesearchDisplay << "    forward pass average time step: " << avgTimeStepFP * 1e+3 << " [ms].\n";
      Solver_BASE::printString(linesearchDisplay.str());
    }

  } catch (const std::exception& error) {
    performanceIndex.merit = std::numeric_limits<scalar_t>::max();
    performanceIndex.totalCost = std::numeric_limits<scalar_t>::max();
    if (ddpSettings_.displayInfo_) {
      Solver_BASE::printString("    [Thread " + std::to_string(workerIndex) + "] rollout with step length " + std::to_string(stepLength) +
                               " is terminated: " + error.what());
    }
  }

  return avgTimeStepFP;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaussNewtonDDP::lineSearch(LineSearchModule& lineSearchModule) {
  // number of line search iterations (the if statements order is important)
  size_t maxNumOfLineSearches = 0;
  if (numerics::almost_eq(ddpSettings_.lineSearch_.minStepLength_, ddpSettings_.lineSearch_.maxStepLength_)) {
    maxNumOfLineSearches = 1;
  } else if (ddpSettings_.lineSearch_.maxStepLength_ < ddpSettings_.lineSearch_.minStepLength_) {
    maxNumOfLineSearches = 0;
  } else {
    const auto ratio = ddpSettings_.lineSearch_.minStepLength_ / ddpSettings_.lineSearch_.maxStepLength_;
    maxNumOfLineSearches = static_cast<size_t>(
        std::log(ratio + OCS2NumericTraits<scalar_t>::limitEpsilon()) / std::log(ddpSettings_.lineSearch_.contractionRate_) + 1);
  }

  // perform a rollout with steplength zero.
  const size_t threadId = 0;
  const scalar_t stepLength = 0.0;
  scalar_t avgTimeStepFP =
      performFullRollout(threadId, stepLength, nominalControllersStock_, nominalTimeTrajectoriesStock_, nominalPostEventIndicesStock_,
                         nominalStateTrajectoriesStock_, nominalInputTrajectoriesStock_, modelDataTrajectoriesStock_, performanceIndex_);
  if (performanceIndex_.merit == std::numeric_limits<scalar_t>::max()) {
    throw std::runtime_error("DDP feedback gains do not generate a stable rollout.");
  }

  // compute average time step of forward rollout
  avgTimeStepFP_ = 0.9 * avgTimeStepFP_ + 0.1 * avgTimeStepFP;

  // initialize lineSearchModule
  lineSearchModule.baselineMerit = performanceIndex_.merit;
  lineSearchModule.stepLengthStar = 0.0;
  lineSearchModule.initControllersStock = nominalControllersStock_;  // this will serve to init the workers
  lineSearchModule.alphaExpNext = 0;
  lineSearchModule.alphaProcessed = std::vector<bool>(maxNumOfLineSearches, false);

  nextTaskId_ = 0;
  std::function<void(void)> task = [this, &lineSearchModule] { lineSearchTask(lineSearchModule); };
  runParallel(task, ddpSettings_.nThreads_);

  // revitalize all integrators
  for (auto& rolloutPtr : dynamicsForwardRolloutPtrStock_) {
    rolloutPtr->reactivateRollout();
  }

  // clear the feedforward increments
  for (auto& controller : nominalControllersStock_) {
    controller.deltaBiasArray_.clear();
  }

  // display
  if (ddpSettings_.displayInfo_) {
    std::cerr << "The chosen step length is: " + std::to_string(lineSearchModule.stepLengthStar) << std::endl;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaussNewtonDDP::lineSearchTask(LineSearchModule& lineSearchModule) {
  size_t taskId = nextTaskId_++;  // assign task ID (atomic)

  // local search forward simulation's variables
  PerformanceIndex performanceIndex;
  std::vector<LinearController> controllersStock(numPartitions_, LinearController(stateDim_, inputDim_));
  scalar_array2_t timeTrajectoriesStock(numPartitions_);
  size_array2_t postEventIndicesStock(numPartitions_);
  vector_array2_t stateTrajectoriesStock(numPartitions_);
  vector_array2_t inputTrajectoriesStock(numPartitions_);
  std::vector<std::vector<ModelDataBase>> modelDataTrajectoriesStock(numPartitions_);

  while (true) {
    size_t alphaExp = lineSearchModule.alphaExpNext++;
    scalar_t stepLength = ddpSettings_.lineSearch_.maxStepLength_ * std::pow(ddpSettings_.lineSearch_.contractionRate_, alphaExp);

    /*
     * finish this thread's task since the learning rate is less than the minimum learning rate.
     * This means that the all the line search tasks are already processed or they are under
     * process in other threads.
     */
    if (!numerics::almost_ge(stepLength, ddpSettings_.lineSearch_.minStepLength_)) {
      break;
    }

    // skip if the current learning rate is less than the best candidate
    if (stepLength < lineSearchModule.stepLengthStar) {
      // display
      if (ddpSettings_.displayInfo_) {
        std::string linesearchDisplay;
        linesearchDisplay = "    [Thread " + std::to_string(taskId) + "] rollout with step length " + std::to_string(stepLength) +
                            " is skipped: A larger learning rate is already found!";
        Solver_BASE::printString(linesearchDisplay);
      }
      break;
    }

    // do a line search
    controllersStock = lineSearchModule.initControllersStock;
    scalar_t avgTimeStepFP =
        performFullRollout(taskId, stepLength, controllersStock, timeTrajectoriesStock, postEventIndicesStock, stateTrajectoriesStock,
                           inputTrajectoriesStock, modelDataTrajectoriesStock, performanceIndex);

    bool terminateLinesearchTasks = false;
    {
      std::lock_guard<std::mutex> lock(lineSearchModule.lineSearchResultMutex);

      /*
       * based on the "Armijo backtracking" step length selection policy:
       * cost should be better than the baseline cost but learning rate should
       * be as high as possible. This is equivalent to a single core line search.
       */
      const bool progressCondition = performanceIndex.merit < (lineSearchModule.baselineMerit * (1.0 - 1e-3 * stepLength));
      const bool armijoCondition = performanceIndex.merit < (lineSearchModule.baselineMerit - ddpSettings_.lineSearch_.armijoCoefficient_ *
                                                                                                  stepLength * nominalControllerUpdateIS_);
      if (armijoCondition && stepLength > lineSearchModule.stepLengthStar) {
        lineSearchModule.stepLengthStar = stepLength;
        performanceIndex_ = performanceIndex;
        swap(nominalControllersStock_, controllersStock);
        nominalTimeTrajectoriesStock_.swap(timeTrajectoriesStock);
        nominalPostEventIndicesStock_.swap(postEventIndicesStock);
        nominalStateTrajectoriesStock_.swap(stateTrajectoriesStock);
        nominalInputTrajectoriesStock_.swap(inputTrajectoriesStock);
        modelDataTrajectoriesStock_.swap(modelDataTrajectoriesStock);

        // whether to stop all other thread.
        terminateLinesearchTasks = true;
        for (size_t i = 0; i < alphaExp; i++) {
          if (!lineSearchModule.alphaProcessed[i]) {
            terminateLinesearchTasks = false;
            break;
          }
        }  // end of i loop

      }  // end of if

      lineSearchModule.alphaProcessed[alphaExp] = true;

    }  // end lock

    // kill other ongoing line search tasks
    if (terminateLinesearchTasks) {
      for (auto& rolloutPtr : dynamicsForwardRolloutPtrStock_) {
        rolloutPtr->abortRollout();
      }
      if (ddpSettings_.displayInfo_) {
        Solver_BASE::printString("    LS: interrupt other rollout's integrations.");
      }
      break;
    }

  }  // end of while loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaussNewtonDDP::calculateRolloutMerit(PerformanceIndex& performanceIndex) const {
  // total cost
  performanceIndex.merit = performanceIndex.totalCost;

  // intermediate state-only equality constraints
  performanceIndex.merit += constraintPenaltyCoefficients_.stateEqualityPenaltyCoeff * std::sqrt(performanceIndex.stateEqConstraintISE);

  // final state-only equality constraints
  performanceIndex.merit +=
      constraintPenaltyCoefficients_.stateEqualityFinalPenaltyCoeff * std::sqrt(performanceIndex.stateEqFinalConstraintSSE);

  // intermediate state-input equality constraints
  performanceIndex.merit +=
      constraintPenaltyCoefficients_.stateInputEqualityPenaltyCoeff * std::sqrt(performanceIndex.stateInputEqConstraintISE);

  // intermediate inequality constraints
  performanceIndex.merit += performanceIndex.inequalityConstraintPenalty;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaussNewtonDDP::levenbergMarquardt(LevenbergMarquardtModule& levenbergMarquardtModule) {
  const size_t taskId = 0;

  throw std::runtime_error("Implemented in the next PR");
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t GaussNewtonDDP::solveSequentialRiccatiEquationsImpl(const matrix_t& SmFinal, const vector_t& SvFinal, const scalar_t& sFinal) {
  SmFinalStock_[finalActivePartition_] = SmFinal;
  SvFinalStock_[finalActivePartition_] = SvFinal;
  sFinalStock_[finalActivePartition_] = sFinal;
  xFinalStock_[finalActivePartition_] = nominalStateTrajectoriesStock_[finalActivePartition_].back();

  // solve it sequentially for the first time when useParallelRiccatiSolverFromInitItr_ is false
  if (iteration_ == 0 && !useParallelRiccatiSolverFromInitItr_) {
    nextTaskId_ = 0;
    for (int i = 0; i < ddpSettings_.nThreads_; i++) {
      riccatiSolverTask();
    }
  } else {  // solve it in parallel if useParallelRiccatiSolverFromInitItr_ is true
    nextTaskId_ = 0;
    std::function<void(void)> task = [this] { riccatiSolverTask(); };
    runParallel(task, ddpSettings_.nThreads_);
  }

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
            std::cerr << "s[" << SsTimeTrajectoryStock_[i][kp] << "]:\t" << sTrajectoryStock_[i][kp] << std::endl;
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
void GaussNewtonDDP::riccatiSolverTask() {
  size_t taskId = nextTaskId_++;  // assign task ID (atomic)

  for (int i = endingIndicesRiccatiWorker_[taskId]; i >= startingIndicesRiccatiWorker_[taskId]; i--) {
    // for inactive subsystems
    if (i < initActivePartition_ || i > finalActivePartition_) {
      SsTimeTrajectoryStock_[i].clear();
      SsNormalizedTimeTrajectoryStock_[i].clear();
      SsNormalizedEventsPastTheEndIndecesStock_[i].clear();
      SmTrajectoryStock_[i].clear();
      SvTrajectoryStock_[i].clear();
      sTrajectoryStock_[i].clear();

    } else {
      matrix_t SmFinal;
      vector_t SvFinal;
      scalar_t sFinal;
      vector_t xFinal;

      {  // lock data
        std::lock_guard<std::mutex> lock(riccatiSolverDataMutex_);

        SmFinal = SmFinalStock_[i];
        SvFinal = SvFinalStock_[i];
        sFinal = sFinalStock_[i];
        xFinal = xFinalStock_[i];
      }

      // modify the end subsystem final values based on the cached values for asynchronous run
      if (i == endingIndicesRiccatiWorker_[taskId] && i < finalActivePartition_) {
        const vector_t deltaState = nominalStateTrajectoriesStock_[i + 1].front() - xFinal;
        sFinal += deltaState.dot(0.5 * SmFinal * deltaState + SvFinal);
        SvFinal += SmFinal * deltaState;
      }

      // solve the backward pass
      riccatiEquationsWorker(taskId, i, SmFinal, SvFinal, sFinal);

      // set the final value for next Riccati equation
      if (i > initActivePartition_) {
        // lock data
        std::lock_guard<std::mutex> lock(riccatiSolverDataMutex_);

        SmFinalStock_[i - 1] = SmTrajectoryStock_[i].front();
        SvFinalStock_[i - 1] = SvTrajectoryStock_[i].front();
        sFinalStock_[i - 1] = sTrajectoryStock_[i].front();
        xFinalStock_[i - 1] = nominalStateTrajectoriesStock_[i].front();
      }
    }
  }
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
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t GaussNewtonDDP::calculateControllerUpdateIS(const std::vector<LinearController>& controllersStock) const {
  // integrates using the trapezoidal approximation method
  scalar_t controllerUpdateIS = 0.0;
  for (const auto& controller : controllersStock) {
    scalar_t currDeltaSquared = 0.0;
    if (!controller.empty()) {
      currDeltaSquared = controller.deltaBiasArray_.front().squaredNorm();
    }
    for (int k = 0; k < controller.size() - 1; k++) {
      scalar_t nextDeltaSquared = controller.deltaBiasArray_[k + 1].squaredNorm();
      controllerUpdateIS += 0.5 * (currDeltaSquared + nextDeltaSquared) * (controller.timeStamp_[k + 1] - controller.timeStamp_[k]);
      currDeltaSquared = nextDeltaSquared;
    }  // end of k loop
  }    // end of controller loop

  return controllerUpdateIS;
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
      const auto indexAlpha = LinearInterpolation::timeSegment(time, &(nominalTimeTrajectoriesStock_[i]));
      vector_t nominalState;
      LinearInterpolation::interpolate(indexAlpha, nominalState, &(nominalStateTrajectoriesStock_[i]));
      vector_t nominalInput;
      LinearInterpolation::interpolate(indexAlpha, nominalInput, &(nominalInputTrajectoriesStock_[i]));
      vector_t deltaUee =
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
    if (nominalTimeTrajectoriesStock_[i].size() > 0) {
      // set cost desired trajectories
      for (auto& lqApproximator : linearQuadraticApproximatorPtrStock_) {
        lqApproximator->costFunction().setCostDesiredTrajectoriesPtr(&this->getCostDesiredTrajectories());
      }

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
          augmentCostWorker(taskId, constraintPenaltyCoefficients_.stateEqualityPenaltyCoeff, 0.0,
                            modelDataTrajectoriesStock_[i][timeIndex]);
        }
      };
      runParallel(task, ddpSettings_.nThreads_);
    }

    /*
     * compute and augment the LQ approximation of the event times for the partition i.
     * also call shiftHessian on the event time's cost 2nd order derivative.
     */
    size_t NE = nominalPostEventIndicesStock_[i].size();
    modelDataEventTimesStock_[i].resize(NE);
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
          linearQuadraticApproximatorPtrStock_[taskId]->approximateUnconstrainedLQProblemAtEventTime(
              nominalTimeTrajectoriesStock_[i][k], nominalStateTrajectoriesStock_[i][k], nominalInputTrajectoriesStock_[i][k],
              modelDataEventTimesStock_[i][timeIndex]);
          // augment cost
          augmentCostWorker(taskId, constraintPenaltyCoefficients_.stateEqualityFinalPenaltyCoeff, 0.0,
                            modelDataEventTimesStock_[i][timeIndex]);
          // shift Hessian
          shiftHessian(modelDataEventTimesStock_[i][timeIndex].cost_.dfdxx);
        }
      };
      runParallel(task, ddpSettings_.nThreads_);
    }

  }  // end of i loop

  /*
   * compute the Heuristics function at the final time.
   * Also call shiftHessian on the Heuristics 2nd order derivative.
   */
  heuristicsFunctionsPtrStock_[0]->setCostDesiredTrajectoriesPtr(&this->getCostDesiredTrajectories());

  heuristics_ = heuristicsFunctionsPtrStock_[0]->finalCostQuadraticApproximation(
      nominalTimeTrajectoriesStock_[finalActivePartition_].back(), nominalStateTrajectoriesStock_[finalActivePartition_].back());

  // shift Hessian
  shiftHessian(heuristics_.dfdxx);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaussNewtonDDP::computeProjectionAndRiccatiModification(ddp_strategy::type strategy, const ModelDataBase& modelData,
                                                             const matrix_t& Sm, ModelDataBase& projectedModelData,
                                                             riccati_modification::Data& riccatiModification) const {
  // compute the Hamiltonian's Hessian
  riccatiModification.time_ = modelData.time_;
  riccatiModification.hamiltonianHessian_ = computeHamiltonianHessian(strategy, modelData, Sm);

  // compute projectors
  computeProjections(riccatiModification.hamiltonianHessian_, modelData.stateInputEqConstr_.dfdu,
                     riccatiModification.constraintRangeProjector_, riccatiModification.constraintNullProjector_);

  // project LQ
  projectLQ(modelData, riccatiModification.constraintRangeProjector_, riccatiModification.constraintNullProjector_, projectedModelData);

  // compute deltaQm, deltaGv, deltaGm
  computeRiccatiModification(strategy, projectedModelData, riccatiModification.deltaQm_, riccatiModification.deltaGv_,
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
    constraintRangeProjector.setZero(inputDim_, 0);
    constraintNullProjector = HmInvUmUmT;

  } else {
    // check numerics
    if (ddpSettings_.checkNumericalStability_) {
      if (LinearAlgebra::rank(Dm) != Dm.rows()) {
        std::string msg = ">>> WARNING: The state-input constraints are rank deficient!";
        Solver_BASE::printString(msg);
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
      std::cerr << "HmProjected:\n" << HmProjected << std::endl;
      throw std::runtime_error("HmProjected should be identity!");
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaussNewtonDDP::computeRiccatiModification(ddp_strategy::type strategy, const ModelDataBase& projectedModelData, matrix_t& deltaQm,
                                                vector_t& deltaGv, matrix_t& deltaGm) const {
  switch (strategy) {
    case ddp_strategy::type::LINE_SEARCH: {
      const auto& QmProjected = projectedModelData.cost_.dfdxx;
      const auto& PmProjected = projectedModelData.cost_.dfdux;

      // Q_minus_PTRinvP
      matrix_t Q_minus_PTRinvP = QmProjected;
      Q_minus_PTRinvP.noalias() -= PmProjected.transpose() * PmProjected;

      // deltaQm
      deltaQm = Q_minus_PTRinvP;
      shiftHessian(deltaQm);
      deltaQm -= Q_minus_PTRinvP;

      // deltaGv, deltaGm
      const auto projectedInputDim = projectedModelData.dynamics_.dfdu.cols();
      deltaGv.setZero(projectedInputDim, 1);
      deltaGm.setZero(projectedInputDim, projectedModelData.stateDim_);

      break;
    }
    case ddp_strategy::type::LEVENBERG_MARQUARDT: {
      const auto& HvProjected = projectedModelData.dynamicsBias_;
      const auto& AmProjected = projectedModelData.dynamics_.dfdx;
      const auto& BmProjected = projectedModelData.dynamics_.dfdu;

      // deltaQm, deltaRm, deltaPm
      deltaQm = 1e-6 * matrix_t::Identity(projectedModelData.stateDim_, projectedModelData.stateDim_);
      deltaGv.noalias() = levenbergMarquardtModule_.riccatiMultiple * BmProjected.transpose() * HvProjected;
      deltaGm.noalias() = levenbergMarquardtModule_.riccatiMultiple * BmProjected.transpose() * AmProjected;

      break;
    }
  }  // end of switch-case
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaussNewtonDDP::projectLQ(const ModelDataBase& modelData, const matrix_t& constraintRangeProjector,
                               const matrix_t& constraintNullProjector, ModelDataBase& projectedModelData) const {
  // dimensions and time
  projectedModelData.time_ = modelData.time_;
  projectedModelData.stateDim_ = modelData.stateDim_;
  projectedModelData.inputDim_ = modelData.inputDim_ - modelData.stateInputEqConstr_.f.rows();

  // dynamics
  projectedModelData.dynamics_.f = modelData.dynamics_.f;
  projectedModelData.dynamicsBias_ = modelData.dynamicsBias_;
  projectedModelData.dynamics_.dfdx = modelData.dynamics_.dfdx;
  projectedModelData.dynamics_.dfdu.noalias() = modelData.dynamics_.dfdu * constraintNullProjector;

  // cost
  projectedModelData.cost_.f = modelData.cost_.f;
  projectedModelData.cost_.dfdx = modelData.cost_.dfdx;
  projectedModelData.cost_.dfdxx = modelData.cost_.dfdxx;
  projectedModelData.cost_.dfduu = constraintNullProjector.transpose() * modelData.cost_.dfduu * constraintNullProjector;

  // constraints
  projectedModelData.ineqConstr_.f.setZero(0);
  projectedModelData.stateEqConstr_.f.setZero(0);

  if (modelData.stateInputEqConstr_.f.rows() == 0) {
    // projected state-input equality constraints
    projectedModelData.stateInputEqConstr_.f.setZero(modelData.inputDim_);
    projectedModelData.stateInputEqConstr_.dfdx.setZero(modelData.inputDim_, modelData.stateDim_);
    projectedModelData.stateInputEqConstr_.dfdu.setZero(modelData.inputDim_, modelData.inputDim_);
    // cost
    projectedModelData.cost_.dfdu.noalias() = constraintNullProjector.transpose() * modelData.cost_.dfdu;
    projectedModelData.cost_.dfdux.noalias() = constraintNullProjector.transpose() * modelData.cost_.dfdux;

  } else {
    /* projected state-input equality constraints */
    projectedModelData.stateInputEqConstr_.f.noalias() = constraintRangeProjector * modelData.stateInputEqConstr_.f;
    projectedModelData.stateInputEqConstr_.dfdx.noalias() = constraintRangeProjector * modelData.stateInputEqConstr_.dfdx;
    projectedModelData.stateInputEqConstr_.dfdu.noalias() = constraintRangeProjector * modelData.stateInputEqConstr_.dfdu;

    // Hv -= BmDmDaggerEv
    projectedModelData.dynamicsBias_.noalias() -= modelData.dynamics_.dfdu * projectedModelData.stateInputEqConstr_.f;

    // Am -= BmDmDaggerCm
    projectedModelData.dynamics_.dfdx.noalias() -= modelData.dynamics_.dfdu * projectedModelData.stateInputEqConstr_.dfdx;

    // common pre-computations
    vector_t RmDmDaggerEv = modelData.cost_.dfduu * projectedModelData.stateInputEqConstr_.f;
    matrix_t RmDmDaggerCm = modelData.cost_.dfduu * projectedModelData.stateInputEqConstr_.dfdx;

    // Rv constrained
    projectedModelData.cost_.dfdu = constraintNullProjector.transpose() * (modelData.cost_.dfdu - RmDmDaggerEv);

    // Pm constrained
    projectedModelData.cost_.dfdux = constraintNullProjector.transpose() * (modelData.cost_.dfdux - RmDmDaggerCm);

    // Qm constrained
    matrix_t PmTransDmDaggerCm = modelData.cost_.dfdux.transpose() * projectedModelData.stateInputEqConstr_.dfdx;
    projectedModelData.cost_.dfdxx -= PmTransDmDaggerCm + PmTransDmDaggerCm.transpose();
    // += DmDaggerCm_Trans_Rm_DmDaggerCm
    projectedModelData.cost_.dfdxx.noalias() += projectedModelData.stateInputEqConstr_.dfdx.transpose() * RmDmDaggerCm;

    // Qv  constrained
    // -= PmTransDmDaggerEv
    projectedModelData.cost_.dfdx.noalias() -= modelData.cost_.dfdux.transpose() * projectedModelData.stateInputEqConstr_.f;
    // -= DmDaggerCmTransRv
    projectedModelData.cost_.dfdx.noalias() -= projectedModelData.stateInputEqConstr_.dfdx.transpose() * modelData.cost_.dfdu;
    // += RmDmDaggerCm_Trans_DmDaggerEv
    projectedModelData.cost_.dfdx.noalias() += RmDmDaggerCm.transpose() * projectedModelData.stateInputEqConstr_.f;

    // q constrained
    // -= Rv_Trans_DmDaggerEv
    projectedModelData.cost_.f -= modelData.cost_.dfdu.dot(projectedModelData.stateInputEqConstr_.f);
    // += 0.5 DmDaggerEv_Trans_RmDmDaggerEv
    projectedModelData.cost_.f += 0.5 * projectedModelData.stateInputEqConstr_.f.dot(RmDmDaggerEv);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaussNewtonDDP::shiftHessian(matrix_t& matrix) const {
  hessian_correction::shiftHessian(ddpSettings_.lineSearch_.hessianCorrectionStrategy_, matrix,
                                   ddpSettings_.lineSearch_.hessianCorrectionMultiple_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaussNewtonDDP::augmentCostWorker(size_t workerIndex, scalar_t stateEqualityPenaltyCoeff, scalar_t stateInputEqualityPenaltyCoeff,
                                       ModelDataBase& modelData) const {
  // state equality constraint (type 2) coefficients
  if (modelData.stateEqConstr_.f.rows() > 0) {
    const vector_t& Hv = modelData.stateEqConstr_.f;
    const matrix_t& Fm = modelData.stateEqConstr_.dfdx;
    modelData.cost_.f += 0.5 * stateEqualityPenaltyCoeff * Hv.dot(Hv);
    modelData.cost_.dfdx.noalias() += stateEqualityPenaltyCoeff * Fm.transpose() * Hv;
    modelData.cost_.dfdu.noalias() += stateEqualityPenaltyCoeff * Fm.transpose() * Fm;
  }

  // state-input equality constraint (type 1) coefficients
  if (modelData.stateInputEqConstr_.f.rows() > 0 && !numerics::almost_eq(stateInputEqualityPenaltyCoeff, 0.0)) {
    const vector_t& Ev = modelData.stateInputEqConstr_.f;
    const matrix_t& Cm = modelData.stateInputEqConstr_.dfdx;
    const matrix_t& Dm = modelData.stateInputEqConstr_.dfdu;
    modelData.cost_.f += 0.5 * stateInputEqualityPenaltyCoeff * Ev.dot(Ev);
    modelData.cost_.dfdx.noalias() += stateInputEqualityPenaltyCoeff * Cm.transpose() * Ev;
    modelData.cost_.dfdu.noalias() += stateInputEqualityPenaltyCoeff * Dm.transpose() * Ev;
    modelData.cost_.dfdxx.noalias() += stateInputEqualityPenaltyCoeff * Cm.transpose() * Cm;
    modelData.cost_.dfduu.noalias() += stateInputEqualityPenaltyCoeff * Dm.transpose() * Dm;
    modelData.cost_.dfdux.noalias() += stateInputEqualityPenaltyCoeff * Dm.transpose() * Cm;
  }

  // inequality constraints
  if (modelData.ineqConstr_.f.rows() > 0) {
    auto penalty = penaltyPtrStock_[workerIndex]->penaltyCostQuadraticApproximation(modelData.ineqConstr_);
    modelData.cost_.f += penalty.f;
    modelData.cost_.dfdx.noalias() += penalty.dfdx;
    modelData.cost_.dfdu.noalias() += penalty.dfdu;
    modelData.cost_.dfdxx.noalias() += penalty.dfdxx;
    modelData.cost_.dfduu.noalias() += penalty.dfduu;
    modelData.cost_.dfdux.noalias() += penalty.dfdux;

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
  constraintPenaltyCoefficients_.stateEqualityPenaltyCoeff = ddpSettings_.constraintPenaltyInitialValue_;
  constraintPenaltyCoefficients_.stateEqualityPenaltyTol = ddpSettings_.constraintTolerance_;

  // final state-only equality
  constraintPenaltyCoefficients_.stateEqualityFinalPenaltyCoeff = ddpSettings_.constraintPenaltyInitialValue_;
  constraintPenaltyCoefficients_.stateEqualityFinalPenaltyTol = ddpSettings_.constraintTolerance_;

  // state-input equality
  constraintPenaltyCoefficients_.stateInputEqualityPenaltyCoeff = ddpSettings_.constraintPenaltyInitialValue_;
  constraintPenaltyCoefficients_.stateInputEqualityPenaltyTol =
      1.0 / std::pow(constraintPenaltyCoefficients_.stateInputEqualityPenaltyCoeff, 0.1);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaussNewtonDDP::updateConstraintPenalties(scalar_t stateEqConstraintISE, scalar_t stateEqFinalConstraintSSE,
                                               scalar_t stateInputEqConstraintISE) {
  // state-only equality penalty
  if (stateEqConstraintISE > ddpSettings_.constraintTolerance_) {
    constraintPenaltyCoefficients_.stateEqualityPenaltyCoeff *= ddpSettings_.constraintPenaltyIncreaseRate_;
    constraintPenaltyCoefficients_.stateEqualityPenaltyTol = ddpSettings_.constraintTolerance_;
  }

  // final state-only equality
  if (stateEqFinalConstraintSSE > ddpSettings_.constraintTolerance_) {
    constraintPenaltyCoefficients_.stateEqualityFinalPenaltyCoeff *= ddpSettings_.constraintPenaltyIncreaseRate_;
    constraintPenaltyCoefficients_.stateEqualityFinalPenaltyTol = ddpSettings_.constraintTolerance_;
  }

  // state-input equality penalty
  if (stateInputEqConstraintISE < constraintPenaltyCoefficients_.stateInputEqualityPenaltyTol) {
    // tighten tolerance
    constraintPenaltyCoefficients_.stateInputEqualityPenaltyTol /=
        std::pow(constraintPenaltyCoefficients_.stateInputEqualityPenaltyCoeff, 0.9);
  } else {
    // tighten tolerance & increase penalty
    constraintPenaltyCoefficients_.stateInputEqualityPenaltyCoeff *= ddpSettings_.constraintPenaltyIncreaseRate_;
    constraintPenaltyCoefficients_.stateInputEqualityPenaltyTol /=
        std::pow(constraintPenaltyCoefficients_.stateInputEqualityPenaltyCoeff, 0.1);
  }
  constraintPenaltyCoefficients_.stateInputEqualityPenaltyTol =
      std::max(constraintPenaltyCoefficients_.stateInputEqualityPenaltyTol, ddpSettings_.constraintTolerance_);

  // display
  if (ddpSettings_.displayInfo_) {
    std::string displayText = "Augmented Lagrangian Penalty Parameters:\n";

    displayText += "    State Equality:      ";
    displayText += "    Penalty Tolerance: " + std::to_string(constraintPenaltyCoefficients_.stateEqualityPenaltyTol);
    displayText += "    Penalty Coefficient: " + std::to_string(constraintPenaltyCoefficients_.stateEqualityPenaltyCoeff) + '\n';

    displayText += "    Final State Equality:";
    displayText += "    Penalty Tolerance: " + std::to_string(constraintPenaltyCoefficients_.stateEqualityFinalPenaltyTol);
    displayText += "    Penalty Coefficient: " + std::to_string(constraintPenaltyCoefficients_.stateEqualityFinalPenaltyCoeff) + '\n';

    displayText += "    State-Input Equality:";
    displayText += "    Penalty Tolerance: " + std::to_string(constraintPenaltyCoefficients_.stateInputEqualityPenaltyTol);
    displayText += "    Penalty Coefficient: " + std::to_string(constraintPenaltyCoefficients_.stateInputEqualityPenaltyCoeff) + ".";
    Solver_BASE::printString(displayText);
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
      const scalar_t finalTime = cachedTimeTrajectoriesStock_[i].back() + OCS2NumericTraits<scalar_t>::weakEpsilon();
      const auto timeSegment = LinearInterpolation::timeSegment(finalTime, &nominalTimeTrajectoriesStock_[i]);

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
          auto indexAlpha = LinearInterpolation::timeSegment(nominalTimeTrajectoriesStock_[i][k], &cachedTimeTrajectoriesStock_[i]);

          vector_t stateCached;
          LinearInterpolation::interpolate(indexAlpha, stateCached, &cachedStateTrajectoriesStock_[i]);
          if (!stateCached.isApprox(nominalStateTrajectoriesStock_[i][k])) {
            throw std::runtime_error("The tail of the cached state trajectory is not correctly set.");
          }

          vector_t inputCached;
          LinearInterpolation::interpolate(indexAlpha, inputCached, &cachedInputTrajectoriesStock_[i]);
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

  // cache the nominal trajectories before the new rollout (time, state, input, ...)
  swapDataToCache();

  // initial controller rollout
  forwardPassTimer_.startTimer();
  const size_t threadId = 0;
  const scalar_t stepLength = 0.0;
  avgTimeStepFP_ =
      performFullRollout(threadId, stepLength, nominalControllersStock_, nominalTimeTrajectoriesStock_, nominalPostEventIndicesStock_,
                         nominalStateTrajectoriesStock_, nominalInputTrajectoriesStock_, modelDataTrajectoriesStock_, performanceIndex_);

  forwardPassTimer_.endTimer();

  // This is necessary for:
  // + The moving horizon (MPC) application
  // + The very first call of the algorithm where there is no previous nominal trajectories.
  correctInitcachedNominalTrajectories();

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
  std::swap(cachedControllerUpdateIS_, nominalControllerUpdateIS_);
  swap(cachedControllersStock_, nominalControllersStock_);
  // update nominal controller
  calculateController();
  nominalControllerUpdateIS_ = calculateControllerUpdateIS(nominalControllersStock_);
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
void GaussNewtonDDP::runIteration() {
  // disable Eigen multi-threading
  Eigen::setNbThreads(1);

  // finding the optimal stepLength
  searchStrategyTimer_.startTimer();
  switch (ddpSettings_.strategy_) {  // clang-format off
    case ddp_strategy::type::LINE_SEARCH: { lineSearch(lineSearchModule_); break; }
    case ddp_strategy::type::LEVENBERG_MARQUARDT: { levenbergMarquardt(levenbergMarquardtModule_); break; }
  }  // clang-format on
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
  std::swap(cachedControllerUpdateIS_, nominalControllerUpdateIS_);
  swap(cachedControllersStock_, nominalControllersStock_);
  // update nominal controller
  calculateController();
  nominalControllerUpdateIS_ = calculateControllerUpdateIS(nominalControllersStock_);
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

  std::vector<LinearController> noInitialController(numPartitions, LinearController(stateDim_, inputDim_));
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
    std::cerr << std::endl;
    std::cerr << "++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
    std::cerr << "+++++++++++++ " + algorithmName_ + " solver is initialized ++++++++++++++" << std::endl;
    std::cerr << "++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
  }

  // infeasible learning rate adjustment scheme
  if (!numerics::almost_ge(ddpSettings_.lineSearch_.maxStepLength_, ddpSettings_.lineSearch_.minStepLength_)) {
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
  isInitInternalControllerEmpty_ = false;
  for (const auto& controller : nominalControllersStock_) {
    isInitInternalControllerEmpty_ = isInitInternalControllerEmpty_ || controller.empty();
  }

  // display
  if (ddpSettings_.displayInfo_) {
    std::cerr << "\n#### Iteration " << iteration_ << " (Dynamics might have been violated)" << std::endl;
  }

  // distribution of the sequential tasks (e.g. Riccati solver) in between threads
  distributeWork();

  // run DDP initializer and update the member variables
  runInit();

  performanceIndexHistory_.emplace_back(performanceIndex_);

  // convergence conditions variables
  bool isOptimizationConverged = false;
  bool isCostFunctionConverged = false;
  bool isStepLengthStarZero = false;
  bool isConstraintsSatisfied = false;
  scalar_t relCost = 2.0 * ddpSettings_.minRelCost_;

  // DDP main loop
  while (iteration_ + 1 < ddpSettings_.maxNumIterations_ && !isOptimizationConverged) {
    // increment iteration counter
    iteration_++;

    // display the iteration's input update norm (before caching the old nominals)
    if (ddpSettings_.displayInfo_) {
      std::cerr << "\n#### Iteration " << iteration_ << std::endl;

      scalar_t maxDeltaUffNorm, maxDeltaUeeNorm;
      calculateControllerUpdateMaxNorm(maxDeltaUffNorm, maxDeltaUeeNorm);
      std::cerr << "max feedforward norm: " << maxDeltaUffNorm << std::endl;
    }

    scalar_t cachedCost = performanceIndex_.totalCost;
    scalar_t cachedStateInputEqConstraintISE = performanceIndex_.stateInputEqConstraintISE;
    scalar_t cachedInequalityConstraintPenalty = performanceIndex_.inequalityConstraintPenalty;

    // cache the nominal trajectories before the new rollout (time, state, input, ...)
    swapDataToCache();

    // run the an iteration of the DDP algorithm and update the member variables
    runIteration();

    performanceIndexHistory_.emplace_back(performanceIndex_);

    // loop break variables
    isCostFunctionConverged = false;
    isStepLengthStarZero = false;
    relCost = std::abs(performanceIndex_.totalCost + performanceIndex_.inequalityConstraintPenalty - cachedCost -
                       cachedInequalityConstraintPenalty);
    switch (ddpSettings_.strategy_) {
      case ddp_strategy::type::LINE_SEARCH: {
        isStepLengthStarZero = numerics::almost_eq(lineSearchModule_.stepLengthStar.load(), 0.0) && !isInitInternalControllerEmpty_;
        isCostFunctionConverged = relCost <= ddpSettings_.minRelCost_;
        break;
      }
      case ddp_strategy::type::LEVENBERG_MARQUARDT: {
        if (levenbergMarquardtModule_.numSuccessiveRejections == 0 && !isInitInternalControllerEmpty_) {
          isCostFunctionConverged = relCost <= ddpSettings_.minRelCost_;
        }
        break;
      }
    }
    isConstraintsSatisfied = performanceIndex_.stateInputEqConstraintISE <= ddpSettings_.constraintTolerance_;
    isOptimizationConverged = (isCostFunctionConverged || isStepLengthStarZero) && isConstraintsSatisfied;
    isInitInternalControllerEmpty_ = false;

  }  // end of while loop

  // display the final iteration's input update norm (before caching the old nominals)
  if (ddpSettings_.displayInfo_) {
    std::cerr << "\n#### Final rollout" << std::endl;

    scalar_t maxDeltaUffNorm, maxDeltaUeeNorm;
    calculateControllerUpdateMaxNorm(maxDeltaUffNorm, maxDeltaUeeNorm);
    std::cerr << "max feedforward norm: " << maxDeltaUffNorm << std::endl;
  }

  // cache the nominal trajectories before the new rollout (time, state, input, ...)
  swapDataToCache();

  // finding the final optimal stepLength and getting the optimal trajectories and controller
  searchStrategyTimer_.startTimer();
  switch (ddpSettings_.strategy_) {  // clang-format off
    case ddp_strategy::type::LINE_SEARCH: { lineSearch(lineSearchModule_); break; }
    case ddp_strategy::type::LEVENBERG_MARQUARDT: { levenbergMarquardt(levenbergMarquardtModule_); break; }
  }  // clang-format on
  searchStrategyTimer_.endTimer();

  // display
  if (ddpSettings_.displayInfo_ || ddpSettings_.displayShortSummary_) {
    std::cerr << "\n++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
    std::cerr << "++++++++++++++ " + algorithmName_ + " solver has terminated +++++++++++++" << std::endl;
    std::cerr << "++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
    std::cerr << "Time Period:                [" << initTime_ << " ," << finalTime_ << "]" << std::endl;
    std::cerr << "Number of Iterations:       " << iteration_ + 1 << " out of " << ddpSettings_.maxNumIterations_ << std::endl;

    printRolloutInfo();

    if (isOptimizationConverged) {
      if (isStepLengthStarZero) {
        std::cerr << algorithmName_ + " successfully terminates as step length reduced to zero." << std::endl;
      }
      if (isCostFunctionConverged) {
        std::cerr << algorithmName_ + " successfully terminates as cost relative change (relCost=" << relCost
                  << ") reached to the minimum value." << std::endl;
      }
      if (isConstraintsSatisfied) {
        std::cerr << "State-input equality constraint absolute ISE (absConstraint1ISE=" << performanceIndex_.stateInputEqConstraintISE
                  << ") reached to its minimum value." << std::endl;
      }
    } else {
      std::cerr << "Maximum number of iterations has reached." << std::endl;
    }
    std::cerr << std::endl;
  }
}

}  // namespace ocs2
