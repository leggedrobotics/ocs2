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
GaussNewtonDDP::GaussNewtonDDP(ddp::Settings ddpSettings, const RolloutBase& rollout, const OptimalControlProblem& optimalControlProblem,
                               const Initializer& initializer)
    : ddpSettings_(std::move(ddpSettings)), threadPool_(std::max(ddpSettings_.nThreads_, size_t(1)) - 1, ddpSettings_.threadPriority_) {
  // Dynamics, Constraints, derivatives, and cost
  dynamicsForwardRolloutPtrStock_.reserve(ddpSettings_.nThreads_);
  initializerRolloutPtrStock_.reserve(ddpSettings_.nThreads_);
  optimalControlProblemStock_.reserve(ddpSettings_.nThreads_);

  // initialize all subsystems, etc.
  for (size_t i = 0; i < ddpSettings_.nThreads_; i++) {
    optimalControlProblemStock_.push_back(optimalControlProblem);

    // initialize rollout
    dynamicsForwardRolloutPtrStock_.emplace_back(rollout.clone());

    // initialize initializerRollout
    initializerRolloutPtrStock_.emplace_back(new InitializerRollout(initializer, rollout.settings()));
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
  auto meritFunc = [this](const PerformanceIndex& p) { return calculateRolloutMerit(p); };
  switch (ddpSettings_.strategy_) {
    case search_strategy::Type::LINE_SEARCH: {
      std::vector<std::reference_wrapper<RolloutBase>> rolloutRefStock;
      std::vector<std::reference_wrapper<OptimalControlProblem>> problemRefStock;
      for (size_t i = 0; i < ddpSettings_.nThreads_; i++) {
        rolloutRefStock.emplace_back(*dynamicsForwardRolloutPtrStock_[i]);
        problemRefStock.emplace_back(optimalControlProblemStock_[i]);
      }  // end of i loop
      searchStrategyPtr_.reset(new LineSearchStrategy(basicStrategySettings, ddpSettings_.lineSearch_, threadPool_,
                                                      std::move(rolloutRefStock), std::move(problemRefStock), *penaltyPtr_, meritFunc));
      break;
    }
    case search_strategy::Type::LEVENBERG_MARQUARDT: {
      constexpr size_t threadID = 0;
      searchStrategyPtr_.reset(new LevenbergMarquardtStrategy(basicStrategySettings, ddpSettings_.levenbergMarquardt_,
                                                              *dynamicsForwardRolloutPtrStock_[threadID],
                                                              optimalControlProblemStock_[threadID], *penaltyPtr_, meritFunc));
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

  // very important, these are variables that are carried in between iterations
  nominalPrimalData_.clear();
  dualData_.clear();

  cachedPrimalData_.clear();
  cachedDualData_.clear();

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
  return scalar_array_t{initTime_, finalTime_};
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
  const int N = optimizedPrimalData_.primalSolution.timeTrajectory_.size();

  auto getRequestedDataLength = [](const scalar_array_t& timeTrajectory, scalar_t time) {
    int index = std::distance(timeTrajectory.cbegin(), std::upper_bound(timeTrajectory.cbegin(), timeTrajectory.cend(), time));
    // PR#519 fix solution time window to include 1 point beyond requested time
    index += index != timeTrajectory.size() ? 1 : 0;
    return index;
  };
  const int length = getRequestedDataLength(optimizedPrimalData_.primalSolution.timeTrajectory_, finalTime);
  // fill trajectories
  primalSolutionPtr->timeTrajectory_.clear();
  primalSolutionPtr->timeTrajectory_.reserve(N);
  primalSolutionPtr->stateTrajectory_.clear();
  primalSolutionPtr->stateTrajectory_.reserve(N);
  primalSolutionPtr->inputTrajectory_.clear();
  primalSolutionPtr->inputTrajectory_.reserve(N);

  primalSolutionPtr->timeTrajectory_.insert(primalSolutionPtr->timeTrajectory_.end(),
                                            optimizedPrimalData_.primalSolution.timeTrajectory_.begin(),
                                            optimizedPrimalData_.primalSolution.timeTrajectory_.begin() + length);
  primalSolutionPtr->stateTrajectory_.insert(primalSolutionPtr->stateTrajectory_.end(),
                                             optimizedPrimalData_.primalSolution.stateTrajectory_.begin(),
                                             optimizedPrimalData_.primalSolution.stateTrajectory_.begin() + length);
  primalSolutionPtr->inputTrajectory_.insert(primalSolutionPtr->inputTrajectory_.end(),
                                             optimizedPrimalData_.primalSolution.inputTrajectory_.begin(),
                                             optimizedPrimalData_.primalSolution.inputTrajectory_.begin() + length);

  // fill controller
  if (ddpSettings_.useFeedbackPolicy_) {
    primalSolutionPtr->controllerPtr_.reset(new LinearController);
    // length of the copy
    const int length = getRequestedDataLength(getLinearController(optimizedPrimalData_).timeStamp_, finalTime);
    primalSolutionPtr->controllerPtr_->concatenate(optimizedPrimalData_.primalSolution.controllerPtr_.get(), 0, length);

  } else {
    primalSolutionPtr->controllerPtr_.reset(
        new FeedforwardController(primalSolutionPtr->timeTrajectory_, primalSolutionPtr->inputTrajectory_));
  }

  // fill mode schedule
  primalSolutionPtr->modeSchedule_ = optimizedPrimalData_.primalSolution.modeSchedule_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation GaussNewtonDDP::getValueFunctionImpl(
    const scalar_t& time, const vector_t& state, const PrimalDataContainer& primalData,
    const std::vector<ScalarFunctionQuadraticApproximation>& valueFunctionTrajectory) const {
  // result
  ScalarFunctionQuadraticApproximation valueFunction;
  const auto indexAlpha = LinearInterpolation::timeSegment(time, primalData.primalSolution.timeTrajectory_);
  valueFunction.f = LinearInterpolation::interpolate(
      indexAlpha, valueFunctionTrajectory,
      +[](const std::vector<ocs2::ScalarFunctionQuadraticApproximation>& vec, size_t ind) -> const scalar_t& { return vec[ind].f; });
  valueFunction.dfdx = LinearInterpolation::interpolate(
      indexAlpha, valueFunctionTrajectory,
      +[](const std::vector<ocs2::ScalarFunctionQuadraticApproximation>& vec, size_t ind) -> const vector_t& { return vec[ind].dfdx; });
  valueFunction.dfdxx = LinearInterpolation::interpolate(
      indexAlpha, valueFunctionTrajectory,
      +[](const std::vector<ocs2::ScalarFunctionQuadraticApproximation>& vec, size_t ind) -> const matrix_t& { return vec[ind].dfdxx; });

  // Re-center around query state
  const vector_t xNominal = LinearInterpolation::interpolate(indexAlpha, primalData.primalSolution.stateTrajectory_);
  const vector_t deltaX = state - xNominal;
  const vector_t SmDeltaX = valueFunction.dfdxx * deltaX;
  valueFunction.f += deltaX.dot(0.5 * SmDeltaX + valueFunction.dfdx);
  valueFunction.dfdx += SmDeltaX;  // Adapt dfdx after f!

  return valueFunction;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation GaussNewtonDDP::getValueFunctionFromCache(scalar_t time, const vector_t& state) const {
  return getValueFunctionImpl(time, state, cachedPrimalData_, cachedDualData_.valueFunctionTrajectory);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation GaussNewtonDDP::getValueFunction(scalar_t time, const vector_t& state) const {
  return getValueFunctionImpl(time, state, optimizedPrimalData_, dualData_.valueFunctionTrajectory);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation GaussNewtonDDP::getHamiltonian(scalar_t time, const vector_t& state, const vector_t& input) const {
  ModelData modelData;

  // perform the LQ approximation of the OC problem
  // note that the cost already includes:
  // - state-input intermediate cost
  // - state-input soft constraint cost
  // - state-only intermediate cost
  // - state-only soft constraint cost
  LinearQuadraticApproximator lqapprox(optimalControlProblemStock_[0], settings().checkNumericalStability_);
  lqapprox.approximateLQProblem(time, state, input, modelData);
  modelData.time_ = time;
  modelData.stateDim_ = state.rows();
  modelData.inputDim_ = input.rows();
  modelData.dynamicsBias_.setZero(state.rows());
  modelData.checkSizes(state.rows(), input.rows());

  // augment the cost with state-only equality and state-input inequality constraint terms
  augmentCostWorker(0, constraintPenaltyCoefficients_.stateEqConstrPenaltyCoeff, 0.0, modelData);

  // initialize the Hamiltonian with the augmented cost
  ScalarFunctionQuadraticApproximation hamiltonian(modelData.cost_);

  // add the state-input equality constraint cost nu(x) * g(x,u) to the Hamiltonian
  // note that nu has no approximation and is used as a constant
  const vector_t nu = getStateInputEqualityConstraintLagrangian(time, state);
  hamiltonian.f += nu.dot(modelData.stateInputEqConstr_.f);
  hamiltonian.dfdx.noalias() += modelData.stateInputEqConstr_.dfdx.transpose() * nu;
  hamiltonian.dfdu.noalias() += modelData.stateInputEqConstr_.dfdu.transpose() * nu;
  // dfdxx is zero for the state-input equality constraint cost
  // dfdux is zero for the state-input equality constraint cost
  // dfduu is zero for the state-input equality constraint cost

  // add the "future cost" dVdx(x) * f(x,u) to the Hamiltonian
  const ScalarFunctionQuadraticApproximation V = getValueFunction(time, state);
  const matrix_t dVdxx_dfdx = V.dfdxx.transpose() * modelData.dynamics_.dfdx;
  hamiltonian.f += V.dfdx.dot(modelData.dynamics_.f);
  hamiltonian.dfdx.noalias() += V.dfdxx.transpose() * modelData.dynamics_.f + modelData.dynamics_.dfdx.transpose() * V.dfdx;
  hamiltonian.dfdu.noalias() += modelData.dynamics_.dfdu.transpose() * V.dfdx;
  hamiltonian.dfdxx.noalias() += dVdxx_dfdx + dVdxx_dfdx.transpose();
  hamiltonian.dfdux.noalias() += modelData.dynamics_.dfdu.transpose() * V.dfdxx;
  // dfduu is zero for the "future cost"

  return hamiltonian;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t GaussNewtonDDP::getStateInputEqualityConstraintLagrangianImpl(scalar_t time, const vector_t& state,
                                                                       const PrimalDataContainer& primalData,
                                                                       const DualDataContainer& dualData) const {
  const auto indexAlpha = LinearInterpolation::timeSegment(time, primalData.primalSolution.timeTrajectory_);
  const vector_t xNominal = LinearInterpolation::interpolate(indexAlpha, primalData.primalSolution.stateTrajectory_);

  const matrix_t Bm = LinearInterpolation::interpolate(indexAlpha, primalData.modelDataTrajectory, model_data::dynamics_dfdu);
  const matrix_t Pm = LinearInterpolation::interpolate(indexAlpha, primalData.modelDataTrajectory, model_data::cost_dfdux);
  const vector_t Rv = LinearInterpolation::interpolate(indexAlpha, primalData.modelDataTrajectory, model_data::cost_dfdu);

  const vector_t EvProjected =
      LinearInterpolation::interpolate(indexAlpha, dualData.projectedModelDataTrajectory, model_data::stateInputEqConstr_f);
  const matrix_t CmProjected =
      LinearInterpolation::interpolate(indexAlpha, dualData.projectedModelDataTrajectory, model_data::stateInputEqConstr_dfdx);
  const matrix_t Hm =
      LinearInterpolation::interpolate(indexAlpha, dualData.riccatiModificationTrajectory, riccati_modification::hamiltonianHessian);
  const matrix_t DmDagger =
      LinearInterpolation::interpolate(indexAlpha, dualData.riccatiModificationTrajectory, riccati_modification::constraintRangeProjector);

  const vector_t deltaX = state - xNominal;
  const vector_t costate = getValueFunction(time, state).dfdx;

  vector_t err = EvProjected;
  err.noalias() += CmProjected * deltaX;

  vector_t temp = -Rv;
  temp.noalias() -= Pm * deltaX;
  temp.noalias() -= Bm.transpose() * costate;
  temp.noalias() += Hm * err;

  return DmDagger.transpose() * temp;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t GaussNewtonDDP::getStateInputEqualityConstraintLagrangian(scalar_t time, const vector_t& state) const {
  return getStateInputEqualityConstraintLagrangianImpl(time, state, nominalPrimalData_, dualData_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaussNewtonDDP::retrieveActiveNormalizedTime(const std::pair<int, int>& partitionInterval, const scalar_array_t& timeTrajectory,
                                                  const size_array_t& postEventIndices, scalar_array_t& normalizedTimeTrajectory,
                                                  size_array_t& normalizedPostEventIndices) {
  // Although the rightmost point is excluded from the current interval, i.e. it won't be written into the dual solution array, we still
  // the following two (+1) are essential to start the backward pass
  auto firstTimeItr = timeTrajectory.begin() + partitionInterval.first;
  auto lastTimeItr = timeTrajectory.begin() + partitionInterval.second + 1;
  const int N = partitionInterval.second - partitionInterval.first + 1;
  // normalized time
  normalizedTimeTrajectory.resize(N);
  std::transform(firstTimeItr, lastTimeItr, normalizedTimeTrajectory.rbegin(), [](scalar_t t) -> scalar_t { return -t; });

  auto firstEventItr = std::upper_bound(postEventIndices.begin(), postEventIndices.end(), partitionInterval.first);
  auto lastEventItr = std::upper_bound(postEventIndices.begin(), postEventIndices.end(), partitionInterval.second);
  const int NE = std::distance(firstEventItr, lastEventItr);
  // normalized event past the index
  normalizedPostEventIndices.resize(NE);
  std::transform(firstEventItr, lastEventItr, normalizedPostEventIndices.rbegin(),
                 [N, &partitionInterval](size_t i) -> size_t { return N - i + partitionInterval.first; });
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaussNewtonDDP::adjustController(const scalar_array_t& newEventTimes, const scalar_array_t& controllerEventTimes) {
  // TODO:
  // adjust the nominal controllerStock using trajectory spreading
  // if (!nominalControllersStock_.empty()) {
  //   trajectorySpreadingController_.adjustController(newEventTimes, controllerEventTimes, nominalControllersStock_);
  // }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::vector<std::pair<int, int>> GaussNewtonDDP::getPartitionIntervalsFromTimeTrajectory(const scalar_array_t& timeTrajectory,
                                                                                         int numWorkers) {
  scalar_array_t desiredPartitionPoints;
  desiredPartitionPoints.resize(numWorkers + 1);
  desiredPartitionPoints.front() = timeTrajectory.front();

  const scalar_t increment = (timeTrajectory.back() - timeTrajectory.front()) / static_cast<scalar_t>(numWorkers);
  for (size_t i = 1u; i < desiredPartitionPoints.size() - 1; i++) {
    desiredPartitionPoints[i] = desiredPartitionPoints[i - 1] + increment;
  }
  desiredPartitionPoints.back() = timeTrajectory.back();

  std::vector<std::pair<int, int>> partitionIntervals;
  int startPos = 0;
  int endPos;
  for (size_t i = 1u; i < desiredPartitionPoints.size(); i++) {
    const scalar_t& time = desiredPartitionPoints[i];
    endPos = std::distance(timeTrajectory.begin(), std::lower_bound(timeTrajectory.begin(), timeTrajectory.end(), time));
    if (endPos != startPos) {
      partitionIntervals.emplace_back(startPos, endPos);
      startPos = endPos;
    }
  }

  return partitionIntervals;
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
scalar_t GaussNewtonDDP::rolloutInitialTrajectory(PrimalDataContainer& primalData, ControllerBase* controller, size_t workerIndex /*= 0*/) {
  // get event times
  const scalar_array_t& eventTimes = this->getReferenceManager().getModeSchedule().eventTimes;
  primalData.primalSolution.modeSchedule_ = this->getReferenceManager().getModeSchedule();

  // prepare outputs
  // cannot use PrimalDataContainer::clear() as the controller need to be preserved.
  auto& timeTrajectory = primalData.primalSolution.timeTrajectory_;
  auto& stateTrajectory = primalData.primalSolution.stateTrajectory_;
  auto& inputTrajectory = primalData.primalSolution.inputTrajectory_;
  auto& postEventIndices = primalData.postEventIndices;
  auto& modelDataTrajectory = primalData.modelDataTrajectory;
  auto& modelDataEventTimes = primalData.modelDataEventTimes;

  timeTrajectory.clear();
  postEventIndices.clear();
  stateTrajectory.clear();
  inputTrajectory.clear();
  modelDataTrajectory.clear();
  modelDataEventTimes.clear();

  // Find until where we have a controller available for the rollout
  scalar_t controllerAvailableTill = controller->empty() ? initTime_ : static_cast<LinearController*>(controller)->timeStamp_.back();

  if (ddpSettings_.debugPrintRollout_) {
    std::cerr << "[GaussNewtonDDP::rolloutInitialTrajectory] for t = [" << initTime_ << ", " << finalTime_ << "]\n"
              << "\tcontroller available till t = " << controllerAvailableTill << "\n";
  }

  vector_t xCurrent = initState_;
  // Start and end of rollout segment
  const scalar_t t0 = initTime_;
  const scalar_t tf = finalTime_;

  // Divide the rollout segment in controller rollout and operating points
  const std::pair<scalar_t, scalar_t> controllerRolloutFromTo{t0, std::max(t0, std::min(controllerAvailableTill, tf))};
  const std::pair<scalar_t, scalar_t> operatingPointsFromTo{controllerRolloutFromTo.second, tf};

  if (ddpSettings_.debugPrintRollout_) {
    std::cerr << "[GaussNewtonDDP::rolloutInitialTrajectory]"
              << " for t = [" << t0 << ", " << tf << "]\n";
    if (controllerRolloutFromTo.first < controllerRolloutFromTo.second) {
      std::cerr << "\twill use controller for t = [" << controllerRolloutFromTo.first << ", " << controllerRolloutFromTo.second << "]\n";
    }
    if (operatingPointsFromTo.first < operatingPointsFromTo.second) {
      std::cerr << "\twill use operating points for t = [" << operatingPointsFromTo.first << ", " << operatingPointsFromTo.second << "]\n";
    }
  }

  // Rollout with controller
  if (controllerRolloutFromTo.first < controllerRolloutFromTo.second) {
    xCurrent = dynamicsForwardRolloutPtrStock_[workerIndex]->run(controllerRolloutFromTo.first, xCurrent, controllerRolloutFromTo.second,
                                                                 controller, eventTimes, timeTrajectory, postEventIndices, stateTrajectory,
                                                                 inputTrajectory);
  }

  // Finish rollout with operating points
  if (operatingPointsFromTo.first < operatingPointsFromTo.second) {
    // Remove last point of the controller rollout if it is directly past an event. Here where we want to use the initializer
    // instead. However, we do start the integration at the state after the event. i.e. the jump map remains applied.
    if (!postEventIndices.empty() && postEventIndices.back() == (timeTrajectory.size() - 1)) {
      // Start new integration at the time point after the event
      timeTrajectory.pop_back();
      stateTrajectory.pop_back();
      inputTrajectory.pop_back();
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
      eventIndex += stateTrajectory.size();  // This size of this trajectory part was missing when counting events in the tail
    }

    // Concatenate the operating points to the rollout
    timeTrajectory.insert(timeTrajectory.end(), timeTrajectoryTail.begin(), timeTrajectoryTail.end());
    postEventIndices.insert(postEventIndices.end(), eventsPastTheEndIndecesTail.begin(), eventsPastTheEndIndecesTail.end());
    stateTrajectory.insert(stateTrajectory.end(), stateTrajectoryTail.begin(), stateTrajectoryTail.end());
    inputTrajectory.insert(inputTrajectory.end(), inputTrajectoryTail.begin(), inputTrajectoryTail.end());
  }

  // update model data trajectory
  modelDataTrajectory.resize(timeTrajectory.size());
  for (size_t k = 0; k < timeTrajectory.size(); k++) {
    modelDataTrajectory[k].time_ = timeTrajectory[k];
    modelDataTrajectory[k].stateDim_ = stateTrajectory[k].size();
    modelDataTrajectory[k].inputDim_ = inputTrajectory[k].size();
    modelDataTrajectory[k].dynamicsBias_.setZero(stateTrajectory[k].size());
  }

  // update model data at event times
  modelDataEventTimes.resize(postEventIndices.size());
  for (size_t ke = 0; ke < postEventIndices.size(); ke++) {
    const auto index = postEventIndices[ke] - 1;
    modelDataEventTimes[ke].time_ = timeTrajectory[index];
    modelDataEventTimes[ke].stateDim_ = stateTrajectory[index].size();
    modelDataEventTimes[ke].inputDim_ = inputTrajectory[index].size();
    modelDataEventTimes[ke].dynamicsBias_.setZero(stateTrajectory[index].size());
  }

  if (!xCurrent.allFinite()) {
    throw std::runtime_error("System became unstable during the rollout.");
  }

  // debug print
  if (ddpSettings_.debugPrintRollout_) {
    std::cerr << "\n++++++++++++++++++++++++++++++\n";
    std::cerr << "[GaussNewtonDDP::rolloutInitialTrajectory] for t = [" << timeTrajectory.front() << ", " << timeTrajectory.back() << "]";
    std::cerr << "\n++++++++++++++++++++++++++++++\n";
    RolloutBase::display(timeTrajectory, postEventIndices, stateTrajectory, &inputTrajectory);
  }
  // average time step
  return (controllerAvailableTill - initTime_) / static_cast<double>(timeTrajectory.size());
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
scalar_t GaussNewtonDDP::solveSequentialRiccatiEquationsImpl(const ScalarFunctionQuadraticApproximation& finalValueFunction) {
  // pre-allocate memory for dual solution
  const size_t outputN = nominalPrimalData_.primalSolution.timeTrajectory_.size();
  dualData_.valueFunctionTrajectory.clear();
  dualData_.valueFunctionTrajectory.resize(outputN);

  // solve it sequentially for the first iteration
  if (totalNumIterations_ == 0) {
    const std::pair<int, int> partitionInterval{0, outputN - 1};
    riccatiEquationsWorker(0, partitionInterval, finalValueFunction);
  } else {  // solve it in parallel
    // do equal-time partitions based on available thread resource
    std::vector<std::pair<int, int>> partitionIntervals =
        getPartitionIntervalsFromTimeTrajectory(nominalPrimalData_.primalSolution.timeTrajectory_, ddpSettings_.nThreads_);

    // hold the final value function of each partition
    std::vector<ScalarFunctionQuadraticApproximation> finalValueFunctionOfEachPartition(partitionIntervals.size());

    finalValueFunctionOfEachPartition.back() = finalValueFunction;

    for (size_t i = 0; i < partitionIntervals.size() - 1; i++) {
      const int startIndexOfNextPartition = partitionIntervals[i + 1].first;
      const vector_t& xFinalUpdated = nominalPrimalData_.primalSolution.stateTrajectory_[startIndexOfNextPartition];
      finalValueFunctionOfEachPartition[i] =
          getValueFunctionFromCache(nominalPrimalData_.primalSolution.timeTrajectory_[startIndexOfNextPartition], xFinalUpdated);
    }  // end of loop

    nextTaskId_ = 0;
    std::function<void(void)> task = [&] {
      const size_t taskId = nextTaskId_++;  // assign task ID (atomic)
      const auto& partitionInterval = partitionIntervals[taskId];
      riccatiEquationsWorker(taskId, partitionInterval, finalValueFunctionOfEachPartition[taskId]);
    };
    runParallel(task, partitionIntervals.size());
  }

  // the last index of the partition is excluded, so the final value function is filled manually
  dualData_.valueFunctionTrajectory.back() = finalValueFunction;

  // testing the numerical stability of the Riccati equations
  if (ddpSettings_.checkNumericalStability_) {
    int N = nominalPrimalData_.primalSolution.timeTrajectory_.size();
    for (int k = N - 1; k >= 0; k--) {
      try {
        const ScalarFunctionQuadraticApproximation& valueFunction = dualData_.valueFunctionTrajectory[k];
        if (!valueFunction.dfdxx.allFinite()) {
          throw std::runtime_error("Sm is unstable.");
        }
        if (LinearAlgebra::eigenvalues(valueFunction.dfdxx).real().minCoeff() < -Eigen::NumTraits<scalar_t>::epsilon()) {
          throw std::runtime_error("Sm matrix is not positive semi-definite. It's smallest eigenvalue is " +
                                   std::to_string(LinearAlgebra::eigenvalues(valueFunction.dfdxx).real().minCoeff()) + ".");
        }
        if (!valueFunction.dfdx.allFinite()) {
          throw std::runtime_error("Sv is unstable.");
        }
        if (std::isnan(valueFunction.f)) {
          throw std::runtime_error("s is unstable");
        }
      } catch (const std::exception& error) {
        std::cerr << "what(): " << error.what() << " at time " << nominalPrimalData_.primalSolution.timeTrajectory_[k] << " [sec].\n";
        for (int kp = k; kp < k + 10; kp++) {
          if (kp >= N) {
            continue;
          }
          std::cerr << "Sm[" << nominalPrimalData_.primalSolution.timeTrajectory_[kp] << "]:\n"
                    << dualData_.valueFunctionTrajectory[kp].dfdxx.norm() << "\n";
          std::cerr << "Sv[" << nominalPrimalData_.primalSolution.timeTrajectory_[kp] << "]:\t"
                    << dualData_.valueFunctionTrajectory[kp].dfdx.transpose().norm() << "\n";
          std::cerr << "s[" << nominalPrimalData_.primalSolution.timeTrajectory_[kp] << "]:\t" << dualData_.valueFunctionTrajectory[kp].f
                    << "\n";
        }
        throw;
      }
    }  // end of k loop
  }

  // average time step
  return (finalTime_ - initTime_) / outputN;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaussNewtonDDP::calculateController() {
  const size_t N = nominalPrimalData_.primalSolution.timeTrajectory_.size();

  if (unoptimizedControllerPtr_ == nullptr) {
    unoptimizedControllerPtr_.reset(new LinearController);
  }
  LinearController* linearController = static_cast<LinearController*>(unoptimizedControllerPtr_.get());

  linearController->clear();
  linearController->timeStamp_ = nominalPrimalData_.primalSolution.timeTrajectory_;
  linearController->gainArray_.resize(N);
  linearController->biasArray_.resize(N);
  linearController->deltaBiasArray_.resize(N);

  // perform the calculateControllerWorker for partition i
  nextTimeIndex_ = 0;
  std::function<void(void)> task = [this, N, linearController] {
    int timeIndex;
    // get next time index (atomic)
    while ((timeIndex = nextTimeIndex_++) < N) {
      calculateControllerWorker(timeIndex, nominalPrimalData_, dualData_, *linearController);
    }
  };
  runParallel(task, ddpSettings_.nThreads_);

  // if the final time is not an event time change the last control to the second to the last
  if (nominalPrimalData_.postEventIndices.empty() ||
      nominalPrimalData_.postEventIndices.back() != nominalPrimalData_.primalSolution.timeTrajectory_.size() - 1) {
    LinearController& ctrl = *linearController;
    if (ctrl.size() > 1) {
      const size_t secondToLastIndex = ctrl.size() - 2u;
      ctrl.gainArray_.back() = ctrl.gainArray_[secondToLastIndex];
      ctrl.biasArray_.back() = ctrl.biasArray_[secondToLastIndex];
      ctrl.deltaBiasArray_.back() = ctrl.deltaBiasArray_[secondToLastIndex];
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaussNewtonDDP::calculateControllerUpdateMaxNorm(scalar_t& maxDeltaUffNorm, scalar_t& maxDeltaUeeNorm) const {
  maxDeltaUffNorm = 0.0;
  maxDeltaUeeNorm = 0.0;

  const LinearController* linearController = static_cast<const LinearController*>(unoptimizedControllerPtr_.get());

  for (size_t k = 0; k < linearController->timeStamp_.size(); k++) {
    maxDeltaUffNorm = std::max(maxDeltaUffNorm, linearController->deltaBiasArray_[k].norm());

    const auto time = linearController->timeStamp_[k];
    const auto indexAlpha = LinearInterpolation::timeSegment(time, nominalPrimalData_.primalSolution.timeTrajectory_);
    const vector_t nominalState = LinearInterpolation::interpolate(indexAlpha, nominalPrimalData_.primalSolution.stateTrajectory_);
    const vector_t nominalInput = LinearInterpolation::interpolate(indexAlpha, nominalPrimalData_.primalSolution.inputTrajectory_);
    const vector_t deltaUee = nominalInput - linearController->gainArray_[k] * nominalState - linearController->biasArray_[k];
    maxDeltaUeeNorm = std::max(maxDeltaUeeNorm, deltaUee.norm());
  }  // end of k loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaussNewtonDDP::approximateOptimalControlProblem() {
  /*
   * compute and augment the LQ approximation of intermediate times for the partition i
   */
  // perform the LQ approximation for intermediate times at partition i
  approximateIntermediateLQ(nominalPrimalData_.primalSolution.timeTrajectory_, nominalPrimalData_.postEventIndices,
                            nominalPrimalData_.primalSolution.stateTrajectory_, nominalPrimalData_.primalSolution.inputTrajectory_,
                            nominalPrimalData_.modelDataTrajectory);

  // augment the intermediate cost by performing augmentCostWorker for the partition i
  nextTimeIndex_ = 0;
  nextTaskId_ = 0;

  const size_t N = nominalPrimalData_.primalSolution.timeTrajectory_.size();
  std::function<void(void)> task = [this, N] {
    size_t timeIndex;
    size_t taskId = nextTaskId_++;  // assign task ID (atomic)

    // get next time index is atomic
    while ((timeIndex = nextTimeIndex_++) < N) {
      // augment cost
      augmentCostWorker(taskId, constraintPenaltyCoefficients_.stateEqConstrPenaltyCoeff, 0.0,
                        nominalPrimalData_.modelDataTrajectory[timeIndex]);
    }
  };
  runParallel(task, ddpSettings_.nThreads_);

  /*
   * compute and augment the LQ approximation of the event times for the partition.
   * also call shiftHessian on the event time's cost 2nd order derivative.
   */
  const size_t NE = nominalPrimalData_.postEventIndices.size();
  if (NE > 0) {
    nextTimeIndex_ = 0;
    nextTaskId_ = 0;
    std::function<void(void)> task = [this, NE] {
      int timeIndex;
      const size_t taskId = nextTaskId_++;  // assign task ID (atomic)

      LinearQuadraticApproximator lqapprox(optimalControlProblemStock_[taskId], ddpSettings_.checkNumericalStability_);

      // nextTimeIndex_ is atomic
      while ((timeIndex = nextTimeIndex_++) < NE) {
        ModelData& modelData = nominalPrimalData_.modelDataEventTimes[timeIndex];

        // execute approximateLQ for the pre-event node
        const size_t preEventIndex = nominalPrimalData_.postEventIndices[timeIndex] - 1;
        lqapprox.approximateLQProblemAtEventTime(nominalPrimalData_.primalSolution.timeTrajectory_[preEventIndex],
                                                 nominalPrimalData_.primalSolution.stateTrajectory_[preEventIndex], modelData);
        // augment cost
        augmentCostWorker(taskId, constraintPenaltyCoefficients_.stateFinalEqConstrPenaltyCoeff, 0.0, modelData);
        // shift Hessian
        if (ddpSettings_.strategy_ == search_strategy::Type::LINE_SEARCH) {
          hessian_correction::shiftHessian(ddpSettings_.lineSearch_.hessianCorrectionStrategy_, modelData.cost_.dfdxx,
                                           ddpSettings_.lineSearch_.hessianCorrectionMultiple_);
        }
      }
    };
    runParallel(task, ddpSettings_.nThreads_);
  }

  /*
   * compute the Heuristics function at the final time. Also call shiftHessian on the Heuristics 2nd order derivative.
   */
  ModelData heuristicsModelData;
  LinearQuadraticApproximator lqapprox(optimalControlProblemStock_[0], ddpSettings_.checkNumericalStability_);
  lqapprox.approximateLQProblemAtFinalTime(nominalPrimalData_.primalSolution.timeTrajectory_.back(),
                                           nominalPrimalData_.primalSolution.stateTrajectory_.back(), heuristicsModelData);
  heuristics_ = std::move(heuristicsModelData.cost_);

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
    modelData.cost_ += penaltyPtr_->getQuadraticApproximation(modelData.time_, modelData.ineqConstr_);

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
void GaussNewtonDDP::runSearchStrategy(scalar_t lqModelExpectedCost, PrimalDataContainer& dstPrimalData) {
  dstPrimalData.clear();
  auto performanceIndex = performanceIndex_;

  const auto& modeSchedule = this->getReferenceManager().getModeSchedule();
  dstPrimalData.primalSolution.modeSchedule_ = this->getReferenceManager().getModeSchedule();

  LinearController* linearController = static_cast<LinearController*>(unoptimizedControllerPtr_.get());

  bool success = searchStrategyPtr_->run(lqModelExpectedCost, initTime_, initState_, finalTime_, modeSchedule, *linearController,
                                         performanceIndex, dstPrimalData.primalSolution.timeTrajectory_, dstPrimalData.postEventIndices,
                                         dstPrimalData.primalSolution.stateTrajectory_, dstPrimalData.primalSolution.inputTrajectory_,
                                         dstPrimalData.modelDataTrajectory, dstPrimalData.modelDataEventTimes, avgTimeStepFP_);

  // accept or reject the search
  if (success) {
    // update nominal trajectories
    performanceIndex_ = performanceIndex;
    // unoptimizedController_ is now optimized. Regarding the controller, search is an in-place operation
    linearController->deltaBiasArray_.clear();
    dstPrimalData.primalSolution.controllerPtr_.swap(unoptimizedControllerPtr_);
  } else {
    // If fail, copy the entire cache back. To keep the consistence of cached data, all cache should be left untouched.
    dstPrimalData = cachedPrimalData_;
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
  nominalPrimalData_.swap(cachedPrimalData_);
  dualData_.swap(cachedDualData_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaussNewtonDDP::correctInitcachedNominalTrajectories() {
  // TODO: Cache Rectification Take care of the case where the end system of the cached trajectories is different from the system in the
  // partition points. Usually, the cached value used to start each partition are aligned with the partition points in time, so there won't
  // be problems. But sometimes, in MPC setup, horizon shifts a lot between consecutive solve that the end value of the cache is needed to
  // start the second to last partition. Here, the time is not aligned, and we have to rectify cache to match state, input dimensions.
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
    const auto avgTimeStep = rolloutInitialTrajectory(nominalPrimalData_, optimizedPrimalData_.primalSolution.controllerPtr_.get(), taskId);
    scalar_t heuristicsValue = 0.0;

    searchStrategyPtr_->rolloutCostAndConstraints(
        optimalControlProblemStock_[taskId], nominalPrimalData_.primalSolution.timeTrajectory_, nominalPrimalData_.postEventIndices,
        nominalPrimalData_.primalSolution.stateTrajectory_, nominalPrimalData_.primalSolution.inputTrajectory_,
        nominalPrimalData_.modelDataTrajectory, nominalPrimalData_.modelDataEventTimes, heuristicsValue);

    // This is necessary for:
    // + The moving horizon (MPC) application
    // + The very first call of the algorithm where there is no previous nominal trajectories.
    correctInitcachedNominalTrajectories();

    performanceIndex_ = searchStrategyPtr_->calculateRolloutPerformanceIndex(
        *penaltyPtr_, nominalPrimalData_.primalSolution.timeTrajectory_, nominalPrimalData_.modelDataTrajectory,
        nominalPrimalData_.modelDataEventTimes, heuristicsValue);

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
  avgTimeStepBP_ = solveSequentialRiccatiEquations(heuristics_);
  backwardPassTimer_.endTimer();

  // calculate controller
  computeControllerTimer_.startTimer();
  // calculate controller. Result is stored in an intermediate variable. The optimized controller, the one after searching, will be
  // swapped back to corresponding primalDataContainer in the search stage
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
void GaussNewtonDDP::runIteration(scalar_t lqModelExpectedCost) {
  // disable Eigen multi-threading
  Eigen::setNbThreads(1);

  // finding the optimal stepLength
  searchStrategyTimer_.startTimer();
  runSearchStrategy(lqModelExpectedCost, nominalPrimalData_);
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
  avgTimeStepBP_ = solveSequentialRiccatiEquations(heuristics_);
  backwardPassTimer_.endTimer();

  // calculate controller
  computeControllerTimer_.startTimer();
  // calculate controller. Result is stored in an intermediate variable. The optimized controller, the one after searching, will be
  // swapped back to corresponding primalDataContainer in the search stage
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
  LinearController noInitialController;
  std::vector<ControllerBase*> noInitialControllerPtrArray{&noInitialController};

  // call the "run" method which uses the internal controllers stock (i.e. nominalControllersStock_)
  runImpl(initTime, initState, finalTime, partitioningTimes, noInitialControllerPtrArray);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaussNewtonDDP::runImpl(scalar_t initTime, const vector_t& initState, scalar_t finalTime, const scalar_array_t& partitioningTimes,
                             const std::vector<ControllerBase*>& externalNominalController) {
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
  // Optimized controller is the result of the previous solve. It is preserved to rollout the initial trajectory of the current solve. But
  // for the very first run, controller is empty and thus need to be initialized.
  if (optimizedPrimalData_.primalSolution.controllerPtr_ == nullptr) {
    optimizedPrimalData_.primalSolution.controllerPtr_.reset(new LinearController);
  }

  // Use the input controller if it is not empty otherwise use the internal controller (nominalControllersStock_).
  // In the later case 2 scenarios are possible: either the internal controller is already set (such as the MPC case
  // where the warm starting option is set true) or the internal controller is empty in which instead of performing
  // a rollout the operating trajectories will be used.
  if (!externalNominalController.empty() && !externalNominalController.front()->empty()) {
    optimizedPrimalData_.primalSolution.controllerPtr_->clear();

    // ensure initial controllers are of the right type, then assign
    const LinearController* linearControllerPtr = dynamic_cast<const LinearController*>(externalNominalController.front());
    if (linearControllerPtr == nullptr) {
      throw std::runtime_error("GaussNewtonDDP::run -- controller must be a LinearController.");
    }
    *optimizedPrimalData_.primalSolution.controllerPtr_ = *linearControllerPtr;
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
  bool unreliableControllerIncrement = optimizedPrimalData_.primalSolution.controllerPtr_->empty();

  // set cost desired trajectories
  for (size_t i = 0; i < ddpSettings_.nThreads_; i++) {
    const auto& targetTrajectories = this->getReferenceManager().getTargetTrajectories();
    optimalControlProblemStock_[i].targetTrajectoriesPtr = &targetTrajectories;
  }

  // display
  if (ddpSettings_.displayInfo_) {
    std::cerr << "\n###################";
    std::cerr << "\n#### Iteration " << (totalNumIterations_ - initIteration) << " (Dynamics might have been violated)";
    std::cerr << "\n###################\n";
  }

  // KEEP THE ORDER!!
  // first swap nominal trajectories (time, state, input, ...) to cache before new rollout
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

    performanceIndexHistory_.push_back(performanceIndex_);

    // run the an iteration of the DDP algorithm and update the member variables
    // the controller which is designed solely based on operation trajectories possibly has invalid feedforward.
    // Therefore the expected cost/merit (calculated by the Riccati solution) is not reliable as well.
    const scalar_t lqModelExpectedCost =
        unreliableControllerIncrement ? performanceIndex_.merit : dualData_.valueFunctionTrajectory.front().f;

    // cache the nominal trajectories before the new rollout (time, state, input, ...)
    swapDataToCache();
    runIteration(lqModelExpectedCost);

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

  performanceIndexHistory_.push_back(performanceIndex_);

  // finding the final optimal stepLength and getting the optimal trajectories and controller
  searchStrategyTimer_.startTimer();
  const scalar_t lqModelExpectedCost = dualData_.valueFunctionTrajectory.front().f;
  runSearchStrategy(lqModelExpectedCost, optimizedPrimalData_);
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
