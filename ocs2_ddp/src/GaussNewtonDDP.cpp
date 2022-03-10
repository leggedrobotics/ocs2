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

#include <ocs2_oc/approximate_model/ChangeOfInputVariables.h>
#include <ocs2_oc/rollout/InitializerRollout.h>
#include <ocs2_oc/trajectory_adjustment/TrajectorySpreadingHelperFunctions.h>

#include <ocs2_ddp/DDP_HelperFunctions.h>
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
  // check OCP
  if (!optimalControlProblem.stateEqualityConstraintPtr->empty()) {
    throw std::runtime_error(
        "[GaussNewtonDDP] DDP does not support intermediate state-only equality constraints (a.k.a. stateEqualityConstraintPtr), instead "
        "use the Lagrangian method!");
  }
  if (!optimalControlProblem.preJumpEqualityConstraintPtr->empty()) {
    throw std::runtime_error(
        "[GaussNewtonDDP] DDP does not support prejump equality constraints (a.k.a. preJumpEqualityConstraintPtr), instead use the "
        "Lagrangian method!");
  }
  if (!optimalControlProblem.finalEqualityConstraintPtr->empty()) {
    throw std::runtime_error(
        "[GaussNewtonDDP] DDP does not support final equality constraints (a.k.a. finalEqualityConstraintPtr), instead use the Lagrangian "
        "method!");
  }

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
                                                      std::move(rolloutRefStock), std::move(problemRefStock), meritFunc));
      break;
    }
    case search_strategy::Type::LEVENBERG_MARQUARDT: {
      constexpr size_t threadID = 0;
      searchStrategyPtr_.reset(new LevenbergMarquardtStrategy(basicStrategySettings, ddpSettings_.levenbergMarquardt_,
                                                              *dynamicsForwardRolloutPtrStock_[threadID],
                                                              optimalControlProblemStock_[threadID], meritFunc));
      break;
    }
  }  // end of switch-case

  // initialize controller
  nominalPrimalData_.primalSolution.controllerPtr_.reset(new LinearController);
  cachedPrimalData_.primalSolution.controllerPtr_.reset(new LinearController);
  optimizedPrimalData_.primalSolution.controllerPtr_.reset(new LinearController);
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
  // search strategy
  searchStrategyPtr_->reset();

  // very important, these are variables that are carried in between iterations
  nominalPrimalData_.clear();
  optimizedPrimalData_.clear();
  cachedPrimalData_.clear();
  dualData_.clear();
  cachedDualData_.clear();

  // initialize Augmented Lagrangian parameters
  initializeConstraintPenalties();

  // performance measures
  avgTimeStepFP_ = 0.0;
  avgTimeStepBP_ = 0.0;
  totalNumIterations_ = 0;
  performanceIndexHistory_.clear();

  // benchmarking timers
  initializationTimer_.reset();
  linearQuadraticApproximationTimer_.reset();
  backwardPassTimer_.reset();
  computeControllerTimer_.reset();
  searchStrategyTimer_.reset();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaussNewtonDDP::getPrimalSolution(scalar_t finalTime, PrimalSolution* primalSolutionPtr) const {
  // total number of nodes
  const int N = optimizedPrimalData_.primalSolution.timeTrajectory_.size();

  auto getRequestedDataLength = [](const scalar_array_t& timeTrajectory, scalar_t time) {
    int index = std::distance(timeTrajectory.cbegin(), std::upper_bound(timeTrajectory.cbegin(), timeTrajectory.cend(), time));
    // fix solution time window to include 1 point beyond requested time
    index += (index != timeTrajectory.size()) ? 1 : 0;
    return index;
  };

  auto getRequestedEventDataLength = [](const size_array_t& postEventIndices, int finalIndex) {
    return std::distance(postEventIndices.cbegin(), std::upper_bound(postEventIndices.cbegin(), postEventIndices.cend(), finalIndex));
  };

  // length of trajectories
  const int length = getRequestedDataLength(optimizedPrimalData_.primalSolution.timeTrajectory_, finalTime);
  const int eventLenght = getRequestedEventDataLength(optimizedPrimalData_.primalSolution.postEventIndices_, length - 1);

  // fill trajectories
  primalSolutionPtr->timeTrajectory_.clear();
  primalSolutionPtr->timeTrajectory_.reserve(length);
  primalSolutionPtr->stateTrajectory_.clear();
  primalSolutionPtr->stateTrajectory_.reserve(length);
  primalSolutionPtr->inputTrajectory_.clear();
  primalSolutionPtr->inputTrajectory_.reserve(length);
  primalSolutionPtr->postEventIndices_.clear();
  primalSolutionPtr->postEventIndices_.reserve(eventLenght);

  primalSolutionPtr->timeTrajectory_.insert(primalSolutionPtr->timeTrajectory_.end(),
                                            optimizedPrimalData_.primalSolution.timeTrajectory_.begin(),
                                            optimizedPrimalData_.primalSolution.timeTrajectory_.begin() + length);
  primalSolutionPtr->stateTrajectory_.insert(primalSolutionPtr->stateTrajectory_.end(),
                                             optimizedPrimalData_.primalSolution.stateTrajectory_.begin(),
                                             optimizedPrimalData_.primalSolution.stateTrajectory_.begin() + length);
  primalSolutionPtr->inputTrajectory_.insert(primalSolutionPtr->inputTrajectory_.end(),
                                             optimizedPrimalData_.primalSolution.inputTrajectory_.begin(),
                                             optimizedPrimalData_.primalSolution.inputTrajectory_.begin() + length);
  primalSolutionPtr->postEventIndices_.insert(primalSolutionPtr->postEventIndices_.end(),
                                              optimizedPrimalData_.primalSolution.postEventIndices_.begin(),
                                              optimizedPrimalData_.primalSolution.postEventIndices_.begin() + eventLenght);

  // fill controller
  if (ddpSettings_.useFeedbackPolicy_) {
    primalSolutionPtr->controllerPtr_.reset(new LinearController);
    // length of the copy
    const int length = getRequestedDataLength(getLinearController(optimizedPrimalData_.primalSolution).timeStamp_, finalTime);
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
    const scalar_t time, const vector_t& state, const PrimalDataContainer& primalData,
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
ScalarFunctionQuadraticApproximation GaussNewtonDDP::getHamiltonian(scalar_t time, const vector_t& state, const vector_t& input) {
  // perform the LQ approximation of the OC problem
  // note that the cost already includes:
  // - state-input intermediate cost
  // - state-input soft constraint cost
  // - state-only intermediate cost
  // - state-only soft constraint cost
  const auto modelData = ocs2::approximateIntermediateLQ(optimalControlProblemStock_[0], time, state, input);

  // check sizes
  if (ddpSettings_.checkNumericalStability_) {
    const auto err = checkSize(modelData, state.rows(), input.rows());
    if (!err.empty()) {
      throw std::runtime_error("[GaussNewtonDDP::getHamiltonian] Mismatch in dimensions at time: " + std::to_string(time) + "\n" + err);
    }
  }

  // initialize the Hamiltonian with the augmented cost
  ScalarFunctionQuadraticApproximation hamiltonian(modelData.cost);

  // add the state-input equality constraint cost nu(x) * g(x,u) to the Hamiltonian
  // note that nu has no approximation and is used as a constant
  const vector_t nu = getStateInputEqualityConstraintLagrangian(time, state);
  hamiltonian.f += nu.dot(modelData.stateInputEqConstraint.f);
  hamiltonian.dfdx.noalias() += modelData.stateInputEqConstraint.dfdx.transpose() * nu;
  hamiltonian.dfdu.noalias() += modelData.stateInputEqConstraint.dfdu.transpose() * nu;
  // dfdxx is zero for the state-input equality constraint cost
  // dfdux is zero for the state-input equality constraint cost
  // dfduu is zero for the state-input equality constraint cost

  // add the "future cost" dVdx(x) * f(x,u) to the Hamiltonian
  const ScalarFunctionQuadraticApproximation V = getValueFunction(time, state);
  const matrix_t dVdxx_dfdx = V.dfdxx.transpose() * modelData.dynamics.dfdx;
  hamiltonian.f += V.dfdx.dot(modelData.dynamics.f);
  hamiltonian.dfdx.noalias() += V.dfdxx.transpose() * modelData.dynamics.f + modelData.dynamics.dfdx.transpose() * V.dfdx;
  hamiltonian.dfdu.noalias() += modelData.dynamics.dfdu.transpose() * V.dfdx;
  hamiltonian.dfdxx.noalias() += dVdxx_dfdx + dVdxx_dfdx.transpose();
  hamiltonian.dfdux.noalias() += modelData.dynamics.dfdu.transpose() * V.dfdxx;
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
      LinearInterpolation::interpolate(indexAlpha, dualData.projectedModelDataTrajectory, model_data::stateInputEqConstraint_f);
  const matrix_t CmProjected =
      LinearInterpolation::interpolate(indexAlpha, dualData.projectedModelDataTrajectory, model_data::stateInputEqConstraint_dfdx);
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
std::vector<std::pair<int, int>> GaussNewtonDDP::getPartitionIntervalsFromTimeTrajectory(const scalar_array_t& timeTrajectory,
                                                                                         int numWorkers) {
  scalar_array_t desiredPartitionPoints(numWorkers + 1);
  desiredPartitionPoints.front() = timeTrajectory.front();

  const scalar_t increment = (timeTrajectory.back() - timeTrajectory.front()) / static_cast<scalar_t>(numWorkers);
  for (size_t i = 1u; i < desiredPartitionPoints.size() - 1; i++) {
    desiredPartitionPoints[i] = desiredPartitionPoints[i - 1] + increment;
  }
  desiredPartitionPoints.back() = timeTrajectory.back();

  std::vector<std::pair<int, int>> partitionIntervals;
  partitionIntervals.reserve(desiredPartitionPoints.size());

  int endPos, startPos = 0;
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
void GaussNewtonDDP::rolloutInitialTrajectory(PrimalDataContainer& primalData, ControllerBase* controller, size_t workerIndex /*= 0*/) {
  assert(primalData.primalSolution.controllerPtr_.get() != controller);
  // clear output
  primalData.clear();
  // for non-StateTriggeredRollout initialize modeSchedule
  primalData.primalSolution.modeSchedule_ = this->getReferenceManager().getModeSchedule();

  // create alias
  auto& modeSchedule = primalData.primalSolution.modeSchedule_;
  auto& timeTrajectory = primalData.primalSolution.timeTrajectory_;
  auto& stateTrajectory = primalData.primalSolution.stateTrajectory_;
  auto& inputTrajectory = primalData.primalSolution.inputTrajectory_;
  auto& postEventIndices = primalData.primalSolution.postEventIndices_;

  // divide the rollout segment in controller rollout and operating points
  const auto controllerAvailableTill = controller->empty() ? initTime_ : static_cast<LinearController*>(controller)->timeStamp_.back();
  const auto controllerRolloutFromTo = std::make_pair(initTime_, std::max(initTime_, std::min(controllerAvailableTill, finalTime_)));
  const auto operatingPointsFromTo = std::make_pair(controllerRolloutFromTo.second, finalTime_);

  // display
  if (ddpSettings_.debugPrintRollout_) {
    std::cerr << "[GaussNewtonDDP::rolloutInitialTrajectory] for t = [" << initTime_ << ", " << finalTime_ << "]\n";
    std::cerr << "\tcontroller available till t = " << controllerAvailableTill << "\n";
    if (controllerRolloutFromTo.first < controllerRolloutFromTo.second) {
      std::cerr << "\twill use controller for t = [" << controllerRolloutFromTo.first << ", " << controllerRolloutFromTo.second << "]\n";
    }
    if (operatingPointsFromTo.first < operatingPointsFromTo.second) {
      std::cerr << "\twill use operating points for t = [" << operatingPointsFromTo.first << ", " << operatingPointsFromTo.second << "]\n";
    }
  }

  // rollout with controller
  vector_t xCurrent = initState_;
  if (controllerRolloutFromTo.first < controllerRolloutFromTo.second) {
    xCurrent = dynamicsForwardRolloutPtrStock_[workerIndex]->run(controllerRolloutFromTo.first, initState_, controllerRolloutFromTo.second,
                                                                 controller, modeSchedule, timeTrajectory, postEventIndices,
                                                                 stateTrajectory, inputTrajectory);
  }

  // finish rollout with operating points
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
                                                             modeSchedule, timeTrajectoryTail, eventsPastTheEndIndecesTail,
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

  if (!xCurrent.allFinite()) {
    throw std::runtime_error("[GaussNewtonDDP::rolloutInitialTrajectory] System became unstable during the initial rollout!");
  }

  // debug print
  if (ddpSettings_.debugPrintRollout_) {
    std::cerr << "\n++++++++++++++++++++++++++++++++++++++++\n";
    std::cerr << "[GaussNewtonDDP::rolloutInitialTrajectory] for t = [" << timeTrajectory.front() << ", " << timeTrajectory.back() << "]";
    std::cerr << "\n+++++++++++++++++++++++++++++++++++++++++\n";
    RolloutBase::display(timeTrajectory, postEventIndices, stateTrajectory, &inputTrajectory);
  }
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
  // cost
  scalar_t merit = performanceIndex.cost;
  // state/state-input equality constraints
  merit += constraintPenaltyCoefficients_.penaltyCoeff * std::sqrt(performanceIndex.equalityConstraintsSSE);
  // state/state-input equality Lagrangian
  merit += performanceIndex.equalityLagrangian;
  // state/state-input inequality Lagrangian
  merit += performanceIndex.inequalityLagrangian;

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

  // the last index of the partition is excluded, namely [first, last), so the value function approximation of the end point of the end
  // partition is filled manually.
  // For other partitions except the last one, the end points are filled in the solving stage of the next partition. For example,
  // [first1,last1), [first2(last1), last2).
  dualData_.valueFunctionTrajectory.back() = finalValueFunction;

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
    auto task = [this, &partitionIntervals, &finalValueFunctionOfEachPartition]() {
      const size_t taskId = nextTaskId_++;  // assign task ID (atomic)
      riccatiEquationsWorker(taskId, partitionIntervals[taskId], finalValueFunctionOfEachPartition[taskId]);
    };
    runParallel(task, partitionIntervals.size());
  }

  // testing the numerical stability of the Riccati equations
  if (ddpSettings_.checkNumericalStability_) {
    int N = nominalPrimalData_.primalSolution.timeTrajectory_.size();
    for (int k = N - 1; k >= 0; k--) {
      try {
        const auto& valueFunction = dualData_.valueFunctionTrajectory[k];
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
  return (finalTime_ - initTime_) / static_cast<scalar_t>(outputN);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaussNewtonDDP::calculateController() {
  const size_t N = nominalPrimalData_.primalSolution.timeTrajectory_.size();

  unoptimizedController_.clear();
  unoptimizedController_.timeStamp_ = nominalPrimalData_.primalSolution.timeTrajectory_;
  unoptimizedController_.gainArray_.resize(N);
  unoptimizedController_.biasArray_.resize(N);
  unoptimizedController_.deltaBiasArray_.resize(N);

  nextTimeIndex_ = 0;
  auto task = [this, N] {
    int timeIndex;
    // get next time index (atomic)
    while ((timeIndex = nextTimeIndex_++) < N) {
      calculateControllerWorker(timeIndex, nominalPrimalData_, dualData_, unoptimizedController_);
    }
  };
  runParallel(task, ddpSettings_.nThreads_);

  // Since the controller for the last timestamp is invalid, if the last time is not the event time, use the control policy of the second to
  // last time for the last time
  const bool finalTimeIsNotAnEvent =
      nominalPrimalData_.primalSolution.postEventIndices_.empty() ||
      (nominalPrimalData_.primalSolution.postEventIndices_.back() != nominalPrimalData_.primalSolution.timeTrajectory_.size() - 1);
  // finalTimeIsNotAnEvent && there are at least two time stamps
  if (finalTimeIsNotAnEvent && unoptimizedController_.size() >= 2) {
    const size_t secondToLastIndex = unoptimizedController_.size() - 2u;
    unoptimizedController_.gainArray_.back() = unoptimizedController_.gainArray_[secondToLastIndex];
    unoptimizedController_.biasArray_.back() = unoptimizedController_.biasArray_[secondToLastIndex];
    unoptimizedController_.deltaBiasArray_.back() = unoptimizedController_.deltaBiasArray_[secondToLastIndex];
  }

  // checking the numerical stability of the controller parameters
  if (settings().checkNumericalStability_) {
    for (int timeIndex = 0; timeIndex < unoptimizedController_.size(); timeIndex++) {
      std::stringstream errorDescription;
      if (!unoptimizedController_.gainArray_[timeIndex].allFinite()) {
        errorDescription << "Feedback gains are unstable!\n";
      }
      if (!unoptimizedController_.deltaBiasArray_[timeIndex].allFinite()) {
        errorDescription << "Feedforward control is unstable!\n";
      }
      if (errorDescription.tellp() != 0) {
        std::stringstream errorMessage;
        errorMessage << "At time " << unoptimizedController_.timeStamp_[timeIndex] << " [sec].\n" << errorDescription.str();
        throw std::runtime_error(errorMessage.str());
      }
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t GaussNewtonDDP::maxControllerUpdateNorm(const LinearController& controller) const {
  scalar_t maxDeltaUffNorm = 0.0;
  for (const auto& deltaBias : controller.deltaBiasArray_) {
    maxDeltaUffNorm = std::max(maxDeltaUffNorm, deltaBias.norm());
  }
  return maxDeltaUffNorm;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaussNewtonDDP::approximateOptimalControlProblem() {
  /*
   * compute and augment the LQ approximation of intermediate times
   */
  // perform the LQ approximation for intermediate times
  approximateIntermediateLQ(nominalPrimalData_);

  /*
   * compute and augment the LQ approximation of the event times.
   * also call shiftHessian on the event time's cost 2nd order derivative.
   */
  const size_t NE = nominalPrimalData_.primalSolution.postEventIndices_.size();
  nominalPrimalData_.modelDataEventTimes.clear();
  nominalPrimalData_.modelDataEventTimes.resize(NE);
  if (NE > 0) {
    nextTimeIndex_ = 0;
    nextTaskId_ = 0;
    auto task = [this, NE]() {
      const size_t taskId = nextTaskId_++;  // assign task ID (atomic)

      // timeIndex is atomic
      int timeIndex;
      while ((timeIndex = nextTimeIndex_++) < NE) {
        ModelData& modelData = nominalPrimalData_.modelDataEventTimes[timeIndex];
        const size_t preEventIndex = nominalPrimalData_.primalSolution.postEventIndices_[timeIndex] - 1;
        const auto& time = nominalPrimalData_.primalSolution.timeTrajectory_[preEventIndex];
        const auto& state = nominalPrimalData_.primalSolution.stateTrajectory_[preEventIndex];

        // approximate LQ for the pre-event node
        ocs2::approximatePreJumpLQ(optimalControlProblemStock_[taskId], time, state, modelData);

        // checking the numerical properties
        if (ddpSettings_.checkNumericalStability_) {
          const auto errSize = checkSize(modelData, state.rows(), 0);
          if (!errSize.empty()) {
            throw std::runtime_error("[GaussNewtonDDP::approximateOptimalControlProblem] Mismatch in dimensions at intermediate time: " +
                                     std::to_string(time) + "\n" + errSize);
          }
          const std::string errProperties =
              checkDynamicsProperties(modelData) + checkCostProperties(modelData) + checkConstraintProperties(modelData);
          if (!errProperties.empty()) {
            throw std::runtime_error("[GaussNewtonDDP::approximateOptimalControlProblem] Ill-posed problem at event time: " +
                                     std::to_string(time) + "\n" + errProperties);
          }
        }

        // shift Hessian
        if (ddpSettings_.strategy_ == search_strategy::Type::LINE_SEARCH) {
          hessian_correction::shiftHessian(ddpSettings_.lineSearch_.hessianCorrectionStrategy_, modelData.cost.dfdxx,
                                           ddpSettings_.lineSearch_.hessianCorrectionMultiple_);
        }
      }
    };
    runParallel(task, ddpSettings_.nThreads_);
  }

  /*
   * compute the Heuristics function at the final time. Also call shiftHessian on the Heuristics 2nd order derivative.
   */
  if (!nominalPrimalData_.primalSolution.timeTrajectory_.empty()) {
    const auto& time = nominalPrimalData_.primalSolution.timeTrajectory_.back();
    const auto& state = nominalPrimalData_.primalSolution.stateTrajectory_.back();
    auto modelData = ocs2::approximateFinalLQ(optimalControlProblemStock_[0], time, state);

    // checking the numerical properties
    if (ddpSettings_.checkNumericalStability_) {
      const std::string err = checkCostProperties(modelData) + checkConstraintProperties(modelData);
      if (!err.empty()) {
        throw std::runtime_error(
            "[GaussNewtonDDP::approximateOptimalControlProblem] Ill-posed problem at final time: " + std::to_string(time) + "\n" + err);
      }
    }

    heuristics_ = std::move(modelData.cost);

    // shift Hessian for final time
    if (ddpSettings_.strategy_ == search_strategy::Type::LINE_SEARCH) {
      hessian_correction::shiftHessian(ddpSettings_.lineSearch_.hessianCorrectionStrategy_, heuristics_.dfdxx,
                                       ddpSettings_.lineSearch_.hessianCorrectionMultiple_);
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaussNewtonDDP::computeProjectionAndRiccatiModification(const ModelData& modelData, const matrix_t& Sm, ModelData& projectedModelData,
                                                             riccati_modification::Data& riccatiModification) const {
  // compute the Hamiltonian's Hessian
  riccatiModification.time_ = modelData.time;
  riccatiModification.hamiltonianHessian_ = computeHamiltonianHessian(modelData, Sm);

  // compute projectors
  computeProjections(riccatiModification.hamiltonianHessian_, modelData.stateInputEqConstraint.dfdu,
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
  projectedModelData.time = modelData.time;
  projectedModelData.stateDim = modelData.stateDim;
  projectedModelData.inputDim = modelData.inputDim - modelData.stateInputEqConstraint.f.rows();

  // unhandled constraints
  projectedModelData.stateEqConstraint.f = vector_t();

  if (modelData.stateInputEqConstraint.f.rows() == 0) {
    // Change of variables u = Pu * tilde{u}
    // Pu = constraintNullProjector;

    // projected state-input equality constraints
    projectedModelData.stateInputEqConstraint.f.setZero(projectedModelData.inputDim);
    projectedModelData.stateInputEqConstraint.dfdx.setZero(projectedModelData.inputDim, projectedModelData.stateDim);
    projectedModelData.stateInputEqConstraint.dfdu.setZero(modelData.inputDim, modelData.inputDim);

    // dynamics
    projectedModelData.dynamics = modelData.dynamics;
    changeOfInputVariables(projectedModelData.dynamics, constraintNullProjector);

    // dynamics bias
    projectedModelData.dynamicsBias = modelData.dynamicsBias;

    // cost
    projectedModelData.cost = modelData.cost;
    changeOfInputVariables(projectedModelData.cost, constraintNullProjector);

  } else {
    // Change of variables u = Pu * tilde{u} + Px * x + u0
    // Pu = constraintNullProjector;
    // Px (= -CmProjected) = -constraintRangeProjector * C
    // u0 (= -EvProjected) = -constraintRangeProjector * e

    /* projected state-input equality constraints */
    projectedModelData.stateInputEqConstraint.f.noalias() = constraintRangeProjector * modelData.stateInputEqConstraint.f;
    projectedModelData.stateInputEqConstraint.dfdx.noalias() = constraintRangeProjector * modelData.stateInputEqConstraint.dfdx;
    projectedModelData.stateInputEqConstraint.dfdu.noalias() = constraintRangeProjector * modelData.stateInputEqConstraint.dfdu;

    // Change of variable matrices
    const auto& Pu = constraintNullProjector;
    const matrix_t Px = -projectedModelData.stateInputEqConstraint.dfdx;
    const matrix_t u0 = -projectedModelData.stateInputEqConstraint.f;

    // dynamics
    projectedModelData.dynamics = modelData.dynamics;
    changeOfInputVariables(projectedModelData.dynamics, Pu, Px, u0);

    // dynamics bias
    projectedModelData.dynamicsBias = modelData.dynamicsBias;
    projectedModelData.dynamicsBias.noalias() += modelData.dynamics.dfdu * u0;

    // cost
    projectedModelData.cost = modelData.cost;
    changeOfInputVariables(projectedModelData.cost, Pu, Px, u0);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaussNewtonDDP::initializeConstraintPenalties() {
  assert(ddpSettings_.constraintPenaltyInitialValue_ > 1.0);
  assert(ddpSettings_.constraintPenaltyIncreaseRate_ > 1.0);

  constraintPenaltyCoefficients_.penaltyCoeff = ddpSettings_.constraintPenaltyInitialValue_;
  constraintPenaltyCoefficients_.penaltyTol = 1.0 / std::pow(constraintPenaltyCoefficients_.penaltyCoeff, 0.1);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaussNewtonDDP::runSearchStrategy(scalar_t lqModelExpectedCost, const LinearController& unoptimizedController,
                                       PrimalDataContainer& primalData, PerformanceIndex& performanceIndex, MetricsCollection& metrics) {
  const auto& modeSchedule = this->getReferenceManager().getModeSchedule();

  // Primal solution controller is now optimized.
  scalar_t avgTimeStep;
  search_strategy::SolutionRef solution(primalData.primalSolution, performanceIndex, metrics, avgTimeStep);
  const bool success =
      searchStrategyPtr_->run({initTime_, finalTime_}, initState_, lqModelExpectedCost, unoptimizedController, modeSchedule, solution);
  avgTimeStepFP_ = 0.9 * avgTimeStepFP_ + 0.1 * avgTimeStep;

  // If fail, copy the entire cache back. To keep the consistency of cached data, all cache should be left untouched.
  if (!success) {
    primalData = cachedPrimalData_;
    performanceIndex = performanceIndexHistory_.back();
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaussNewtonDDP::updateConstraintPenalties(scalar_t equalityConstraintsSSE) {
  // state-input equality penalty
  if (equalityConstraintsSSE < constraintPenaltyCoefficients_.penaltyTol) {
    // tighten tolerance
    constraintPenaltyCoefficients_.penaltyTol /= std::pow(constraintPenaltyCoefficients_.penaltyCoeff, 0.9);
  } else {
    // tighten tolerance & increase penalty
    constraintPenaltyCoefficients_.penaltyCoeff *= ddpSettings_.constraintPenaltyIncreaseRate_;
    constraintPenaltyCoefficients_.penaltyTol /= std::pow(constraintPenaltyCoefficients_.penaltyCoeff, 0.1);
  }
  constraintPenaltyCoefficients_.penaltyTol = std::max(constraintPenaltyCoefficients_.penaltyTol, ddpSettings_.constraintTolerance_);

  // display
  if (ddpSettings_.displayInfo_) {
    std::string displayText = "Equality Constraints Penalty Parameters:\n";
    displayText += "    Penalty Tolerance: " + std::to_string(constraintPenaltyCoefficients_.penaltyTol);
    displayText += "    Penalty Coefficient: " + std::to_string(constraintPenaltyCoefficients_.penaltyCoeff) + ".\n";

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
void GaussNewtonDDP::runInit() {
  // disable Eigen multi-threading
  Eigen::setNbThreads(1);

  // initial controller rollout
  initializationTimer_.startTimer();
  try {
    constexpr size_t taskId = 0;
    constexpr scalar_t stepLength = 0.0;
    // perform a rollout
    // Nominal controller is stored in the optimized primal data as it is either the result of previous solve or it is provided by user and
    // copied to optimized data container manually at the beginning of runImpl
    rolloutInitialTrajectory(nominalPrimalData_, optimizedPrimalData_.primalSolution.controllerPtr_.get(), taskId);
    // swap controller used to rollout the nominal trajectories back to nominal data container.
    nominalPrimalData_.primalSolution.controllerPtr_.swap(optimizedPrimalData_.primalSolution.controllerPtr_);

    computeRolloutMetrics(optimalControlProblemStock_[taskId], nominalPrimalData_.primalSolution, metrics_);

    performanceIndex_ = computeRolloutPerformanceIndex(nominalPrimalData_.primalSolution.timeTrajectory_, metrics_);

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
  updateConstraintPenalties(0.0);

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
  runSearchStrategy(lqModelExpectedCost, unoptimizedController_, nominalPrimalData_, performanceIndex_, metrics_);
  searchStrategyTimer_.endTimer();

  // update the constraint penalty coefficients
  updateConstraintPenalties(performanceIndex_.equalityConstraintsSSE);

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
void GaussNewtonDDP::runImpl(scalar_t initTime, const vector_t& initState, scalar_t finalTime,
                             const ControllerBase* externalControllerPtr) {
  if (ddpSettings_.displayInfo_) {
    std::cerr << "\n++++++++++++++++++++++++++++++++++++++++++++++++++++++";
    std::cerr << "\n+++++++++++++ " + ddp::toAlgorithmName(ddpSettings_.algorithm_) + " solver is initialized ++++++++++++++";
    std::cerr << "\n++++++++++++++++++++++++++++++++++++++++++++++++++++++\n";
    std::cerr << "\nSolver starts from initial time " << initTime << " to final time " << finalTime << ".\n";
    std::cerr << this->getReferenceManager().getModeSchedule() << "\n";
  }

  // Use the input controller if it is not empty otherwise use the internal controller. In the later case two scenarios are
  // possible: either the internal controller is already set (such as the MPC case where the warm starting option is set true)
  // or the internal controller is empty in which instead of performing a rollout the operating trajectories will be used.
  if (externalControllerPtr != nullptr) {
    // ensure initial controllers are of the right type, then assign
    const LinearController* linearControllerPtr = dynamic_cast<const LinearController*>(externalControllerPtr);
    if (linearControllerPtr == nullptr) {
      throw std::runtime_error("[GaussNewtonDDP::run] controller must be a LinearController type!");
    }
    *optimizedPrimalData_.primalSolution.controllerPtr_ = *linearControllerPtr;
  }

  initState_ = initState;
  initTime_ = initTime;
  finalTime_ = finalTime;
  performanceIndexHistory_.clear();
  const auto initIteration = totalNumIterations_;

  // adjust controller
  if (!optimizedPrimalData_.primalSolution.controllerPtr_->empty()) {
    std::ignore = trajectorySpread(optimizedPrimalData_.primalSolution.modeSchedule_, getReferenceManager().getModeSchedule(),
                                   getLinearController(optimizedPrimalData_.primalSolution));
  }

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

  // swap nominal trajectories (time, state, input, ...) to cache before new rollout
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
      std::cerr << "max feedforward norm: " << maxControllerUpdateNorm(unoptimizedController_) << "\n";
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
    std::cerr << "max feedforward norm: " << maxControllerUpdateNorm(unoptimizedController_) << "\n";
  }

  performanceIndexHistory_.push_back(performanceIndex_);

  // finding the final optimal stepLength and getting the optimal trajectories and controller
  searchStrategyTimer_.startTimer();
  const scalar_t lqModelExpectedCost = dualData_.valueFunctionTrajectory.front().f;
  runSearchStrategy(lqModelExpectedCost, unoptimizedController_, optimizedPrimalData_, performanceIndex_, metrics_);
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
