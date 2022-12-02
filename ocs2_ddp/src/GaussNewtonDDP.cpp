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

#include <ocs2_oc/oc_problem/OptimalControlProblemHelperFunction.h>
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
  Eigen::setNbThreads(1);  // no multithreading within Eigen.
  Eigen::initParallel();

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

  // initializer Rollout
  initializerRolloutPtr_.reset(new InitializerRollout(initializer, rollout.settings()));

  // initialize rollout and OCP instances for multi-thread compuation
  optimalControlProblemStock_.reserve(ddpSettings_.nThreads_);
  dynamicsForwardRolloutPtrStock_.reserve(ddpSettings_.nThreads_);
  for (size_t i = 0; i < ddpSettings_.nThreads_; i++) {
    optimalControlProblemStock_.push_back(optimalControlProblem);
    dynamicsForwardRolloutPtrStock_.emplace_back(rollout.clone());
  }  // end of i loop

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
  optimizedPrimalSolution_.controllerPtr_.reset(new LinearController);
  nominalPrimalData_.primalSolution.controllerPtr_.reset(new LinearController);
  cachedPrimalData_.primalSolution.controllerPtr_.reset(new LinearController);
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
  const auto dualSolutionTotal = totalDualSolutionTimer_.getTotalInMilliseconds();

  const auto benchmarkTotal = initializationTotal + linearQuadraticApproximationTotal + backwardPassTotal + computeControllerTotal +
                              searchStrategyTotal + dualSolutionTotal;

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
               << searchStrategyTotal / benchmarkTotal * 100 << "%)\n";
    infoStream << "\tDual Solution      :\t" << totalDualSolutionTimer_.getAverageInMilliseconds() << " [ms] \t\t("
               << dualSolutionTotal / benchmarkTotal * 100 << "%)\n\n";
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
  nominalDualData_.clear();
  nominalPrimalData_.clear();
  cachedDualData_.clear();
  cachedPrimalData_.clear();

  // optimized data
  optimizedDualSolution_.clear();
  optimizedPrimalSolution_.clear();
  optimizedProblemMetrics_.clear();

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
  totalDualSolutionTimer_.reset();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaussNewtonDDP::getPrimalSolution(scalar_t finalTime, PrimalSolution* primalSolutionPtr) const {
  // total number of nodes
  const int N = optimizedPrimalSolution_.timeTrajectory_.size();

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
  const int length = getRequestedDataLength(optimizedPrimalSolution_.timeTrajectory_, finalTime);
  const int eventLenght = getRequestedEventDataLength(optimizedPrimalSolution_.postEventIndices_, length - 1);

  // fill trajectories
  primalSolutionPtr->timeTrajectory_.clear();
  primalSolutionPtr->timeTrajectory_.reserve(length);
  primalSolutionPtr->stateTrajectory_.clear();
  primalSolutionPtr->stateTrajectory_.reserve(length);
  primalSolutionPtr->inputTrajectory_.clear();
  primalSolutionPtr->inputTrajectory_.reserve(length);
  primalSolutionPtr->postEventIndices_.clear();
  primalSolutionPtr->postEventIndices_.reserve(eventLenght);

  primalSolutionPtr->timeTrajectory_.insert(primalSolutionPtr->timeTrajectory_.end(), optimizedPrimalSolution_.timeTrajectory_.begin(),
                                            optimizedPrimalSolution_.timeTrajectory_.begin() + length);
  primalSolutionPtr->stateTrajectory_.insert(primalSolutionPtr->stateTrajectory_.end(), optimizedPrimalSolution_.stateTrajectory_.begin(),
                                             optimizedPrimalSolution_.stateTrajectory_.begin() + length);
  primalSolutionPtr->inputTrajectory_.insert(primalSolutionPtr->inputTrajectory_.end(), optimizedPrimalSolution_.inputTrajectory_.begin(),
                                             optimizedPrimalSolution_.inputTrajectory_.begin() + length);
  primalSolutionPtr->postEventIndices_.insert(primalSolutionPtr->postEventIndices_.end(),
                                              optimizedPrimalSolution_.postEventIndices_.begin(),
                                              optimizedPrimalSolution_.postEventIndices_.begin() + eventLenght);

  // fill controller
  if (ddpSettings_.useFeedbackPolicy_) {
    primalSolutionPtr->controllerPtr_.reset(new LinearController);
    // length of the copy
    const int length = getRequestedDataLength(getLinearController(optimizedPrimalSolution_).timeStamp_, finalTime);
    primalSolutionPtr->controllerPtr_->concatenate(optimizedPrimalSolution_.controllerPtr_.get(), 0, length);

  } else {
    primalSolutionPtr->controllerPtr_.reset(
        new FeedforwardController(primalSolutionPtr->timeTrajectory_, primalSolutionPtr->inputTrajectory_));
  }

  // fill mode schedule
  primalSolutionPtr->modeSchedule_ = optimizedPrimalSolution_.modeSchedule_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation GaussNewtonDDP::getValueFunctionImpl(
    const scalar_t time, const vector_t& state, const PrimalSolution& primalSolution,
    const std::vector<ScalarFunctionQuadraticApproximation>& valueFunctionTrajectory) const {
  // result
  ScalarFunctionQuadraticApproximation valueFunction;
  const auto indexAlpha = LinearInterpolation::timeSegment(time, primalSolution.timeTrajectory_);
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
  const vector_t xNominal = LinearInterpolation::interpolate(indexAlpha, primalSolution.stateTrajectory_);
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
  const ModelData modelData = [&]() {
    const auto multiplierCollection = getIntermediateDualSolution(time);
    return ocs2::approximateIntermediateLQ(optimalControlProblemStock_[0], time, state, input, multiplierCollection);
  }();

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
bool GaussNewtonDDP::rolloutInitialController(PrimalSolution& inputPrimalSolution, PrimalSolution& outputPrimalSolution) {
  if (inputPrimalSolution.controllerPtr_->empty()) {
    return false;
  }

  // cast to linear the controller
  auto& inputLinearController = getLinearController(inputPrimalSolution);

  // adjust in-place the controller
  std::ignore = trajectorySpread(inputPrimalSolution.modeSchedule_, getReferenceManager().getModeSchedule(), inputLinearController);
  // after adjustment it might become empty
  if (inputLinearController.empty()) {
    return false;
  }

  const auto finalTime = std::max(initTime_, std::min(inputLinearController.timeStamp_.back(), finalTime_));

  if (initTime_ < finalTime) {
    if (ddpSettings_.debugPrintRollout_) {
      std::cerr << "[GaussNewtonDDP::rolloutInitialController] for t = [" << initTime_ << ", " << finalTime_ << "]\n";
      std::cerr << "\twill use controller for t = [" << initTime_ << ", " << finalTime << "]\n";
    }
    outputPrimalSolution.controllerPtr_.swap(inputPrimalSolution.controllerPtr_);
    std::ignore = rolloutTrajectory(*dynamicsForwardRolloutPtrStock_[0], initTime_, initState_, finalTime, outputPrimalSolution);
    return true;

  } else {
    return false;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool GaussNewtonDDP::extractInitialTrajectories(PrimalSolution& inputPrimalSolution, PrimalSolution& outputPrimalSolution) {
  if (inputPrimalSolution.timeTrajectory_.empty()) {
    return false;
  }

  // adjust in-place the primalSolution
  std::ignore = trajectorySpread(inputPrimalSolution.modeSchedule_, getReferenceManager().getModeSchedule(), inputPrimalSolution);
  // after adjustment it might become empty
  if (inputPrimalSolution.timeTrajectory_.empty()) {
    return false;
  }

  const auto finalTime = std::max(initTime_, std::min(inputPrimalSolution.timeTrajectory_.back(), finalTime_));

  if (initTime_ < finalTime) {
    if (ddpSettings_.debugPrintRollout_) {
      std::cerr << "[GaussNewtonDDP::extractInitialTrajectories] for t = [" << initTime_ << ", " << finalTime_ << "]\n";
      std::cerr << "\twill use PrimalSolution trajectory for t = [" << initTime_ << ", " << finalTime << "]\n";
    }
    extractPrimalSolution({initTime_, finalTime}, inputPrimalSolution, outputPrimalSolution);
    return true;

  } else {
    return false;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaussNewtonDDP::rolloutInitializer(PrimalSolution& primalSolution) {
  // create alias
  auto& modeSchedule = primalSolution.modeSchedule_;
  auto& timeTrajectory = primalSolution.timeTrajectory_;
  auto& stateTrajectory = primalSolution.stateTrajectory_;
  auto& inputTrajectory = primalSolution.inputTrajectory_;
  auto& postEventIndices = primalSolution.postEventIndices_;

  // finish rollout with Initializer
  if (timeTrajectory.empty() || timeTrajectory.back() < finalTime_) {
    scalar_t initTime = timeTrajectory.empty() ? initTime_ : timeTrajectory.back();
    const vector_t initState = stateTrajectory.empty() ? initState_ : stateTrajectory.back();

    // Remove last point of the rollout if it is directly past an event. Here where we want to use the Initializer
    // instead. However, we do start the integration at the state after the event. i.e. the jump map remains applied.
    if (!postEventIndices.empty() && postEventIndices.back() == (timeTrajectory.size() - 1)) {
      // Start new integration at the time point after the event
      timeTrajectory.pop_back();
      stateTrajectory.pop_back();
      inputTrajectory.pop_back();
      // eventsPastTheEndIndeces is not removed because we need to mark the start of the operatingPointTrajectory as being after an event.

      // adjusting the start time to correct for subsystem recognition
      constexpr auto eps = numeric_traits::weakEpsilon<scalar_t>();
      initTime = std::min(initTime + eps, finalTime_);
    }

    scalar_array_t timeTrajectoryTail;
    size_array_t postEventIndicesTail;
    vector_array_t stateTrajectoryTail;
    vector_array_t inputTrajectoryTail;
    std::ignore = initializerRolloutPtr_->run(initTime, initState, finalTime_, nullptr, modeSchedule, timeTrajectoryTail,
                                              postEventIndicesTail, stateTrajectoryTail, inputTrajectoryTail);

    // Add controller rollout length to event past the indeces
    for (auto& eventIndex : postEventIndicesTail) {
      eventIndex += stateTrajectory.size();  // the size of the old trajectory is missing when counting event's index
    }

    // Concatenate
    timeTrajectory.insert(timeTrajectory.end(), timeTrajectoryTail.begin(), timeTrajectoryTail.end());
    postEventIndices.insert(postEventIndices.end(), postEventIndicesTail.begin(), postEventIndicesTail.end());
    stateTrajectory.insert(stateTrajectory.end(), stateTrajectoryTail.begin(), stateTrajectoryTail.end());
    inputTrajectory.insert(inputTrajectory.end(), inputTrajectoryTail.begin(), inputTrajectoryTail.end());
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
  nominalDualData_.valueFunctionTrajectory.clear();
  nominalDualData_.valueFunctionTrajectory.resize(outputN);

  // the last index of the partition is excluded, namely [first, last), so the value function approximation of the end point of the end
  // partition is filled manually.
  // For other partitions except the last one, the end points are filled in the solving stage of the next partition. For example,
  // [first1,last1), [first2(last1), last2).
  nominalDualData_.valueFunctionTrajectory.back() = finalValueFunction;

  // solve it sequentially for the first iteration
  if (totalNumIterations_ == 0) {
    const std::pair<int, int> partitionInterval{0, outputN - 1};
    riccatiEquationsWorker(0, partitionInterval, finalValueFunction);
  } else {  // solve it in parallel
    // do equal-time partitions based on available thread resource
    const auto partitionIntervals = computePartitionIntervals(nominalPrimalData_.primalSolution.timeTrajectory_, ddpSettings_.nThreads_);

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
    const int N = nominalPrimalData_.primalSolution.timeTrajectory_.size();
    for (int k = N - 1; k >= 0; k--) {
      // check size
      auto errorDescription = checkSize(nominalPrimalData_.primalSolution.stateTrajectory_[k].size(), 0,
                                        nominalDualData_.valueFunctionTrajectory[k], "ValueFunction");
      if (!errorDescription.empty()) {
        throw std::runtime_error(errorDescription);
      }
      // check PSD
      errorDescription = checkBeingPSD(nominalDualData_.valueFunctionTrajectory[k], "ValueFunction");
      if (!errorDescription.empty()) {
        std::stringstream throwMsg;
        throwMsg << "at time " << nominalPrimalData_.primalSolution.timeTrajectory_[k] << ":\n";
        throwMsg << errorDescription << "The error takes place in the following segment of trajectory:\n";
        for (int kp = k; kp < std::min(k + 10, N); kp++) {
          throwMsg << ">>> time: " << nominalPrimalData_.primalSolution.timeTrajectory_[kp] << "\n";
          throwMsg << "|| Sm ||:\t" << nominalDualData_.valueFunctionTrajectory[kp].dfdxx.norm() << "\n";
          throwMsg << "|| Sv ||:\t" << nominalDualData_.valueFunctionTrajectory[kp].dfdx.transpose().norm() << "\n";
          throwMsg << "   s    :\t" << nominalDualData_.valueFunctionTrajectory[kp].f << "\n";
        }
        throw std::runtime_error(throwMsg.str());
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
      calculateControllerWorker(timeIndex, nominalPrimalData_, nominalDualData_, unoptimizedController_);
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

  // display
  if (ddpSettings_.displayInfo_) {
    std::cerr << "max feedforward norm: " << maxControllerUpdateNorm(unoptimizedController_) << "\n";
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaussNewtonDDP::approximateOptimalControlProblem() {
  /*
   * compute and augment the LQ approximation of intermediate times
   */
  // perform the LQ approximation for intermediate times
  approximateIntermediateLQ(nominalDualData_.dualSolution, nominalPrimalData_);

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
        const auto& multiplier = nominalDualData_.dualSolution.preJumps[timeIndex];

        // approximate LQ for the pre-event node
        ocs2::approximatePreJumpLQ(optimalControlProblemStock_[taskId], time, state, multiplier, modelData);

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
          hessian_correction::shiftHessian(ddpSettings_.lineSearch_.hessianCorrectionStrategy, modelData.cost.dfdxx,
                                           ddpSettings_.lineSearch_.hessianCorrectionMultiple);
        }
      }
    };
    runParallel(task, ddpSettings_.nThreads_);
  }

  /*
   * compute the Heuristics function at the final time. Also call shiftHessian on the Heuristics 2nd order derivative.
   */
  if (!nominalPrimalData_.primalSolution.timeTrajectory_.empty()) {
    ModelData& modelData = nominalPrimalData_.modelDataFinalTime;
    const auto& time = nominalPrimalData_.primalSolution.timeTrajectory_.back();
    const auto& state = nominalPrimalData_.primalSolution.stateTrajectory_.back();
    const auto& multiplier = nominalDualData_.dualSolution.final;
    modelData = ocs2::approximateFinalLQ(optimalControlProblemStock_[0], time, state, multiplier);

    // checking the numerical properties
    if (ddpSettings_.checkNumericalStability_) {
      const std::string err = checkCostProperties(modelData) + checkConstraintProperties(modelData);
      if (!err.empty()) {
        throw std::runtime_error(
            "[GaussNewtonDDP::approximateOptimalControlProblem] Ill-posed problem at final time: " + std::to_string(time) + "\n" + err);
      }
    }

    // shift Hessian for final time
    if (ddpSettings_.strategy_ == search_strategy::Type::LINE_SEARCH) {
      hessian_correction::shiftHessian(ddpSettings_.lineSearch_.hessianCorrectionStrategy, modelData.cost.dfdxx,
                                       ddpSettings_.lineSearch_.hessianCorrectionMultiple);
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
    // constraint projectors are obtained at once
    matrix_t DmDaggerTHmDmDaggerUUT;
    ocs2::LinearAlgebra::computeConstraintProjection(Dm, HmInvUmUmT, constraintRangeProjector, DmDaggerTHmDmDaggerUUT,
                                                     constraintNullProjector);
  }

  // check
  if (ddpSettings_.checkNumericalStability_) {
    matrix_t HmProjected = constraintNullProjector.transpose() * Hm * constraintNullProjector;
    const int nullSpaceDim = Hm.rows() - Dm.rows();
    if (!HmProjected.isApprox(matrix_t::Identity(nullSpaceDim, nullSpaceDim), 1e-6)) {
      std::cerr << "HmProjected:\n" << HmProjected << "\n";
      throw std::runtime_error("HmProjected should be identity!");
    }
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

  // display
  if (ddpSettings_.displayInfo_) {
    std::string displayText = "Initial equality Constraints Penalty Parameters:\n";
    displayText += "    Penalty Tolerance: " + std::to_string(constraintPenaltyCoefficients_.penaltyTol);
    displayText += "    Penalty Coefficient: " + std::to_string(constraintPenaltyCoefficients_.penaltyCoeff) + ".\n";

    this->printString(displayText);
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
bool GaussNewtonDDP::initializePrimalSolution() {
  try {
    // clear before starting to fill
    nominalPrimalData_.clear();

    // for non-StateTriggeredRollout case, set modeSchedule
    nominalPrimalData_.primalSolution.modeSchedule_ = getReferenceManager().getModeSchedule();

    // try to initialize with controller
    bool initialSolutionExists = rolloutInitialController(optimizedPrimalSolution_, nominalPrimalData_.primalSolution);

    // if rolloutInitialController failed, try to initialize with PrimalSolution's state-input trajectories
    if (!initialSolutionExists) {
      // display
      if (ddpSettings_.displayInfo_) {
        std::cerr << "Initial controller is unavailable. Solver resorts to use PrimalSolution trajectories ...\n";
      }

      initialSolutionExists = extractInitialTrajectories(optimizedPrimalSolution_, nominalPrimalData_.primalSolution);
    }

    // display
    if (!initialSolutionExists && ddpSettings_.displayInfo_) {
      std::cerr << "Initial PrimalSolution trajectories are unavailable. Solver resorts to use Initializer ...\n";
    }

    // finish rollout with Initializer
    rolloutInitializer(nominalPrimalData_.primalSolution);

    // true if the rollout is not purely from the Initializer
    return initialSolutionExists;

  } catch (const std::exception& error) {
    const std::string msg = "[GaussNewtonDDP::initializePrimalSolution] the initial rollout is unstable!\n";
    throw std::runtime_error(msg + error.what());
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaussNewtonDDP::initializeDualSolutionAndMetrics() {
  // adjust dual solution
  totalDualSolutionTimer_.startTimer();
  if (!optimizedDualSolution_.timeTrajectory.empty()) {
    const auto status =
        trajectorySpread(optimizedPrimalSolution_.modeSchedule_, nominalPrimalData_.primalSolution.modeSchedule_, optimizedDualSolution_);
  }

  // initialize dual solution
  ocs2::initializeDualSolution(optimalControlProblemStock_[0], nominalPrimalData_.primalSolution, optimizedDualSolution_,
                               nominalDualData_.dualSolution);
  totalDualSolutionTimer_.endTimer();

  computeRolloutMetrics(optimalControlProblemStock_[0], nominalPrimalData_.primalSolution, nominalDualData_.dualSolution,
                        nominalPrimalData_.problemMetrics);

  // update dual
  //  totalDualSolutionTimer_.startTimer();
  //  ocs2::updateDualSolution(optimalControlProblemStock_[0], nominalPrimalData_.primalSolution, nominalPrimalData_.problemMetrics,
  //  nominalDualData_.dualSolution);
  //  totalDualSolutionTimer_.endTimer();

  // calculates rollout merit
  performanceIndex_ = computeRolloutPerformanceIndex(nominalPrimalData_.primalSolution.timeTrajectory_, nominalPrimalData_.problemMetrics);
  performanceIndex_.merit = calculateRolloutMerit(performanceIndex_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaussNewtonDDP::takePrimalDualStep(scalar_t lqModelExpectedCost) {
  // update primal: run search strategy and find the optimal stepLength
  searchStrategyTimer_.startTimer();
  scalar_t avgTimeStep;
  const auto& modeSchedule = this->getReferenceManager().getModeSchedule();
  search_strategy::SolutionRef solution(avgTimeStep, optimizedDualSolution_, optimizedPrimalSolution_, optimizedProblemMetrics_,
                                        performanceIndex_);
  const bool success = searchStrategyPtr_->run({initTime_, finalTime_}, initState_, lqModelExpectedCost, unoptimizedController_,
                                               nominalDualData_.dualSolution, modeSchedule, solution);

  if (success) {
    avgTimeStepFP_ = 0.9 * avgTimeStepFP_ + 0.1 * avgTimeStep;
  }
  searchStrategyTimer_.endTimer();

  // update dual
  totalDualSolutionTimer_.startTimer();
  if (success) {
    ocs2::updateDualSolution(optimalControlProblemStock_[0], optimizedPrimalSolution_, optimizedProblemMetrics_, optimizedDualSolution_);
    performanceIndex_ = computeRolloutPerformanceIndex(optimizedPrimalSolution_.timeTrajectory_, optimizedProblemMetrics_);
    performanceIndex_.merit = calculateRolloutMerit(performanceIndex_);
  }
  totalDualSolutionTimer_.endTimer();

  // if failed, use nominal and to keep the consistency of cached data, all cache should be left untouched
  if (!success) {
    optimizedDualSolution_ = nominalDualData_.dualSolution;
    optimizedPrimalSolution_ = nominalPrimalData_.primalSolution;
    optimizedProblemMetrics_ = nominalPrimalData_.problemMetrics;
    performanceIndex_ = performanceIndexHistory_.back();
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaussNewtonDDP::runImpl(scalar_t initTime, const vector_t& initState, scalar_t finalTime,
                             const ControllerBase* externalControllerPtr) {
  // Use the input controller if it is not empty otherwise use the internal controller. In the later case two scenarios are
  // possible: either the internal controller is already set (such as the MPC case where the warm starting option is set true)
  // or the internal controller is empty in which instead of performing a rollout the operating trajectories will be used.
  if (externalControllerPtr != nullptr) {
    // ensure initial controllers are of the right type, then assign
    const LinearController* linearControllerPtr = dynamic_cast<const LinearController*>(externalControllerPtr);
    if (linearControllerPtr == nullptr) {
      throw std::runtime_error("[GaussNewtonDDP::run] controller must be a LinearController type!");
    }
    optimizedPrimalSolution_.controllerPtr_.reset(linearControllerPtr->clone());
  }

  runImpl(initTime, initState, finalTime);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaussNewtonDDP::runImpl(scalar_t initTime, const vector_t& initState, scalar_t finalTime, const PrimalSolution& primalSolution) {
  // if PrimalSolution's controller exists, use it for initialization
  if (primalSolution.controllerPtr_ != nullptr && !primalSolution.controllerPtr_->empty()) {
    optimizedPrimalSolution_.modeSchedule_ = primalSolution.modeSchedule_;
    runImpl(initTime, initState, finalTime, primalSolution.controllerPtr_.get());

  } else {
    // otherwise initialize with PrimalSolution's state-input trajectories
    optimizedPrimalSolution_.controllerPtr_->clear();
    optimizedPrimalSolution_.modeSchedule_ = primalSolution.modeSchedule_;
    optimizedPrimalSolution_.timeTrajectory_ = primalSolution.timeTrajectory_;
    optimizedPrimalSolution_.postEventIndices_ = primalSolution.postEventIndices_;
    optimizedPrimalSolution_.stateTrajectory_ = primalSolution.stateTrajectory_;
    optimizedPrimalSolution_.inputTrajectory_ = primalSolution.inputTrajectory_;
    runImpl(initTime, initState, finalTime);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaussNewtonDDP::runImpl(scalar_t initTime, const vector_t& initState, scalar_t finalTime) {
  if (ddpSettings_.displayInfo_) {
    std::cerr << "\n++++++++++++++++++++++++++++++++++++++++++++++++++++++";
    std::cerr << "\n+++++++++++++ " + ddp::toAlgorithmName(ddpSettings_.algorithm_) + " solver is initialized ++++++++++++++";
    std::cerr << "\n++++++++++++++++++++++++++++++++++++++++++++++++++++++\n";
    std::cerr << "\nSolver starts from initial time " << initTime << " to final time " << finalTime << ".\n";
    std::cerr << getReferenceManager().getModeSchedule();
  }

  // set cost desired trajectories
  for (auto& ocp : optimalControlProblemStock_) {
    ocp.targetTrajectoriesPtr = &this->getReferenceManager().getTargetTrajectories();
  }

  // initialize parameters
  initTime_ = initTime;
  initState_ = initState;
  finalTime_ = finalTime;
  performanceIndexHistory_.clear();
  const auto initIteration = totalNumIterations_;
  initializeConstraintPenalties();  // initialize penalty coefficients

  // display
  if (ddpSettings_.displayInfo_) {
    std::cerr << "\n###################";
    std::cerr << "\n#### Initial Rollout";
    std::cerr << "\n###################\n";
  }

  // swap primal and dual data to cache
  nominalDualData_.swap(cachedDualData_);
  nominalPrimalData_.swap(cachedPrimalData_);

  // optimized --> nominal: initializes the nominal primal and dual solutions based on the optimized ones
  initializationTimer_.startTimer();
  bool initialSolutionExists = initializePrimalSolution();  // true if the rollout is not purely from the Initializer
  initializeDualSolutionAndMetrics();
  performanceIndexHistory_.push_back(performanceIndex_);
  initializationTimer_.endTimer();

  // display
  if (ddpSettings_.displayInfo_) {
    std::cerr << performanceIndex_ << '\n';
  }

  // convergence variables of the main loop
  bool isConverged = false;
  std::string convergenceInfo;

  // DDP main loop
  while (true) {
    if (ddpSettings_.displayInfo_) {
      std::cerr << "\n###################";
      std::cerr << "\n#### Iteration " << (totalNumIterations_ - initIteration);
      std::cerr << "\n###################\n";
    }

    // nominal --> nominal: constructs the LQ problem around the nominal trajectories
    linearQuadraticApproximationTimer_.startTimer();
    approximateOptimalControlProblem();
    linearQuadraticApproximationTimer_.endTimer();

    // nominal --> nominal: solves the LQ problem
    backwardPassTimer_.startTimer();
    avgTimeStepBP_ = solveSequentialRiccatiEquations(nominalPrimalData_.modelDataFinalTime.cost);
    backwardPassTimer_.endTimer();

    // calculate controller and store the result in unoptimizedController_
    computeControllerTimer_.startTimer();
    calculateController();
    computeControllerTimer_.endTimer();

    // the expected cost/merit calculated by the Riccati solution is not reliable
    const auto lqModelExpectedCost = initialSolutionExists ? nominalDualData_.valueFunctionTrajectory.front().f : performanceIndex_.merit;

    // nominal --> optimized: based on the current LQ solution updates the optimized primal and dual solutions
    takePrimalDualStep(lqModelExpectedCost);

    // iteration info
    ++totalNumIterations_;
    performanceIndexHistory_.push_back(performanceIndex_);

    // display
    if (ddpSettings_.displayInfo_) {
      printRolloutInfo();
    }

    // check convergence
    std::tie(isConverged, convergenceInfo) = searchStrategyPtr_->checkConvergence(
        !initialSolutionExists, *std::prev(performanceIndexHistory_.end(), 2), performanceIndexHistory_.back());
    initialSolutionExists = true;

    if (isConverged || (totalNumIterations_ - initIteration) == ddpSettings_.maxNumIterations_) {
      break;

    } else {
      // update the constraint penalty coefficients
      updateConstraintPenalties(performanceIndex_.equalityConstraintsSSE);

      // optimized --> nominal: use the optimized solution as the nominal for the next iteration
      nominalDualData_.swap(cachedDualData_);
      nominalPrimalData_.swap(cachedPrimalData_);
      optimizedDualSolution_.swap(nominalDualData_.dualSolution);
      optimizedPrimalSolution_.swap(nominalPrimalData_.primalSolution);
      optimizedProblemMetrics_.swap(nominalPrimalData_.problemMetrics);
    }
  }  // end of while loop

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
