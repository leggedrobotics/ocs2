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

#include "ocs2_ddp/ILQR.h"
#include <ocs2_ddp/riccati_equations/RiccatiTransversalityConditions.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ILQR::ILQR(ddp::Settings ddpSettings, const RolloutBase& rollout, const OptimalControlProblem& optimalControlProblem,
           const Initializer& initializer)
    : BASE(std::move(ddpSettings), rollout, optimalControlProblem, initializer) {
  if (settings().algorithm_ != ddp::Algorithm::ILQR) {
    throw std::runtime_error("In DDP setting the algorithm name is set \"" + ddp::toAlgorithmName(settings().algorithm_) +
                             "\" while ILQR is instantiated!");
  }

  // Riccati Solver
  riccatiEquationsPtrStock_.clear();
  riccatiEquationsPtrStock_.reserve(settings().nThreads_);

  for (size_t i = 0; i < settings().nThreads_; i++) {
    bool preComputeRiccatiTerms = settings().preComputeRiccatiTerms_ && (settings().strategy_ == search_strategy::Type::LINE_SEARCH);
    bool isRiskSensitive = !numerics::almost_eq(settings().riskSensitiveCoeff_, 0.0);
    riccatiEquationsPtrStock_.emplace_back(new DiscreteTimeRiccatiEquations(preComputeRiccatiTerms, isRiskSensitive));
    riccatiEquationsPtrStock_.back()->setRiskSensitiveCoefficient(settings().riskSensitiveCoeff_);
  }  // end of i loop

  Eigen::initParallel();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void ILQR::approximateIntermediateLQ(const scalar_array_t& timeTrajectory, const size_array_t& postEventIndices,
                                     const vector_array_t& stateTrajectory, const vector_array_t& inputTrajectory,
                                     std::vector<ModelData>& modelDataTrajectory) {
  BASE::nextTimeIndex_ = 0;
  BASE::nextTaskId_ = 0;
  std::function<void(void)> task = [&] {
    size_t timeIndex;
    size_t taskId = BASE::nextTaskId_++;  // assign task ID (atomic)

    ModelData continuousTimeModelData;

    // get next time index is atomic
    while ((timeIndex = BASE::nextTimeIndex_++) < timeTrajectory.size()) {
      // execute continuous time LQ approximation for the given partition and time index
      continuousTimeModelData = modelDataTrajectory[timeIndex];

      LinearQuadraticApproximator lqapprox(BASE::optimalControlProblemStock_[taskId], BASE::settings().checkNumericalStability_);
      lqapprox.approximateLQProblem(timeTrajectory[timeIndex], stateTrajectory[timeIndex], inputTrajectory[timeIndex],
                                    continuousTimeModelData);
      continuousTimeModelData.checkSizes(stateTrajectory[timeIndex].rows(), inputTrajectory[timeIndex].rows());

      // discretize LQ problem
      scalar_t timeStep = 0.0;
      if (timeIndex + 1 < timeTrajectory.size()) {
        timeStep = timeTrajectory[timeIndex + 1] - timeTrajectory[timeIndex];
      }

      if (!numerics::almost_eq(timeStep, 0.0)) {
        discreteLQWorker(taskId, timeStep, continuousTimeModelData, modelDataTrajectory[timeIndex]);
      } else {
        modelDataTrajectory[timeIndex] = continuousTimeModelData;
      }
    }
  };

  BASE::runParallel(task, settings().nThreads_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void ILQR::discreteLQWorker(size_t workerIndex, scalar_t timeStep, const ModelData& continuousTimeModelData, ModelData& modelData) {
  /*
   * linearize system dynamics
   */
  modelData.dynamics_.dfdx = matrix_t::Identity(continuousTimeModelData.stateDim_, continuousTimeModelData.stateDim_) +
                             continuousTimeModelData.dynamics_.dfdx * timeStep;
  modelData.dynamics_.dfdu = continuousTimeModelData.dynamics_.dfdu * timeStep;
  modelData.dynamics_.f.setZero(continuousTimeModelData.stateDim_);

  /*
   * quadratic approximation to the cost function
   */
  modelData.cost_.f = continuousTimeModelData.cost_.f * timeStep;
  modelData.cost_.dfdx = continuousTimeModelData.cost_.dfdx * timeStep;
  modelData.cost_.dfdxx = continuousTimeModelData.cost_.dfdxx * timeStep;
  modelData.cost_.dfdu = continuousTimeModelData.cost_.dfdu * timeStep;
  modelData.cost_.dfduu = continuousTimeModelData.cost_.dfduu * timeStep;
  modelData.cost_.dfdux = continuousTimeModelData.cost_.dfdux * timeStep;

  /*
   * linearize constraints
   */
  // state equality constraints
  modelData.stateEqConstr_ = continuousTimeModelData.stateEqConstr_;

  // state-input equality constraints
  modelData.stateInputEqConstr_ = continuousTimeModelData.stateInputEqConstr_;

  // inequality constraints
  modelData.ineqConstr_ = continuousTimeModelData.ineqConstr_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void ILQR::calculateController() {
  BASE::calculateController();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void ILQR::calculateControllerWorker(size_t workerIndex, size_t partitionIndex, size_t timeIndex) {
  const auto k = timeIndex;

  const auto& nominalState = BASE::nominalStateTrajectoriesStock_[k];
  const auto& nominalInput = BASE::nominalInputTrajectoriesStock_[k];

  const auto& EvProjected = BASE::projectedModelDataTrajectoriesStock_[k].stateInputEqConstr_.f;
  const auto& CmProjected = BASE::projectedModelDataTrajectoriesStock_[k].stateInputEqConstr_.dfdx;

  const auto& Qu = BASE::riccatiModificationTrajectoriesStock_[k].constraintNullProjector_;

  // feedback gains
  BASE::nominalControllersStock_.gainArray_[k] = -CmProjected;
  BASE::nominalControllersStock_.gainArray_[k].noalias() += Qu * projectedKmTrajectoryStock_[k];

  // bias input
  BASE::nominalControllersStock_.biasArray_[k] = nominalInput;
  // std::cerr << k << std::endl;
  BASE::nominalControllersStock_.biasArray_[k].noalias() -= BASE::nominalControllersStock_.gainArray_[k] * nominalState;
  BASE::nominalControllersStock_.deltaBiasArray_[k] = -EvProjected;
  BASE::nominalControllersStock_.deltaBiasArray_[k].noalias() += Qu * projectedLvTrajectoryStock_[k];

  // checking the numerical stability of the controller parameters
  if (settings().checkNumericalStability_) {
    try {
      if (!BASE::nominalControllersStock_.gainArray_[k].allFinite()) {
        throw std::runtime_error("Feedback gains are unstable.");
      }
      if (!BASE::nominalControllersStock_.deltaBiasArray_[k].allFinite()) {
        throw std::runtime_error("feedForwardControl is unstable.");
      }
    } catch (const std::exception& error) {
      std::cerr << "what(): " << error.what() << " at time " << BASE::nominalControllersStock_.timeStamp_[k] << " [sec]." << std::endl;
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
scalar_t ILQR::solveSequentialRiccatiEquations(const matrix_t& SmFinal, const vector_t& SvFinal, const scalar_t& sFinal) {
  auto N = BASE::nominalTimeTrajectoriesStock_.size();
  projectedLvTrajectoryStock_.resize(N);
  projectedKmTrajectoryStock_.resize(N);

  BASE::riccatiModificationTrajectoriesStock_.resize(N);
  BASE::projectedModelDataTrajectoriesStock_.resize(N);

  const auto& finalModelData = BASE::modelDataTrajectoriesStock_[N - 1];
  auto& finalRiccatiModification = BASE::riccatiModificationTrajectoriesStock_[N - 1];
  auto& finalProjectedModelData = BASE::projectedModelDataTrajectoriesStock_[N - 1];
  auto& finalProjectedLvFinal = projectedLvTrajectoryStock_[N - 1];
  auto& finalProjectedKmFinal = projectedKmTrajectoryStock_[N - 1];

  const auto SmDummy = matrix_t::Zero(finalModelData.stateDim_, finalModelData.stateDim_);
  BASE::computeProjectionAndRiccatiModification(finalModelData, SmDummy, finalProjectedModelData, finalRiccatiModification);

  // projected feedforward
  finalProjectedLvFinal = -finalProjectedModelData.cost_.dfdu - finalRiccatiModification.deltaGv_;
  finalProjectedLvFinal.noalias() -= finalProjectedModelData.dynamics_.dfdu.transpose() * SvFinal;

  // projected feedback
  finalProjectedKmFinal = -finalProjectedModelData.cost_.dfdux - finalRiccatiModification.deltaGm_;
  finalProjectedKmFinal.noalias() -= finalProjectedModelData.dynamics_.dfdu.transpose() * SmFinal;

  return BASE::solveSequentialRiccatiEquationsImpl(SmFinal, SvFinal, sFinal);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_t ILQR::computeHamiltonianHessian(const ModelData& modelData, const matrix_t& Sm) const {
  const matrix_t BmTransSm = modelData.dynamics_.dfdu.transpose() * Sm;
  matrix_t Hm = modelData.cost_.dfduu;
  Hm.noalias() += BmTransSm * modelData.dynamics_.dfdu;
  return searchStrategyPtr_->augmentHamiltonianHessian(modelData, Hm);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void ILQR::riccatiEquationsWorker(size_t workerIndex, const std::pair<int, int>& partitionInterval, const matrix_t& SmFinal,
                                  const vector_t& SvFinal, const scalar_t& sFinal) {
  const auto firstEventItr =
      std::upper_bound(BASE::nominalPostEventIndicesStock_.begin(), BASE::nominalPostEventIndicesStock_.end(), partitionInterval.first);
  const auto lastEventItr =
      std::upper_bound(BASE::nominalPostEventIndicesStock_.begin(), BASE::nominalPostEventIndicesStock_.end(), partitionInterval.second);

  // final temporal values
  matrix_t SmFinalTemp = SmFinal;
  vector_t SvFinalTemp = SvFinal;
  scalar_t sFinalTemp = sFinal;
  /*
   * solving the Riccati equations
   */
  const matrix_t* SmNext = &SmFinalTemp;
  const vector_t* SvNext = &SvFinalTemp;
  const scalar_t* sNext = &sFinalTemp;

  int curIndex = partitionInterval.second - 1;
  auto nextEventItr = lastEventItr - 1;
  const int stopIndex = partitionInterval.first;
  while (curIndex >= stopIndex) {
    auto& curProjectedLv = projectedLvTrajectoryStock_[curIndex];
    auto& curProjectedKm = projectedKmTrajectoryStock_[curIndex];
    auto& curProjectedModelData = BASE::projectedModelDataTrajectoriesStock_[curIndex];
    auto& curRiccatiModification = BASE::riccatiModificationTrajectoriesStock_[curIndex];
    auto& curModelData = BASE::modelDataTrajectoriesStock_[curIndex];

    auto& curSm = BASE::SmTrajectoryStock_[curIndex];
    auto& curSv = BASE::SvTrajectoryStock_[curIndex];
    auto& curs = BASE::sTrajectoryStock_[curIndex];

    BASE::SsTimeTrajectoryStock_[curIndex] = BASE::nominalTimeTrajectoriesStock_[curIndex];
    BASE::computeProjectionAndRiccatiModification(curModelData, *SmNext, curProjectedModelData, curRiccatiModification);

    riccatiEquationsPtrStock_[workerIndex]->computeMap(curProjectedModelData, curRiccatiModification, *SmNext, *SvNext, *sNext,
                                                       curProjectedKm, curProjectedLv, curSm, curSv, curs);
    SmNext = &curSm;
    SvNext = &curSv;
    sNext = &curs;

    if (std::distance(firstEventItr, nextEventItr) >= 0 && curIndex == *nextEventItr) {
      // move to pre-event index
      --curIndex;

      const int index = std::distance(BASE::nominalPostEventIndicesStock_.begin(), nextEventItr);
      std::tie(SmFinalTemp, SvFinalTemp, sFinalTemp) =
          riccatiTransversalityConditions(BASE::modelDataEventTimesStock_[index], curSm, curSv, curs);

      BASE::sTrajectoryStock_[curIndex] = sFinalTemp;
      BASE::SvTrajectoryStock_[curIndex] = SvFinalTemp;
      BASE::SmTrajectoryStock_[curIndex] = SmFinalTemp;

      // continuous-time for final step
      const auto& finalModelData = BASE::modelDataTrajectoriesStock_[curIndex];
      auto& finalRiccatiModification = BASE::riccatiModificationTrajectoriesStock_[curIndex];
      auto& finalProjectedModelData = BASE::projectedModelDataTrajectoriesStock_[curIndex];
      auto& finalProjectedLvFinal = projectedLvTrajectoryStock_[curIndex];
      auto& finalProjectedKmFinal = projectedKmTrajectoryStock_[curIndex];

      BASE::SsTimeTrajectoryStock_[curIndex] = BASE::nominalTimeTrajectoriesStock_[curIndex];
      const auto SmDummy = matrix_t::Zero(finalModelData.stateDim_, finalModelData.stateDim_);
      BASE::computeProjectionAndRiccatiModification(finalModelData, SmDummy, finalProjectedModelData, finalRiccatiModification);

      // projected feedforward
      finalProjectedLvFinal = -finalProjectedModelData.cost_.dfdu - finalRiccatiModification.deltaGv_;
      finalProjectedLvFinal.noalias() -= finalProjectedModelData.dynamics_.dfdu.transpose() * BASE::SvTrajectoryStock_[curIndex];

      // projected feedback
      finalProjectedKmFinal = -finalProjectedModelData.cost_.dfdux - finalRiccatiModification.deltaGm_;
      finalProjectedKmFinal.noalias() -= finalProjectedModelData.dynamics_.dfdu.transpose() * BASE::SmTrajectoryStock_[curIndex];

      SmNext = &SmFinalTemp;
      SvNext = &SvFinalTemp;
      sNext = &sFinalTemp;

      --nextEventItr;
    }

    --curIndex;
  }  // while
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void ILQR::setupOptimizer(size_t numPartitions) {}

}  // namespace ocs2
