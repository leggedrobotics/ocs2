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
void ILQR::approximateIntermediateLQ(PrimalDataContainer& primalData) {
  // create alias
  const auto& timeTrajectory = primalData.primalSolution.timeTrajectory_;
  const auto& postEventIndices = primalData.postEventIndices;
  const auto& stateTrajectory = primalData.primalSolution.stateTrajectory_;
  const auto& inputTrajectory = primalData.primalSolution.inputTrajectory_;
  auto& modelDataTrajectory = primalData.modelDataTrajectory;

  nextTimeIndex_ = 0;
  nextTaskId_ = 0;
  auto task = [&]() {
    size_t timeIndex;
    size_t taskId = nextTaskId_++;  // assign task ID (atomic)

    ModelData continuousTimeModelData;

    // get next time index is atomic
    while ((timeIndex = nextTimeIndex_++) < timeTrajectory.size()) {
      // execute continuous time LQ approximation for the given partition and time index
      continuousTimeModelData = modelDataTrajectory[timeIndex];

      LinearQuadraticApproximator lqapprox(optimalControlProblemStock_[taskId], settings().checkNumericalStability_);
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

  runParallel(task, settings().nThreads_);
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
void ILQR::calculateControllerWorker(size_t timeIndex, const PrimalDataContainer& primalData, const DualDataContainer& dualData,
                                     LinearController& dstController) {
  const auto& nominalState = primalData.primalSolution.stateTrajectory_[timeIndex];
  const auto& nominalInput = primalData.primalSolution.inputTrajectory_[timeIndex];

  const auto& EvProjected = dualData.projectedModelDataTrajectory[timeIndex].stateInputEqConstr_.f;
  const auto& CmProjected = dualData.projectedModelDataTrajectory[timeIndex].stateInputEqConstr_.dfdx;

  const auto& Qu = dualData.riccatiModificationTrajectory[timeIndex].constraintNullProjector_;

  // feedback gains
  dstController.gainArray_[timeIndex] = -CmProjected;
  dstController.gainArray_[timeIndex].noalias() += Qu * projectedKmTrajectoryStock_[timeIndex];

  // bias input
  dstController.biasArray_[timeIndex] = nominalInput;
  dstController.biasArray_[timeIndex].noalias() -= dstController.gainArray_[timeIndex] * nominalState;
  dstController.deltaBiasArray_[timeIndex] = -EvProjected;
  dstController.deltaBiasArray_[timeIndex].noalias() += Qu * projectedLvTrajectoryStock_[timeIndex];
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
scalar_t ILQR::solveSequentialRiccatiEquations(const ScalarFunctionQuadraticApproximation& finalValueFunction) {
  const size_t N = nominalPrimalData_.primalSolution.timeTrajectory_.size();
  projectedLvTrajectoryStock_.resize(N);
  projectedKmTrajectoryStock_.resize(N);

  dualData_.riccatiModificationTrajectory.resize(N);
  dualData_.projectedModelDataTrajectory.resize(N);

  const auto& finalModelData = nominalPrimalData_.modelDataTrajectory.back();
  auto& finalRiccatiModification = dualData_.riccatiModificationTrajectory.back();
  auto& finalProjectedModelData = dualData_.projectedModelDataTrajectory.back();
  auto& finalProjectedLvFinal = projectedLvTrajectoryStock_.back();
  auto& finalProjectedKmFinal = projectedKmTrajectoryStock_.back();

  const matrix_t SmDummy = matrix_t::Zero(finalModelData.stateDim_, finalModelData.stateDim_);
  computeProjectionAndRiccatiModification(finalModelData, SmDummy, finalProjectedModelData, finalRiccatiModification);

  // projected feedforward
  finalProjectedLvFinal = -finalProjectedModelData.cost_.dfdu - finalRiccatiModification.deltaGv_;
  finalProjectedLvFinal.noalias() -= finalProjectedModelData.dynamics_.dfdu.transpose() * finalValueFunction.dfdx;

  // projected feedback
  finalProjectedKmFinal = -finalProjectedModelData.cost_.dfdux - finalRiccatiModification.deltaGm_;
  finalProjectedKmFinal.noalias() -= finalProjectedModelData.dynamics_.dfdu.transpose() * finalValueFunction.dfdxx;

  return solveSequentialRiccatiEquationsImpl(finalValueFunction);
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
void ILQR::riccatiEquationsWorker(size_t workerIndex, const std::pair<int, int>& partitionInterval,
                                  const ScalarFunctionQuadraticApproximation& finalValueFunction) {
  // find all events belonging to the current partition
  const auto& postEventIndices = nominalPrimalData_.postEventIndices;
  const auto firstEventItr = std::upper_bound(postEventIndices.begin(), postEventIndices.end(), partitionInterval.first);
  const auto lastEventItr = std::upper_bound(postEventIndices.begin(), postEventIndices.end(), partitionInterval.second);

  // final temporal values. Used to store pre-jump value
  ScalarFunctionQuadraticApproximation finalValueTemp = finalValueFunction;

  /*
   * solving the Riccati equations
   */
  const ScalarFunctionQuadraticApproximation* valueFunctionNext = &finalValueTemp;

  int curIndex = partitionInterval.second - 1;
  auto nextEventItr = lastEventItr - 1;
  const int stopIndex = partitionInterval.first;
  while (curIndex >= stopIndex) {
    auto& curProjectedLv = projectedLvTrajectoryStock_[curIndex];
    auto& curProjectedKm = projectedKmTrajectoryStock_[curIndex];
    auto& curProjectedModelData = dualData_.projectedModelDataTrajectory[curIndex];
    auto& curRiccatiModification = dualData_.riccatiModificationTrajectory[curIndex];
    const auto& curModelData = nominalPrimalData_.modelDataTrajectory[curIndex];

    auto& curSm = dualData_.valueFunctionTrajectory[curIndex].dfdxx;
    auto& curSv = dualData_.valueFunctionTrajectory[curIndex].dfdx;
    auto& curs = dualData_.valueFunctionTrajectory[curIndex].f;

    computeProjectionAndRiccatiModification(curModelData, valueFunctionNext->dfdxx, curProjectedModelData, curRiccatiModification);

    riccatiEquationsPtrStock_[workerIndex]->computeMap(curProjectedModelData, curRiccatiModification, valueFunctionNext->dfdxx,
                                                       valueFunctionNext->dfdx, valueFunctionNext->f, curProjectedKm, curProjectedLv, curSm,
                                                       curSv, curs);
    valueFunctionNext = &(dualData_.valueFunctionTrajectory[curIndex]);

    if (std::distance(firstEventItr, nextEventItr) >= 0 && curIndex == *nextEventItr) {
      // move to pre-event index
      --curIndex;

      const int index = std::distance(postEventIndices.begin(), nextEventItr);
      std::tie(finalValueTemp.dfdxx, finalValueTemp.dfdx, finalValueTemp.f) =
          riccatiTransversalityConditions(nominalPrimalData_.modelDataEventTimes[index], curSm, curSv, curs);

      dualData_.valueFunctionTrajectory[curIndex] = finalValueTemp;

      const auto& finalModelData = nominalPrimalData_.modelDataTrajectory[curIndex];
      auto& finalRiccatiModification = dualData_.riccatiModificationTrajectory[curIndex];
      auto& finalProjectedModelData = dualData_.projectedModelDataTrajectory[curIndex];
      auto& finalProjectedLvFinal = projectedLvTrajectoryStock_[curIndex];
      auto& finalProjectedKmFinal = projectedKmTrajectoryStock_[curIndex];

      const matrix_t SmDummy = matrix_t::Zero(finalModelData.stateDim_, finalModelData.stateDim_);
      computeProjectionAndRiccatiModification(finalModelData, SmDummy, finalProjectedModelData, finalRiccatiModification);

      // projected feedforward
      finalProjectedLvFinal = -finalProjectedModelData.cost_.dfdu - finalRiccatiModification.deltaGv_;
      finalProjectedLvFinal.noalias() -=
          finalProjectedModelData.dynamics_.dfdu.transpose() * dualData_.valueFunctionTrajectory[curIndex].dfdx;

      // projected feedback
      finalProjectedKmFinal = -finalProjectedModelData.cost_.dfdux - finalRiccatiModification.deltaGm_;
      finalProjectedKmFinal.noalias() -=
          finalProjectedModelData.dynamics_.dfdu.transpose() * dualData_.valueFunctionTrajectory[curIndex].dfdxx;

      valueFunctionNext = &finalValueTemp;

      --nextEventItr;
    }

    --curIndex;
  }  // while
}
}  // namespace ocs2
