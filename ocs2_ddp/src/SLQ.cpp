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

#include "ocs2_ddp/SLQ.h"

#include "ocs2_ddp/DDP_HelperFunctions.h"
#include "ocs2_ddp/riccati_equations/RiccatiModificationInterpolation.h"

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SLQ::SLQ(ddp::Settings ddpSettings, const RolloutBase& rollout, const OptimalControlProblem& optimalControlProblem,
         const Initializer& initializer)
    : GaussNewtonDDP(std::move(ddpSettings), rollout, optimalControlProblem, initializer) {
  if (settings().algorithm_ != ddp::Algorithm::SLQ) {
    throw std::runtime_error("[SLQ] In DDP setting the algorithm name is set \"" + ddp::toAlgorithmName(settings().algorithm_) +
                             "\" while SLQ is instantiated!");
  }

  allSsTrajectoryStock_.resize(settings().nThreads_);
  SsNormalizedTimeTrajectoryStock_.resize(settings().nThreads_);
  SsNormalizedEventsPastTheEndIndecesStock_.resize(settings().nThreads_);
  // Riccati Solver
  riccatiEquationsPtrStock_.clear();
  riccatiEquationsPtrStock_.reserve(settings().nThreads_);
  riccatiIntegratorPtrStock_.clear();
  riccatiIntegratorPtrStock_.reserve(settings().nThreads_);

  const auto integratorType = settings().backwardPassIntegratorType_;
  if (integratorType != IntegratorType::ODE45 && integratorType != IntegratorType::BULIRSCH_STOER &&
      integratorType != IntegratorType::ODE45_OCS2 && integratorType != IntegratorType::RK4) {
    throw(std::runtime_error("Unsupported Riccati equation integrator type: " +
                             integrator_type::toString(settings().backwardPassIntegratorType_)));
  }

  for (size_t i = 0; i < settings().nThreads_; i++) {
    bool preComputeRiccatiTerms = settings().preComputeRiccatiTerms_ && (settings().strategy_ == search_strategy::Type::LINE_SEARCH);
    bool isRiskSensitive = !numerics::almost_eq(settings().riskSensitiveCoeff_, 0.0);
    riccatiEquationsPtrStock_.emplace_back(new ContinuousTimeRiccatiEquations(preComputeRiccatiTerms, isRiskSensitive));
    riccatiEquationsPtrStock_.back()->setRiskSensitiveCoefficient(settings().riskSensitiveCoeff_);
    riccatiIntegratorPtrStock_.emplace_back(newIntegrator(integratorType));
  }  // end of i loop

  Eigen::initParallel();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SLQ::approximateIntermediateLQ(const DualSolution& dualSolution, PrimalDataContainer& primalData) {
  // create alias
  const auto& timeTrajectory = primalData.primalSolution.timeTrajectory_;
  const auto& stateTrajectory = primalData.primalSolution.stateTrajectory_;
  const auto& inputTrajectory = primalData.primalSolution.inputTrajectory_;
  const auto& postEventIndices = primalData.primalSolution.postEventIndices_;
  const auto& multiplierTrajectory = dualSolution.intermediates;
  auto& modelDataTrajectory = primalData.modelDataTrajectory;

  modelDataTrajectory.clear();
  modelDataTrajectory.resize(timeTrajectory.size());

  nextTimeIndex_ = 0;
  nextTaskId_ = 0;
  auto task = [&]() {
    const size_t taskId = nextTaskId_++;  // assign task ID (atomic)

    // get next time index is atomic
    size_t timeIndex;
    while ((timeIndex = nextTimeIndex_++) < timeTrajectory.size()) {
      // approximate LQ for the given time index
      ocs2::approximateIntermediateLQ(optimalControlProblemStock_[taskId], timeTrajectory[timeIndex], stateTrajectory[timeIndex],
                                      inputTrajectory[timeIndex], multiplierTrajectory[timeIndex], modelDataTrajectory[timeIndex]);

      // checking the numerical properties
      if (settings().checkNumericalStability_) {
        const auto errSize =
            checkSize(modelDataTrajectory[timeIndex], stateTrajectory[timeIndex].rows(), inputTrajectory[timeIndex].rows());
        if (!errSize.empty()) {
          throw std::runtime_error("[SLQ::approximateIntermediateLQ] Mismatch in dimensions at intermediate time: " +
                                   std::to_string(timeTrajectory[timeIndex]) + "\n" + errSize);
        }
        const std::string errProperties = checkDynamicsProperties(modelDataTrajectory[timeIndex]) +
                                          checkCostProperties(modelDataTrajectory[timeIndex]) +
                                          checkConstraintProperties(modelDataTrajectory[timeIndex]);
        if (!errProperties.empty()) {
          throw std::runtime_error("[SLQ::approximateIntermediateLQ] Ill-posed problem at intermediate time: " +
                                   std::to_string(timeTrajectory[timeIndex]) + "\n" + errProperties);
        }
      }
    }  // end of while loop
  };

  runParallel(task, settings().nThreads_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SLQ::calculateControllerWorker(size_t timeIndex, const PrimalDataContainer& primalData, const DualDataContainer& dualData,
                                    LinearController& dstController) {
  const auto time = primalData.primalSolution.timeTrajectory_[timeIndex];

  const vector_t& nominalState = primalData.primalSolution.stateTrajectory_[timeIndex];
  const vector_t& nominalInput = primalData.primalSolution.inputTrajectory_[timeIndex];

  // BmProjected
  const matrix_t& projectedBm = dualData.projectedModelDataTrajectory[timeIndex].dynamics.dfdu;

  // PmProjected
  const matrix_t& projectedPm = dualData.projectedModelDataTrajectory[timeIndex].cost.dfdux;
  // RvProjected
  const vector_t& projectedRv = dualData.projectedModelDataTrajectory[timeIndex].cost.dfdu;
  // EvProjected
  const vector_t& EvProjected = dualData.projectedModelDataTrajectory[timeIndex].stateInputEqConstraint.f;
  // CmProjected
  const matrix_t& CmProjected = dualData.projectedModelDataTrajectory[timeIndex].stateInputEqConstraint.dfdx;
  // projector
  const matrix_t& Qu = dualData.riccatiModificationTrajectory[timeIndex].constraintNullProjector_;
  // deltaGm, projected feedback
  matrix_t projectedKm = dualData.riccatiModificationTrajectory[timeIndex].deltaGm_;
  // deltaGv, projected feedforward
  vector_t projectedLv = dualData.riccatiModificationTrajectory[timeIndex].deltaGv_;

  // projectedKm = projectedPm + projectedBm^t * Sm
  projectedKm = -(projectedKm + projectedPm);
  projectedKm.noalias() -= projectedBm.transpose() * dualData.valueFunctionTrajectory[timeIndex].dfdxx;

  // projectedLv = projectedRv + projectedBm^t * Sv
  projectedLv = -(projectedLv + projectedRv);
  projectedLv.noalias() -= projectedBm.transpose() * dualData.valueFunctionTrajectory[timeIndex].dfdx;

  // feedback gains
  dstController.gainArray_[timeIndex] = -CmProjected;
  dstController.gainArray_[timeIndex].noalias() += Qu * projectedKm;

  // bias input
  dstController.biasArray_[timeIndex] = nominalInput;
  dstController.biasArray_[timeIndex].noalias() -= dstController.gainArray_[timeIndex] * nominalState;
  dstController.deltaBiasArray_[timeIndex] = -EvProjected;
  dstController.deltaBiasArray_[timeIndex].noalias() += Qu * projectedLv;
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
scalar_t SLQ::solveSequentialRiccatiEquations(const ScalarFunctionQuadraticApproximation& finalValueFunction) {
  // fully compute the riccatiModifications and projected modelData
  // number of the intermediate LQ variables
  const size_t N = nominalPrimalData_.primalSolution.timeTrajectory_.size();

  nominalDualData_.riccatiModificationTrajectory.resize(N);
  nominalDualData_.projectedModelDataTrajectory.resize(N);

  if (N > 0) {
    // perform the computeRiccatiModificationTerms for partition i
    nextTimeIndex_ = 0;
    nextTaskId_ = 0;
    auto task = [this, N]() {
      int timeIndex;
      const matrix_t SmDummy = matrix_t::Zero(0, 0);

      // get next time index is atomic
      while ((timeIndex = nextTimeIndex_++) < N) {
        computeProjectionAndRiccatiModification(nominalPrimalData_.modelDataTrajectory[timeIndex], SmDummy,
                                                nominalDualData_.projectedModelDataTrajectory[timeIndex],
                                                nominalDualData_.riccatiModificationTrajectory[timeIndex]);
      }
    };
    runParallel(task, settings().nThreads_);
  }

  return solveSequentialRiccatiEquationsImpl(finalValueFunction);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_t SLQ::computeHamiltonianHessian(const ModelData& modelData, const matrix_t& Sm) const {
  return searchStrategyPtr_->augmentHamiltonianHessian(modelData, modelData.cost.dfduu);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SLQ::riccatiEquationsWorker(size_t workerIndex, const std::pair<int, int>& partitionInterval,
                                 const ScalarFunctionQuadraticApproximation& finalValueFunction) {
  // set data for Riccati equations
  riccatiEquationsPtrStock_[workerIndex]->resetNumFunctionCalls();
  riccatiEquationsPtrStock_[workerIndex]->setData(
      &(nominalPrimalData_.primalSolution.timeTrajectory_), &(nominalDualData_.projectedModelDataTrajectory),
      &(nominalPrimalData_.primalSolution.postEventIndices_), &(nominalPrimalData_.modelDataEventTimes),
      &(nominalDualData_.riccatiModificationTrajectory));

  const auto& nominalTimeTrajectory = nominalPrimalData_.primalSolution.timeTrajectory_;
  const auto& nominalEventsPastTheEndIndices = nominalPrimalData_.primalSolution.postEventIndices_;

  auto& valueFunctionTrajectory = nominalDualData_.valueFunctionTrajectory;

  // Convert final value of value function in vector format
  vector_t allSsFinal = ContinuousTimeRiccatiEquations::convert2Vector(finalValueFunction);

  scalar_array_t& SsNormalizedTime = SsNormalizedTimeTrajectoryStock_[workerIndex];
  SsNormalizedTime.clear();
  size_array_t& SsNormalizedPostEventIndices = SsNormalizedEventsPastTheEndIndecesStock_[workerIndex];
  SsNormalizedPostEventIndices.clear();

  /*
   *  The riccati equations are solved backwards in time
   *  the SsNormalized time is therefore filled with negative time in the reverse order, for example:
   *  nominalTime = [0.0, 1.0, 2.0, ..., 10.0]
   *  SsNormalized = [-10.0, ..., -2.0, -1.0, -0.0]
   */
  vector_array_t& allSsTrajectory = allSsTrajectoryStock_[workerIndex];
  integrateRiccatiEquationNominalTime(*riccatiIntegratorPtrStock_[workerIndex], *riccatiEquationsPtrStock_[workerIndex], partitionInterval,
                                      nominalTimeTrajectory, nominalEventsPastTheEndIndices, std::move(allSsFinal), SsNormalizedTime,
                                      SsNormalizedPostEventIndices, allSsTrajectory);

  // Convert value function to matrix format
  size_t outputN = SsNormalizedTime.size();
  for (size_t k = partitionInterval.first; k < partitionInterval.second; k++) {
    ContinuousTimeRiccatiEquations::convert2Matrix(allSsTrajectory[outputN - 1 - k + partitionInterval.first], valueFunctionTrajectory[k]);
  }  // end of k loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SLQ::integrateRiccatiEquationNominalTime(IntegratorBase& riccatiIntegrator, ContinuousTimeRiccatiEquations& riccatiEquation,
                                              const std::pair<int, int>& partitionInterval, const scalar_array_t& nominalTimeTrajectory,
                                              const size_array_t& nominalEventsPastTheEndIndices, vector_t allSsFinal,
                                              scalar_array_t& SsNormalizedTime, size_array_t& SsNormalizedPostEventIndices,
                                              vector_array_t& allSsTrajectory) {
  // normalized time and post event indices
  retrieveActiveNormalizedTime(partitionInterval, nominalTimeTrajectory, nominalEventsPastTheEndIndices, SsNormalizedTime,
                               SsNormalizedPostEventIndices);
  // Extract sizes
  const int nominalTimeSize = SsNormalizedTime.size();
  const int numEvents = SsNormalizedPostEventIndices.size();
  auto partitionDuration = nominalTimeTrajectory[partitionInterval.second] - nominalTimeTrajectory[partitionInterval.first];

  // Normalized switching time indices, start and end of the partition are added at the beginning and end
  using iterator_t = scalar_array_t::const_iterator;
  std::vector<std::pair<iterator_t, iterator_t>> SsNormalizedSwitchingTimesIndices;
  SsNormalizedSwitchingTimesIndices.reserve(numEvents + 1);
  SsNormalizedSwitchingTimesIndices.emplace_back(SsNormalizedTime.cbegin(), SsNormalizedTime.cbegin());
  for (const auto& index : SsNormalizedPostEventIndices) {
    SsNormalizedSwitchingTimesIndices.back().second = SsNormalizedTime.cbegin() + index;
    SsNormalizedSwitchingTimesIndices.emplace_back(SsNormalizedTime.cbegin() + index, SsNormalizedTime.cbegin() + index);
  }
  SsNormalizedSwitchingTimesIndices.back().second = SsNormalizedTime.cend();

  // integrating the Riccati equations
  allSsTrajectory.clear();
  allSsTrajectory.reserve(nominalTimeSize);
  for (int i = 0; i <= numEvents; i++) {
    iterator_t beginTimeItr = SsNormalizedSwitchingTimesIndices[i].first;
    iterator_t endTimeItr = SsNormalizedSwitchingTimesIndices[i].second;

    // solve Riccati equations
    Observer observer(&allSsTrajectory);
    const auto maxNumTimeSteps = static_cast<size_t>(settings().maxNumStepsPerSecond_ * std::max(1.0, partitionDuration));
    riccatiIntegrator.integrateTimes(riccatiEquation, observer, allSsFinal, beginTimeItr, endTimeItr, settings().timeStep_,
                                     settings().absTolODE_, settings().relTolODE_, maxNumTimeSteps);

    if (i < numEvents) {
      allSsFinal = riccatiEquation.computeJumpMap(*endTimeItr, allSsTrajectory.back());
    }
  }  // end of i loop

  // check size
  if (allSsTrajectory.size() != nominalTimeSize) {
    throw std::runtime_error("[SLQ::integrateRiccatiEquationNominalTim] allSsTrajectory size is incorrect.");
  }
}

}  // namespace ocs2
