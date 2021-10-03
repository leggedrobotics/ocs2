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

  // correcting for the last controller element of partitions
  for (size_t i = BASE::initActivePartition_; i < BASE::finalActivePartition_; i++) {
    BASE::nominalControllersStock_[i].gainArray_.back() = BASE::nominalControllersStock_[i + 1].gainArray_.front();
    BASE::nominalControllersStock_[i].biasArray_.back() = BASE::nominalControllersStock_[i + 1].biasArray_.front();
    BASE::nominalControllersStock_[i].deltaBiasArray_.back() = BASE::nominalControllersStock_[i + 1].deltaBiasArray_.front();
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void ILQR::calculateControllerWorker(size_t workerIndex, size_t partitionIndex, size_t timeIndex) {
  const auto i = partitionIndex;
  const auto k = timeIndex;

  const auto& nominalState = BASE::nominalStateTrajectoriesStock_[i][k];
  const auto& nominalInput = BASE::nominalInputTrajectoriesStock_[i][k];

  const auto& EvProjected = BASE::projectedModelDataTrajectoriesStock_[i][k].stateInputEqConstr_.f;
  const auto& CmProjected = BASE::projectedModelDataTrajectoriesStock_[i][k].stateInputEqConstr_.dfdx;

  const auto& Qu = BASE::riccatiModificationTrajectoriesStock_[i][k].constraintNullProjector_;

  // feedback gains
  BASE::nominalControllersStock_[i].gainArray_[k] = -CmProjected;
  BASE::nominalControllersStock_[i].gainArray_[k].noalias() += Qu * projectedKmTrajectoryStock_[i][k];

  // bias input
  BASE::nominalControllersStock_[i].biasArray_[k] = nominalInput;
  BASE::nominalControllersStock_[i].biasArray_[k].noalias() -= BASE::nominalControllersStock_[i].gainArray_[k] * nominalState;
  BASE::nominalControllersStock_[i].deltaBiasArray_[k] = -EvProjected;
  BASE::nominalControllersStock_[i].deltaBiasArray_[k].noalias() += Qu * projectedLvTrajectoryStock_[i][k];

  // checking the numerical stability of the controller parameters
  if (settings().checkNumericalStability_) {
    try {
      if (!BASE::nominalControllersStock_[i].gainArray_[k].allFinite()) {
        throw std::runtime_error("Feedback gains are unstable.");
      }
      if (!BASE::nominalControllersStock_[i].deltaBiasArray_[k].allFinite()) {
        throw std::runtime_error("feedForwardControl is unstable.");
      }
    } catch (const std::exception& error) {
      std::cerr << "what(): " << error.what() << " at time " << BASE::nominalControllersStock_[i].timeStamp_[k] << " [sec]." << std::endl;
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
scalar_t ILQR::solveSequentialRiccatiEquations(const matrix_t& SmFinal, const vector_t& SvFinal, const scalar_t& sFinal) {
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
void ILQR::riccatiEquationsWorker(size_t workerIndex, size_t partitionIndex, const matrix_t& SmFinal, const vector_t& SvFinal,
                                  const scalar_t& sFinal) {
  const int N = BASE::nominalTimeTrajectoriesStock_[partitionIndex].size();
  const int NE = BASE::nominalPostEventIndicesStock_[partitionIndex].size();

  // normalized time and post event indices
  BASE::computeNormalizedTime(BASE::nominalTimeTrajectoriesStock_[partitionIndex], BASE::nominalPostEventIndicesStock_[partitionIndex],
                              BASE::SsNormalizedTimeTrajectoryStock_[partitionIndex],
                              BASE::SsNormalizedEventsPastTheEndIndecesStock_[partitionIndex]);

  // output containers resizing
  BASE::SsTimeTrajectoryStock_[partitionIndex] = BASE::nominalTimeTrajectoriesStock_[partitionIndex];
  BASE::sTrajectoryStock_[partitionIndex].resize(N);
  BASE::SvTrajectoryStock_[partitionIndex].resize(N);
  BASE::SmTrajectoryStock_[partitionIndex].resize(N);

  projectedLvTrajectoryStock_[partitionIndex].resize(N);
  projectedKmTrajectoryStock_[partitionIndex].resize(N);

  BASE::riccatiModificationTrajectoriesStock_[partitionIndex].resize(N);
  BASE::projectedModelDataTrajectoriesStock_[partitionIndex].resize(N);

  // terminate if the partition is not active
  if (N == 0) {
    return;
  }

  // switching times
  size_array_t SsSwitchingTimesIndices;
  SsSwitchingTimesIndices.reserve(NE + 2);
  SsSwitchingTimesIndices.push_back(0);
  for (int k = 0; k < NE; k++) {
    SsSwitchingTimesIndices.push_back(BASE::nominalPostEventIndicesStock_[partitionIndex][k]);
  }
  SsSwitchingTimesIndices.push_back(N);

  // final temporal values
  matrix_t SmFinalTemp = SmFinal;
  vector_t SvFinalTemp = SvFinal;
  scalar_t sFinalTemp = sFinal;

  /*
   * solving the Riccati equations
   */
  for (int i = NE; i >= 0; i--) {
    int beginTimeItr = SsSwitchingTimesIndices[i];    // similar to std::begin()
    int endTimeItr = SsSwitchingTimesIndices[i + 1];  // similar to std::end()

    /*
     * solve Riccati equations at final time
     */
    BASE::sTrajectoryStock_[partitionIndex][endTimeItr - 1] = sFinal;
    BASE::SvTrajectoryStock_[partitionIndex][endTimeItr - 1] = SvFinal;
    BASE::SmTrajectoryStock_[partitionIndex][endTimeItr - 1] = SmFinal;

    // continuous-time for final step
    const auto& modelDataFinal = BASE::modelDataTrajectoriesStock_[partitionIndex][endTimeItr - 1];
    auto& riccatiModificationFinal = BASE::riccatiModificationTrajectoriesStock_[partitionIndex][endTimeItr - 1];
    auto& projectedModelDataFinal = BASE::projectedModelDataTrajectoriesStock_[partitionIndex][endTimeItr - 1];
    auto& projectedLvFinal = projectedLvTrajectoryStock_[partitionIndex][endTimeItr - 1];
    auto& projectedKmFinal = projectedKmTrajectoryStock_[partitionIndex][endTimeItr - 1];

    const auto SmDummy = matrix_t::Zero(modelDataFinal.stateDim_, modelDataFinal.stateDim_);
    BASE::computeProjectionAndRiccatiModification(modelDataFinal, SmDummy, projectedModelDataFinal, riccatiModificationFinal);

    // projected feedforward
    projectedLvFinal = -projectedModelDataFinal.cost_.dfdu - riccatiModificationFinal.deltaGv_;
    projectedLvFinal.noalias() -=
        projectedModelDataFinal.dynamics_.dfdu.transpose() * BASE::SvTrajectoryStock_[partitionIndex][endTimeItr - 1];

    // projected feedback
    projectedKmFinal = -projectedModelDataFinal.cost_.dfdux - riccatiModificationFinal.deltaGm_;
    projectedKmFinal.noalias() -=
        projectedModelDataFinal.dynamics_.dfdu.transpose() * BASE::SmTrajectoryStock_[partitionIndex][endTimeItr - 1];

    /*
     * solve Riccati equations and compute projected model data and RiccatiModification for the intermediate times
     */
    for (int k = endTimeItr - 2; k >= beginTimeItr; k--) {
      // project
      BASE::computeProjectionAndRiccatiModification(
          BASE::modelDataTrajectoriesStock_[partitionIndex][k], BASE::SmTrajectoryStock_[partitionIndex][k + 1],
          BASE::projectedModelDataTrajectoriesStock_[partitionIndex][k], BASE::riccatiModificationTrajectoriesStock_[partitionIndex][k]);

      // compute one step of Riccati difference equations
      riccatiEquationsPtrStock_[workerIndex]->computeMap(
          BASE::projectedModelDataTrajectoriesStock_[partitionIndex][k], BASE::riccatiModificationTrajectoriesStock_[partitionIndex][k],
          BASE::SmTrajectoryStock_[partitionIndex][k + 1], BASE::SvTrajectoryStock_[partitionIndex][k + 1],
          BASE::sTrajectoryStock_[partitionIndex][k + 1], projectedKmTrajectoryStock_[partitionIndex][k],
          projectedLvTrajectoryStock_[partitionIndex][k], BASE::SmTrajectoryStock_[partitionIndex][k],
          BASE::SvTrajectoryStock_[partitionIndex][k], BASE::sTrajectoryStock_[partitionIndex][k]);
    }  // end of k loop

    if (i > 0) {
      std::tie(SmFinalTemp, SvFinalTemp, sFinalTemp) = riccatiTransversalityConditions(
          BASE::modelDataEventTimesStock_[partitionIndex][i - 1], BASE::SmTrajectoryStock_[partitionIndex][beginTimeItr],
          BASE::SvTrajectoryStock_[partitionIndex][beginTimeItr], BASE::sTrajectoryStock_[partitionIndex][beginTimeItr]);
    }
  }  // end of i loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void ILQR::setupOptimizer(size_t numPartitions) {
  BASE::setupOptimizer(numPartitions);

  projectedLvTrajectoryStock_.resize(numPartitions);
  projectedKmTrajectoryStock_.resize(numPartitions);
}

}  // namespace ocs2
