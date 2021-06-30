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
#include "ocs2_ddp/riccati_equations/RiccatiModificationInterpolation.h"

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SLQ::SLQ(ddp::Settings ddpSettings, const RolloutBase& rollout, const OptimalControlProblem& optimalControlProblem,
         const Initializer& initializer)
    : BASE(std::move(ddpSettings), rollout, optimalControlProblem, initializer) {
  if (settings().algorithm_ != ddp::Algorithm::SLQ) {
    throw std::runtime_error("In DDP setting the algorithm name is set \"" + ddp::toAlgorithmName(settings().algorithm_) +
                             "\" while SLQ is instantiated!");
  }

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
void SLQ::approximateIntermediateLQ(const scalar_array_t& timeTrajectory, const size_array_t& postEventIndices,
                                    const vector_array_t& stateTrajectory, const vector_array_t& inputTrajectory,
                                    std::vector<ModelData>& modelDataTrajectory) {
  BASE::nextTimeIndex_ = 0;
  BASE::nextTaskId_ = 0;
  std::function<void(void)> task = [&] {
    size_t timeIndex;
    size_t taskId = BASE::nextTaskId_++;  // assign task ID (atomic)

    // get next time index is atomic
    while ((timeIndex = BASE::nextTimeIndex_++) < timeTrajectory.size()) {
      // execute approximateLQ for the given partition and time index

      LinearQuadraticApproximator lqapprox(*BASE::optimalControlProblemPtrStock_[taskId], BASE::settings().checkNumericalStability_);

      lqapprox.approximateLQProblem(timeTrajectory[timeIndex], stateTrajectory[timeIndex], inputTrajectory[timeIndex],
                                    modelDataTrajectory[timeIndex]);
      modelDataTrajectory[timeIndex].checkSizes(stateTrajectory[timeIndex].rows(), inputTrajectory[timeIndex].rows());
    }
  };

  BASE::runParallel(task, settings().nThreads_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SLQ::calculateControllerWorker(size_t workerIndex, size_t partitionIndex, size_t timeIndex) {
  const auto i = partitionIndex;
  const auto k = timeIndex;
  const auto time = BASE::SsTimeTrajectoryStock_[i][k];

  // interpolate
  const auto indexAlpha = LinearInterpolation::timeSegment(time, BASE::nominalTimeTrajectoriesStock_[i]);
  const vector_t nominalState = LinearInterpolation::interpolate(indexAlpha, BASE::nominalStateTrajectoriesStock_[i]);
  const vector_t nominalInput = LinearInterpolation::interpolate(indexAlpha, BASE::nominalInputTrajectoriesStock_[i]);

  // BmProjected
  const matrix_t projectedBm =
      LinearInterpolation::interpolate(indexAlpha, BASE::projectedModelDataTrajectoriesStock_[i], model_data::dynamics_dfdu);
  // PmProjected
  const matrix_t projectedPm =
      LinearInterpolation::interpolate(indexAlpha, BASE::projectedModelDataTrajectoriesStock_[i], model_data::cost_dfdux);
  // RvProjected
  const vector_t projectedRv =
      LinearInterpolation::interpolate(indexAlpha, BASE::projectedModelDataTrajectoriesStock_[i], model_data::cost_dfdu);
  // EvProjected
  const vector_t EvProjected =
      LinearInterpolation::interpolate(indexAlpha, BASE::projectedModelDataTrajectoriesStock_[i], model_data::stateInputEqConstr_f);
  // CmProjected
  const matrix_t CmProjected =
      LinearInterpolation::interpolate(indexAlpha, BASE::projectedModelDataTrajectoriesStock_[i], model_data::stateInputEqConstr_dfdx);

  // projector
  const matrix_t Qu = LinearInterpolation::interpolate(indexAlpha, BASE::riccatiModificationTrajectoriesStock_[i],
                                                       riccati_modification::constraintNullProjector);
  // deltaGm, projected feedback
  matrix_t projectedKm =
      LinearInterpolation::interpolate(indexAlpha, BASE::riccatiModificationTrajectoriesStock_[i], riccati_modification::deltaGm);
  // deltaGv, projected feedforward
  vector_t projectedLv =
      LinearInterpolation::interpolate(indexAlpha, BASE::riccatiModificationTrajectoriesStock_[i], riccati_modification::deltaGv);

  // projectedKm = projectedPm + projectedBm^t * Sm
  projectedKm = -(projectedKm + projectedPm);
  projectedKm.noalias() -= projectedBm.transpose() * BASE::SmTrajectoryStock_[i][k];

  // projectedLv = projectedRv + projectedBm^t * Sv
  projectedLv = -(projectedLv + projectedRv);
  projectedLv.noalias() -= projectedBm.transpose() * BASE::SvTrajectoryStock_[i][k];

  // feedback gains
  BASE::nominalControllersStock_[i].gainArray_[k] = -CmProjected;
  BASE::nominalControllersStock_[i].gainArray_[k].noalias() += Qu * projectedKm;

  // bias input
  BASE::nominalControllersStock_[i].biasArray_[k] = nominalInput;
  BASE::nominalControllersStock_[i].biasArray_[k].noalias() -= BASE::nominalControllersStock_[i].gainArray_[k] * nominalState;
  BASE::nominalControllersStock_[i].deltaBiasArray_[k] = -EvProjected;
  BASE::nominalControllersStock_[i].deltaBiasArray_[k].noalias() += Qu * projectedLv;

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
      std::cerr << "what(): " << error.what() << " at time " << time << " [sec]." << std::endl;
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
scalar_t SLQ::solveSequentialRiccatiEquations(const matrix_t& SmFinal, const vector_t& SvFinal, const scalar_t& sFinal) {
  // fully compute the riccatiModifications and projected modelData
  for (size_t i = 0; i < BASE::numPartitions_; i++) {
    // number of the intermediate LQ variables
    auto N = BASE::nominalTimeTrajectoriesStock_[i].size();

    BASE::riccatiModificationTrajectoriesStock_[i].resize(N);
    BASE::projectedModelDataTrajectoriesStock_[i].resize(N);

    if (N > 0) {
      // perform the computeRiccatiModificationTerms for partition i
      BASE::nextTimeIndex_ = 0;
      BASE::nextTaskId_ = 0;
      std::function<void(void)> task = [this, i] {
        int N = BASE::nominalTimeTrajectoriesStock_[i].size();
        int timeIndex;
        size_t taskId = BASE::nextTaskId_++;  // assign task ID (atomic)
        const auto SmDummy = matrix_t::Zero(0, 0);

        // get next time index is atomic
        while ((timeIndex = BASE::nextTimeIndex_++) < N) {
          BASE::computeProjectionAndRiccatiModification(BASE::modelDataTrajectoriesStock_[i][timeIndex], SmDummy,
                                                        BASE::projectedModelDataTrajectoriesStock_[i][timeIndex],
                                                        BASE::riccatiModificationTrajectoriesStock_[i][timeIndex]);
        }
      };
      BASE::runParallel(task, settings().nThreads_);
    }
  }

  return BASE::solveSequentialRiccatiEquationsImpl(SmFinal, SvFinal, sFinal);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_t SLQ::computeHamiltonianHessian(const ModelData& modelData, const matrix_t& Sm) const {
  return searchStrategyPtr_->augmentHamiltonianHessian(modelData, modelData.cost_.dfduu);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SLQ::riccatiEquationsWorker(size_t workerIndex, size_t partitionIndex, const matrix_t& SmFinal, const vector_t& SvFinal,
                                 const scalar_t& sFinal) {
  // set data for Riccati equations
  riccatiEquationsPtrStock_[workerIndex]->resetNumFunctionCalls();
  riccatiEquationsPtrStock_[workerIndex]->setData(
      &BASE::nominalTimeTrajectoriesStock_[partitionIndex], &BASE::projectedModelDataTrajectoriesStock_[partitionIndex],
      &BASE::nominalPostEventIndicesStock_[partitionIndex], &BASE::modelDataEventTimesStock_[partitionIndex],
      &BASE::riccatiModificationTrajectoriesStock_[partitionIndex]);

  // const partition containers
  const auto& nominalTimeTrajectory = BASE::nominalTimeTrajectoriesStock_[partitionIndex];
  const auto& nominalEventsPastTheEndIndices = BASE::nominalPostEventIndicesStock_[partitionIndex];

  // Modified partition containers
  auto& SsNormalizedTime = BASE::SsNormalizedTimeTrajectoryStock_[partitionIndex];
  auto& SsNormalizedPostEventIndices = BASE::SsNormalizedEventsPastTheEndIndecesStock_[partitionIndex];
  auto& SsTimeTrajectory = BASE::SsTimeTrajectoryStock_[partitionIndex];
  auto& SmTrajectory = BASE::SmTrajectoryStock_[partitionIndex];
  auto& SvTrajectory = BASE::SvTrajectoryStock_[partitionIndex];
  auto& sTrajectory = BASE::sTrajectoryStock_[partitionIndex];

  // Convert final value of value function in vector format
  vector_t allSsFinal = ContinuousTimeRiccatiEquations::convert2Vector(SmFinal, SvFinal, sFinal);

  // Clear output containers
  SsNormalizedTime.clear();
  SsNormalizedPostEventIndices.clear();

  /*
   *  The riccati equations are solved backwards in time
   *  the SsNormalized time is therefore filled with negative time in the reverse order, for example:
   *  nominalTime = [0.0, 1.0, 2.0, ..., 10.0]
   *  SsNormalized = [-10.0, ..., -2.0, -1.0, -0.0]
   *
   *  Depending on settings_.useNominalTimeForBackwardPass_:
   *  if true: the integration will produce the same time nodes set in nominalTime (=resulting from the forward pass),
   *  if false: the SsNormalized time is a result of adaptive integration.
   */
  vector_array_t allSsTrajectory;
  if (settings().useNominalTimeForBackwardPass_) {
    integrateRiccatiEquationNominalTime(*riccatiIntegratorPtrStock_[workerIndex], *riccatiEquationsPtrStock_[workerIndex],
                                        nominalTimeTrajectory, nominalEventsPastTheEndIndices, std::move(allSsFinal), SsNormalizedTime,
                                        SsNormalizedPostEventIndices, allSsTrajectory);
  } else {
    integrateRiccatiEquationAdaptiveTime(*riccatiIntegratorPtrStock_[workerIndex], *riccatiEquationsPtrStock_[workerIndex],
                                         nominalTimeTrajectory, nominalEventsPastTheEndIndices, std::move(allSsFinal), SsNormalizedTime,
                                         SsNormalizedPostEventIndices, allSsTrajectory);
  }

  // De-normalize time and convert value function to matrix format
  size_t outputN = SsNormalizedTime.size();
  SsTimeTrajectory.resize(outputN);
  SmTrajectory.resize(outputN);
  SvTrajectory.resize(outputN);
  sTrajectory.resize(outputN);
  for (size_t k = 0; k < outputN; k++) {
    SsTimeTrajectory[k] = -SsNormalizedTime[outputN - 1 - k];
    ContinuousTimeRiccatiEquations::convert2Matrix(allSsTrajectory[outputN - 1 - k], SmTrajectory[k], SvTrajectory[k], sTrajectory[k]);
  }  // end of k loop

  if (settings().debugPrintRollout_) {
    std::cerr << std::endl << "+++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
    std::cerr << "Partition: " << partitionIndex << ", backward pass time trajectory";
    std::cerr << std::endl << "+++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
    for (size_t k = 0; k < outputN; k++) {
      std::cerr << "k: " << k << ", t = " << std::setprecision(12) << SsTimeTrajectory[k] << "\n";
    }
    std::cerr << std::endl;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SLQ::integrateRiccatiEquationNominalTime(IntegratorBase& riccatiIntegrator, ContinuousTimeRiccatiEquations& riccatiEquation,
                                              const scalar_array_t& nominalTimeTrajectory,
                                              const size_array_t& nominalEventsPastTheEndIndices, vector_t allSsFinal,
                                              scalar_array_t& SsNormalizedTime, size_array_t& SsNormalizedPostEventIndices,
                                              vector_array_t& allSsTrajectory) {
  // Extract sizes
  const int nominalTimeSize = nominalTimeTrajectory.size();
  const int numEvents = nominalEventsPastTheEndIndices.size();
  auto partitionDuration = nominalTimeTrajectory.back() - nominalTimeTrajectory.front();
  const auto maxNumSteps = static_cast<size_t>(settings().maxNumStepsPerSecond_ * std::max(1.0, partitionDuration));

  // normalized time and post event indices
  BASE::computeNormalizedTime(nominalTimeTrajectory, nominalEventsPastTheEndIndices, SsNormalizedTime, SsNormalizedPostEventIndices);

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
  allSsTrajectory.reserve(maxNumSteps);
  for (int i = 0; i <= numEvents; i++) {
    iterator_t beginTimeItr = SsNormalizedSwitchingTimesIndices[i].first;
    iterator_t endTimeItr = SsNormalizedSwitchingTimesIndices[i].second;

    Observer observer(&allSsTrajectory);
    // solve Riccati equations
    riccatiIntegrator.integrateTimes(riccatiEquation, observer, allSsFinal, beginTimeItr, endTimeItr, settings().timeStep_,
                                     settings().absTolODE_, settings().relTolODE_, maxNumSteps);

    if (i < numEvents) {
      allSsFinal = riccatiEquation.computeJumpMap(*endTimeItr, allSsTrajectory.back());
    }
  }  // end of i loop

  // check size
  if (allSsTrajectory.size() != nominalTimeSize) {
    throw std::runtime_error("allSsTrajectory size is incorrect.");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SLQ::integrateRiccatiEquationAdaptiveTime(IntegratorBase& riccatiIntegrator, ContinuousTimeRiccatiEquations& riccatiEquation,
                                               const scalar_array_t& nominalTimeTrajectory,
                                               const size_array_t& nominalEventsPastTheEndIndices, vector_t allSsFinal,
                                               scalar_array_t& SsNormalizedTime, size_array_t& SsNormalizedPostEventIndices,
                                               vector_array_t& allSsTrajectory) {
  // Extract sizes
  const int nominalTimeSize = nominalTimeTrajectory.size();
  const int numEvents = nominalEventsPastTheEndIndices.size();
  auto partitionDuration = nominalTimeTrajectory.back() - nominalTimeTrajectory.front();
  const auto maxNumSteps = static_cast<size_t>(settings().maxNumStepsPerSecond_ * std::max(1.0, partitionDuration));

  // Extract switching times from eventIndices and normalize them
  std::vector<std::pair<scalar_t, scalar_t>> SsNormalizedSwitchingTimes;
  SsNormalizedSwitchingTimes.reserve(numEvents + 1);
  SsNormalizedSwitchingTimes.emplace_back(-nominalTimeTrajectory.back(), 0.0);
  for (auto itr = nominalEventsPastTheEndIndices.rbegin(); itr != nominalEventsPastTheEndIndices.rend(); ++itr) {
    SsNormalizedSwitchingTimes.back().second = -nominalTimeTrajectory[*itr];
    SsNormalizedSwitchingTimes.emplace_back(-nominalTimeTrajectory[(*itr) - 1], 0.0);
  }
  SsNormalizedSwitchingTimes.back().second = -nominalTimeTrajectory.front();

  // integrating the Riccati equations
  SsNormalizedTime.reserve(maxNumSteps);
  SsNormalizedPostEventIndices.reserve(numEvents);
  allSsTrajectory.reserve(maxNumSteps);
  for (int i = 0; i <= numEvents; i++) {
    const scalar_t beginTime = SsNormalizedSwitchingTimes[i].first;
    const scalar_t endTime = SsNormalizedSwitchingTimes[i].second;

    Observer observer(&allSsTrajectory, &SsNormalizedTime);
    // solve Riccati equations
    riccatiIntegrator.integrateAdaptive(riccatiEquation, observer, allSsFinal, beginTime, endTime, settings().timeStep_,
                                        settings().absTolODE_, settings().relTolODE_, maxNumSteps);

    // if not the last interval which definitely does not have any event at
    // its final time (there is no even at the beginning of partition)
    if (i < numEvents) {
      SsNormalizedPostEventIndices.push_back(allSsTrajectory.size());
      allSsFinal = riccatiEquation.computeJumpMap(endTime, allSsTrajectory.back());
    }
  }  // end of i loop
}

}  // namespace ocs2
