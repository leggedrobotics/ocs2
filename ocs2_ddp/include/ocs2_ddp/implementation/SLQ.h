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

#include <ocs2_ddp/SLQ.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
SLQ<STATE_DIM, INPUT_DIM>::SLQ(const rollout_base_t* rolloutPtr, const derivatives_base_t* systemDerivativesPtr,
                               const constraint_base_t* systemConstraintsPtr, const cost_function_base_t* costFunctionPtr,
                               const operating_trajectories_base_t* operatingTrajectoriesPtr,
                               const SLQ_Settings& settings /*= SLQ_Settings()*/,
                               std::shared_ptr<HybridLogicRules> logicRulesPtr /*= nullptr*/,
                               const cost_function_base_t* heuristicsFunctionPtr /* = nullptr*/)

    : BASE(rolloutPtr, systemDerivativesPtr, systemConstraintsPtr, costFunctionPtr, operatingTrajectoriesPtr, settings.ddpSettings_,
           heuristicsFunctionPtr, "SLQ", std::move(logicRulesPtr)),
      settings_(settings) {
  // Riccati Solver
  riccatiEquationsPtrStock_.clear();
  riccatiEquationsPtrStock_.reserve(BASE::ddpSettings_.nThreads_);
  riccatiIntegratorPtrStock_.clear();
  riccatiIntegratorPtrStock_.reserve(BASE::ddpSettings_.nThreads_);

  IntegratorType integratorType = settings_.RiccatiIntegratorType_;
  if (integratorType != IntegratorType::ODE45 && integratorType != IntegratorType::BULIRSCH_STOER) {
    throw(
        std::runtime_error("Unsupported Riccati equation integrator type: " + integrator_type::toString(settings_.RiccatiIntegratorType_)));
  }

  for (size_t i = 0; i < BASE::ddpSettings_.nThreads_; i++) {
    bool preComputeRiccatiTerms = BASE::ddpSettings_.preComputeRiccatiTerms_ && (BASE::ddpSettings_.strategy_ == DDP_Strategy::LINE_SEARCH);
    bool isRiskSensitive = !numerics::almost_eq(BASE::ddpSettings_.riskSensitiveCoeff_, 0.0);
    riccatiEquationsPtrStock_.emplace_back(new riccati_equations_t(preComputeRiccatiTerms, isRiskSensitive));
    riccatiEquationsPtrStock_.back()->setRiskSensitiveCoefficient(BASE::ddpSettings_.riskSensitiveCoeff_);
    riccatiIntegratorPtrStock_.emplace_back(newIntegrator<Eigen::Dynamic>(integratorType));
  }  // end of i loop

  Eigen::initParallel();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void SLQ<STATE_DIM, INPUT_DIM>::approximateIntermediateLQ(const scalar_array_t& timeTrajectory, const size_array_t& postEventIndices,
                                                          const state_vector_array_t& stateTrajectory,
                                                          const input_vector_array_t& inputTrajectory,
                                                          ModelDataBase::array_t& modelDataTrajectory) {
  BASE::nextTimeIndex_ = 0;
  BASE::nextTaskId_ = 0;
  std::function<void(void)> task = [&] {
    size_t timeIndex;
    size_t taskId = BASE::nextTaskId_++;  // assign task ID (atomic)

    // get next time index is atomic
    while ((timeIndex = BASE::nextTimeIndex_++) < timeTrajectory.size()) {
      // execute approximateLQ for the given partition and time index
      BASE::linearQuadraticApproximatorPtrStock_[taskId]->approximateUnconstrainedLQProblem(
          timeTrajectory[timeIndex], stateTrajectory[timeIndex], inputTrajectory[timeIndex], modelDataTrajectory[timeIndex]);
    }
  };

  BASE::runParallel(task, BASE::ddpSettings_.nThreads_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void SLQ<STATE_DIM, INPUT_DIM>::calculateControllerWorker(size_t workerIndex, size_t partitionIndex, size_t timeIndex) {
  const auto i = partitionIndex;
  const auto k = timeIndex;
  const auto time = BASE::SsTimeTrajectoryStock_[i][k];

  // local variables
  state_vector_t nominalState;
  input_vector_t nominalInput;

  dynamic_matrix_t projectedBm;
  dynamic_matrix_t projectedPm;
  dynamic_vector_t projectedRv;

  dynamic_matrix_t projectedKm;  // projected feedback
  dynamic_vector_t projectedLv;  // projected feedforward

  dynamic_matrix_t Qu;  // projector
  dynamic_matrix_t CmProjected;
  dynamic_vector_t EvProjected;

  // interpolate
  const auto indexAlpha = LinearInterpolation::timeSegment(time, &(BASE::nominalTimeTrajectoriesStock_[i]));
  LinearInterpolation::interpolate(indexAlpha, nominalState, &(BASE::nominalStateTrajectoriesStock_[i]));
  LinearInterpolation::interpolate(indexAlpha, nominalInput, &(BASE::nominalInputTrajectoriesStock_[i]));

  // BmProjected
  ModelData::interpolate(indexAlpha, projectedBm, &BASE::projectedModelDataTrajectoriesStock_[i], ModelData::dynamicsInputDerivative);
  // PmProjected
  ModelData::interpolate(indexAlpha, projectedPm, &BASE::projectedModelDataTrajectoriesStock_[i], ModelData::costInputStateDerivative);
  // RvProjected
  ModelData::interpolate(indexAlpha, projectedRv, &BASE::projectedModelDataTrajectoriesStock_[i], ModelData::costInputDerivative);
  // EvProjected
  ModelData::interpolate(indexAlpha, EvProjected, &BASE::projectedModelDataTrajectoriesStock_[i], ModelData::stateInputEqConstr);
  // CmProjected
  ModelData::interpolate(indexAlpha, CmProjected, &BASE::projectedModelDataTrajectoriesStock_[i],
                         ModelData::stateInputEqConstrStateDerivative);

  // Qu
  RiccatiModification::interpolate(indexAlpha, Qu, &BASE::riccatiModificationTrajectoriesStock_[i],
                                   RiccatiModification::constraintNullProjector);
  // deltaGm
  RiccatiModification::interpolate(indexAlpha, projectedKm, &BASE::riccatiModificationTrajectoriesStock_[i], RiccatiModification::deltaGm);
  // deltaGv
  RiccatiModification::interpolate(indexAlpha, projectedLv, &BASE::riccatiModificationTrajectoriesStock_[i], RiccatiModification::deltaGv);

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
  if (BASE::ddpSettings_.checkNumericalStability_) {
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
template <size_t STATE_DIM, size_t INPUT_DIM>
typename SLQ<STATE_DIM, INPUT_DIM>::scalar_t SLQ<STATE_DIM, INPUT_DIM>::solveSequentialRiccatiEquations(const dynamic_matrix_t& SmFinal,
                                                                                                        const dynamic_vector_t& SvFinal,
                                                                                                        const scalar_t& sFinal) {
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
        const auto SmDummy = dynamic_matrix_t::Zero(0, 0);

        // get next time index is atomic
        while ((timeIndex = BASE::nextTimeIndex_++) < N) {
          BASE::computeProjectionAndRiccatiModification(BASE::ddpSettings_.strategy_, BASE::modelDataTrajectoriesStock_[i][timeIndex],
                                                        SmDummy, BASE::projectedModelDataTrajectoriesStock_[i][timeIndex],
                                                        BASE::riccatiModificationTrajectoriesStock_[i][timeIndex]);
        }
      };
      BASE::runParallel(task, BASE::ddpSettings_.nThreads_);
    }
  }

  return BASE::solveSequentialRiccatiEquationsImpl(SmFinal, SvFinal, sFinal);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
typename SLQ<STATE_DIM, INPUT_DIM>::dynamic_matrix_t SLQ<STATE_DIM, INPUT_DIM>::computeHamiltonianHessian(
    DDP_Strategy strategy, const ModelDataBase& modelData, const dynamic_matrix_t& Sm) const {
  const auto& Bm = modelData.dynamicsInputDerivative_;
  const auto& Rm = modelData.costInputSecondDerivative_;
  switch (strategy) {
    case DDP_Strategy::LINE_SEARCH: {
      return Rm;
    }
    case DDP_Strategy::LEVENBERG_MARQUARDT: {
      auto HmAug = Rm;
      HmAug.noalias() += BASE::levenbergMarquardtImpl_.riccatiMultiple * Bm.transpose() * Bm;
      return HmAug;
    }
  }  // end of switch-case
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
SLQ_Settings& SLQ<STATE_DIM, INPUT_DIM>::settings() {
  return settings_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void SLQ<STATE_DIM, INPUT_DIM>::riccatiEquationsWorker(size_t workerIndex, size_t partitionIndex, const dynamic_matrix_t& SmFinal,
                                                       const dynamic_vector_t& SvFinal, const scalar_t& sFinal) {
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
  dynamic_vector_t allSsFinal;
  riccati_equations_t::convert2Vector(SmFinal, SvFinal, sFinal, allSsFinal);

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
  dynamic_vector_array_t allSsTrajectory;
  if (settings_.useNominalTimeForBackwardPass_) {
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
    riccati_equations_t::convert2Matrix(allSsTrajectory[outputN - 1 - k], SmTrajectory[k], SvTrajectory[k], sTrajectory[k]);
  }  // end of k loop

  if (BASE::ddpSettings_.debugPrintRollout_) {
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
template <size_t STATE_DIM, size_t INPUT_DIM>
void SLQ<STATE_DIM, INPUT_DIM>::integrateRiccatiEquationNominalTime(
    IntegratorBase<Eigen::Dynamic>& riccatiIntegrator, riccati_equations_t& riccatiEquation, const scalar_array_t& nominalTimeTrajectory,
    const size_array_t& nominalEventsPastTheEndIndices, dynamic_vector_t allSsFinal, scalar_array_t& SsNormalizedTime,
    size_array_t& SsNormalizedPostEventIndices, dynamic_vector_array_t& allSsTrajectory) {
  // Extract sizes
  const int nominalTimeSize = nominalTimeTrajectory.size();
  const int numEvents = nominalEventsPastTheEndIndices.size();
  auto partitionDuration = nominalTimeTrajectory.back() - nominalTimeTrajectory.front();
  const auto maxNumSteps = static_cast<size_t>(BASE::ddpSettings_.maxNumStepsPerSecond_ * std::max(1.0, partitionDuration));

  // normalized time and post event indices
  BASE::computeNormalizedTime(nominalTimeTrajectory, nominalEventsPastTheEndIndices, SsNormalizedTime, SsNormalizedPostEventIndices);

  // Normalized switching time indices, start and end of the partition are added at the beginning and end
  std::vector<int> SsNormalizedSwitchingTimesIndices;
  SsNormalizedSwitchingTimesIndices.reserve(numEvents + 2);
  SsNormalizedSwitchingTimesIndices.push_back(0);
  for (int k = numEvents - 1; k >= 0; k--) {
    int index = static_cast<int>(nominalEventsPastTheEndIndices[k]);
    SsNormalizedSwitchingTimesIndices.push_back(nominalTimeSize - index);
  }
  SsNormalizedSwitchingTimesIndices.push_back(nominalTimeSize);

  // integrating the Riccati equations
  allSsTrajectory.reserve(maxNumSteps);
  for (int i = 0; i <= numEvents; i++) {
    typename scalar_array_t::const_iterator beginTimeItr = SsNormalizedTime.begin() + SsNormalizedSwitchingTimesIndices[i];
    typename scalar_array_t::const_iterator endTimeItr = SsNormalizedTime.begin() + SsNormalizedSwitchingTimesIndices[i + 1];

    Observer<Eigen::Dynamic> observer(&allSsTrajectory);
    // solve Riccati equations
    riccatiIntegrator.integrate_times(riccatiEquation, observer, allSsFinal, beginTimeItr, endTimeItr, BASE::ddpSettings_.minTimeStep_,
                                      BASE::ddpSettings_.absTolODE_, BASE::ddpSettings_.relTolODE_, maxNumSteps);

    if (i < numEvents) {
      riccatiEquation.computeJumpMap(*endTimeItr, allSsTrajectory.back(), allSsFinal);
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
template <size_t STATE_DIM, size_t INPUT_DIM>
void SLQ<STATE_DIM, INPUT_DIM>::integrateRiccatiEquationAdaptiveTime(
    IntegratorBase<Eigen::Dynamic>& riccatiIntegrator, riccati_equations_t& riccatiEquation, const scalar_array_t& nominalTimeTrajectory,
    const size_array_t& nominalEventsPastTheEndIndices, dynamic_vector_t allSsFinal, scalar_array_t& SsNormalizedTime,
    size_array_t& SsNormalizedPostEventIndices, dynamic_vector_array_t& allSsTrajectory) {
  // Extract sizes
  const int nominalTimeSize = nominalTimeTrajectory.size();
  const int numEvents = nominalEventsPastTheEndIndices.size();
  auto partitionDuration = nominalTimeTrajectory.back() - nominalTimeTrajectory.front();
  const auto maxNumSteps = static_cast<size_t>(BASE::ddpSettings_.maxNumStepsPerSecond_ * std::max(1.0, partitionDuration));

  // Extract switching times from eventIndices and normalize them
  scalar_array_t SsNormalizedSwitchingTimes;
  SsNormalizedSwitchingTimes.reserve(numEvents + 2);
  SsNormalizedSwitchingTimes.push_back(-nominalTimeTrajectory.back());
  for (int k = numEvents - 1; k >= 0; k--) {
    size_t index = nominalEventsPastTheEndIndices[k];
    SsNormalizedSwitchingTimes.push_back(-nominalTimeTrajectory[index]);
  }
  SsNormalizedSwitchingTimes.push_back(-nominalTimeTrajectory.front());

  // integrating the Riccati equations
  SsNormalizedTime.reserve(maxNumSteps);
  SsNormalizedPostEventIndices.reserve(numEvents);
  allSsTrajectory.reserve(maxNumSteps);
  for (int i = 0; i <= numEvents; i++) {
    scalar_t beginTime = SsNormalizedSwitchingTimes[i];
    scalar_t endTime = SsNormalizedSwitchingTimes[i + 1];

    Observer<Eigen::Dynamic> observer(&allSsTrajectory, &SsNormalizedTime);
    // solve Riccati equations
    riccatiIntegrator.integrate_adaptive(riccatiEquation, observer, allSsFinal, beginTime, endTime, BASE::ddpSettings_.minTimeStep_,
                                         BASE::ddpSettings_.absTolODE_, BASE::ddpSettings_.relTolODE_, maxNumSteps);

    // if not the last interval which definitely does not have any event at
    // its final time (there is no even at the beginning of partition)
    if (i < numEvents) {
      SsNormalizedPostEventIndices.push_back(allSsTrajectory.size());
      riccatiEquation.computeJumpMap(endTime, allSsTrajectory.back(), allSsFinal);
    }
  }  // end of i loop
}

}  // namespace ocs2
