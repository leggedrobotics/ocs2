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

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
GDDP<STATE_DIM, INPUT_DIM>::GDDP(GDDP_Settings gddpSettings) : gddpSettings_(std::move(gddpSettings)) {
  bvpSensitivityEquationsPtrStock_.clear();
  bvpSensitivityEquationsPtrStock_.reserve(gddpSettings_.nThreads_);
  bvpSensitivityIntegratorsPtrStock_.clear();
  bvpSensitivityIntegratorsPtrStock_.reserve(gddpSettings_.nThreads_);

  bvpSensitivityErrorEquationsPtrStock_.clear();
  bvpSensitivityErrorEquationsPtrStock_.reserve(gddpSettings_.nThreads_);
  bvpSensitivityErrorIntegratorsPtrStock_.clear();
  bvpSensitivityErrorIntegratorsPtrStock_.reserve(gddpSettings_.nThreads_);

  rolloutSensitivityEquationsPtrStock_.clear();
  rolloutSensitivityEquationsPtrStock_.reserve(gddpSettings_.nThreads_);
  rolloutSensitivityIntegratorsPtrStock_.clear();
  rolloutSensitivityIntegratorsPtrStock_.reserve(gddpSettings_.nThreads_);

  riccatiSensitivityEquationsPtrStock_.clear();
  riccatiSensitivityEquationsPtrStock_.reserve(gddpSettings_.nThreads_);
  riccatiSensitivityIntegratorsPtrStock_.clear();
  riccatiSensitivityIntegratorsPtrStock_.reserve(gddpSettings_.nThreads_);

  IntegratorType integratorType = gddpSettings_.riccatiIntegratorType_;
  if (integratorType != IntegratorType::ODE45 && integratorType != IntegratorType::BULIRSCH_STOER) {
    throw(std::runtime_error("Unsupported Riccati equation integrator type: " + integrator_type::toString(integratorType)));
  }

  for (size_t i = 0; i < gddpSettings_.nThreads_; i++) {
    bvpSensitivityEquationsPtrStock_.emplace_back(new bvp_sensitivity_equations_t);
    bvpSensitivityErrorEquationsPtrStock_.emplace_back(new bvp_sensitivity_error_equations_t);
    rolloutSensitivityEquationsPtrStock_.emplace_back(new rollout_sensitivity_equations_t);
    riccatiSensitivityEquationsPtrStock_.emplace_back(new riccati_sensitivity_equations_t);

    bvpSensitivityIntegratorsPtrStock_.emplace_back(newIntegrator<STATE_DIM>(integratorType));
    bvpSensitivityErrorIntegratorsPtrStock_.emplace_back(newIntegrator<STATE_DIM>(integratorType));
    rolloutSensitivityIntegratorsPtrStock_.emplace_back(newIntegrator<STATE_DIM>(integratorType));
    riccatiSensitivityIntegratorsPtrStock_.emplace_back(newIntegrator<riccati_sensitivity_equations_t::S_DIM_>(integratorType));
  }  // end of i loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void GDDP<STATE_DIM, INPUT_DIM>::setupOptimizer(const size_t& numPartitions) {
  if (numPartitions == 0) {
    throw std::runtime_error("The number of partitions cannot be zero!");
  }
}

/*****************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void GDDP<STATE_DIM, INPUT_DIM>::calculateRolloutCostate(const scalar_array2_t& timeTrajectoriesStock,
                                                         const state_vector_array2_t& stateTrajectoriesStock,
                                                         state_vector_array2_t& costateTrajectoriesStock, scalar_t learningRate /*= 0.0*/) {
  costateTrajectoriesStock.resize(numPartitions_);

  for (size_t i = 0; i < numPartitions_; i++) {
    // skip the inactive partitions
    if (i < dataCollectorPtr_->initActivePartition_ || i > dataCollectorPtr_->finalActivePartition_) {
      costateTrajectoriesStock[i].clear();
      continue;
    }

    const size_t N = timeTrajectoriesStock[i].size();
    costateTrajectoriesStock[i].resize(N);
    for (size_t k = 0; k < N; k++) {
      const scalar_t& t = timeTrajectoriesStock[i][k];

      const auto indexAlpha = LinearInterpolation::timeSegment(t, &dataCollectorPtr_->SsTimeTrajectoriesStock_[i]);
      state_matrix_t Sm;
      LinearInterpolation::interpolate(indexAlpha, Sm, &dataCollectorPtr_->SmTrajectoriesStock_[i]);
      state_vector_t Sv;
      LinearInterpolation::interpolate(indexAlpha, Sv, &dataCollectorPtr_->SvTrajectoriesStock_[i]);
      state_vector_t Sve;
      LinearInterpolation::interpolate(indexAlpha, Sve, &dataCollectorPtr_->SveTrajectoriesStock_[i]);

      state_vector_t nominalState;
      LinearInterpolation::interpolate(t, nominalState, &dataCollectorPtr_->nominalTimeTrajectoriesStock_[i],
                                       &dataCollectorPtr_->nominalStateTrajectoriesStock_[i]);

      costateTrajectoriesStock[i][k] = Sve + Sv + learningRate * Sm * (stateTrajectoriesStock[i][k] - nominalState);

    }  // end of k loop
  }    // end of i loop
}

/*****************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void GDDP<STATE_DIM, INPUT_DIM>::calculateRolloutCostate(const std::vector<scalar_array_t>& timeTrajectoriesStock,
                                                         state_vector_array2_t& costateTrajectoriesStock) {
  costateTrajectoriesStock.resize(numPartitions_);

  for (size_t i = 0; i < numPartitions_; i++) {
    // skip the inactive partitions
    if (i < dataCollectorPtr_->initActivePartition_ || i > dataCollectorPtr_->finalActivePartition_) {
      costateTrajectoriesStock[i].clear();
      continue;
    }

    const size_t N = timeTrajectoriesStock[i].size();
    costateTrajectoriesStock[i].resize(N);
    for (size_t k = 0; k < N; k++) {
      const scalar_t& t = timeTrajectoriesStock[i][k];
      auto indexAlpha = LinearInterpolation::timeSegment(t, &dataCollectorPtr_->SsTimeTrajectoriesStock_[i]);
      state_vector_t Sv;
      LinearInterpolation::interpolate(indexAlpha, Sv, &dataCollectorPtr_->SvTrajectoriesStock_[i]);
      state_vector_t Sve;
      LinearInterpolation::interpolate(indexAlpha, Sve, &dataCollectorPtr_->SveTrajectoriesStock_[i]);

      costateTrajectoriesStock[i][k] = Sve + Sv;
    }  // end of k loop
  }    // end of i loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void GDDP<STATE_DIM, INPUT_DIM>::calculateNominalRolloutLagrangeMultiplier(const scalar_array2_t& timeTrajectoriesStock,
                                                                           dynamic_vector_array2_t& lagrangeTrajectoriesStock) {
  lagrangeTrajectoriesStock.resize(numPartitions_);

  for (size_t i = 0; i < numPartitions_; i++) {
    // skip the inactive partitions
    if (i < dataCollectorPtr_->initActivePartition_ || i > dataCollectorPtr_->finalActivePartition_) {
      lagrangeTrajectoriesStock[i].clear();
      continue;
    }

    size_t N = timeTrajectoriesStock[i].size();
    lagrangeTrajectoriesStock[i].resize(N);
    for (size_t k = 0; k < N; k++) {
      const auto& Bm = dataCollectorPtr_->modelDataTrajectoriesStock_[i][k].dynamicsInputDerivative_;
      const auto& Rv = dataCollectorPtr_->modelDataTrajectoriesStock_[i][k].costInputDerivative_;
      const auto& Rm = dataCollectorPtr_->modelDataTrajectoriesStock_[i][k].costInputSecondDerivative_;
      const auto& EvProjected = dataCollectorPtr_->projectedModelDataTrajectoriesStock_[i][k].stateInputEqConstr_;
      const auto& DmDager = dataCollectorPtr_->DmDagerTrajectoriesStock_[i][k];
      const auto& costate = nominalCostateTrajectoriesStock_[i][k];

      lagrangeTrajectoriesStock[i][k] = DmDager.transpose() * (Rm * EvProjected - Rv - Bm.transpose() * costate);
    }  // end of k loop
  }    // end of i loop
}

/*****************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void GDDP<STATE_DIM, INPUT_DIM>::computeEquivalentSystemMultiplier(const size_t& eventTimeIndex, const size_t& activeSubsystem,
                                                                   scalar_t& multiplier) const {
  scalar_t timePeriod;

  if (activeSubsystem == eventTimeIndex + 1) {
    if (activeSubsystem == eventTimes_.size()) {
      if (dataCollectorPtr_->finalTime_ < eventTimes_[eventTimeIndex]) {
        throw std::runtime_error("Final time is smaller than the last triggered event time.");
      } else {
        timePeriod = dataCollectorPtr_->finalTime_ - eventTimes_[eventTimeIndex];
      }
    } else {
      timePeriod = eventTimes_[eventTimeIndex + 1] - eventTimes_[eventTimeIndex];
    }

    multiplier = -1.0 / timePeriod;

  } else if (activeSubsystem == eventTimeIndex) {
    if (activeSubsystem == 0) {
      if (dataCollectorPtr_->initTime_ > eventTimes_[eventTimeIndex]) {
        throw std::runtime_error("Initial time is greater than the last triggered event time.");
      } else {
        timePeriod = eventTimes_[eventTimeIndex] - dataCollectorPtr_->initTime_;
      }
    } else {
      timePeriod = eventTimes_[eventTimeIndex] - eventTimes_[eventTimeIndex - 1];
    }

    multiplier = 1.0 / timePeriod;

  } else {
    timePeriod = 1.0;
    multiplier = 0.0;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void GDDP<STATE_DIM, INPUT_DIM>::getRolloutSensitivity2EventTime(const size_t& eventTimeIndex,
                                                                 std::vector<scalar_array_t>& sensitivityTimeTrajectoriesStock,
                                                                 state_matrix_array2_t& sensitivityStateTrajectoriesStock,
                                                                 input_matrix_array2_t& sensitivityInputTrajectoriesStock) {
  if (eventTimeIndex + 1 > numEventTimes_) {
    throw std::runtime_error("The requested event index is out of bound.");
  }

  sensitivityTimeTrajectoriesStock = dataCollectorPtr_->nominalTimeTrajectoriesStock_;
  sensitivityStateTrajectoriesStock = sensitivityStateTrajectoriesStockSet_[eventTimeIndex];
  sensitivityInputTrajectoriesStock = sensitivityInputTrajectoriesStockSet_[eventTimeIndex];
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
GDDP_Settings& GDDP<STATE_DIM, INPUT_DIM>::settings() {
  return gddpSettings_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
template <typename Derived>
void GDDP<STATE_DIM, INPUT_DIM>::getCostFuntionDerivative(Eigen::MatrixBase<Derived> const& costFunctionDerivative) const {
  // refer to Eigen documentation under the topic "Writing Functions Taking Eigen Types as Parameters"
  const_cast<Eigen::MatrixBase<Derived>&>(costFunctionDerivative) = nominalCostFuntionDerivative_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
const typename GDDP<STATE_DIM, INPUT_DIM>::scalar_array_t& GDDP<STATE_DIM, INPUT_DIM>::eventTimes() const {
  return eventTimes_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void GDDP<STATE_DIM, INPUT_DIM>::propagateRolloutSensitivity(size_t workerIndex, const size_t& eventTimeIndex,
                                                             const linear_controller_array_t& controllersStock,
                                                             const input_vector_array2_t& LvTrajectoriesStock,
                                                             const scalar_array2_t& sensitivityTimeTrajectoriesStock,
                                                             const size_array2_t& postEventIndicesStock,
                                                             state_vector_array2_t& sensitivityStateTrajectoriesStock,
                                                             input_vector_array2_t& sensitivityInputTrajectoriesStock) {
  if (eventTimeIndex < activeEventTimeBeginIndex_ || eventTimeIndex >= activeEventTimeEndIndex_) {
    throw std::runtime_error("The index is associated to an inactive event or it is out of range.");
  }

  // resizing
  sensitivityStateTrajectoriesStock.resize(numPartitions_);
  sensitivityInputTrajectoriesStock.resize(numPartitions_);

  // Initial state sensitivity (which is zero)
  state_vector_t nabla_xInit = state_vector_t::Zero();

  for (size_t i = 0; i < numPartitions_; i++) {
    // skip inactive partitions
    if (i < dataCollectorPtr_->initActivePartition_ || i > dataCollectorPtr_->finalActivePartition_) {
      sensitivityStateTrajectoriesStock[i].clear();
      sensitivityInputTrajectoriesStock[i].clear();
      continue;
    }

    const int N = sensitivityTimeTrajectoriesStock[i].size();
    const int NE = postEventIndicesStock[i].size();

    // set data for rollout sensitivity equation
    rolloutSensitivityEquationsPtrStock_[workerIndex]->resetNumFunctionCalls();
    rolloutSensitivityEquationsPtrStock_[workerIndex]->setData(
        &dataCollectorPtr_->nominalTimeTrajectoriesStock_[i], &dataCollectorPtr_->modelDataTrajectoriesStock_[i],
        &controllersStock[i].timeStamp_, &LvTrajectoriesStock[i], &controllersStock[i].gainArray_);

    // max number of steps of integration
    const size_t maxNumSteps = gddpSettings_.maxNumStepsPerSecond_ *
                               std::max(1.0, sensitivityTimeTrajectoriesStock[i].back() - sensitivityTimeTrajectoriesStock[i].front());

    // resizing
    sensitivityStateTrajectoriesStock[i].clear();
    sensitivityStateTrajectoriesStock[i].reserve(N);
    sensitivityInputTrajectoriesStock[i].clear();
    sensitivityInputTrajectoriesStock[i].reserve(N);

    // integrating
    int k_u = 0;
    typename scalar_array_t::const_iterator beginTimeItr, endTimeItr;
    for (int j = 0; j <= NE; j++) {
      beginTimeItr = (j == 0) ? sensitivityTimeTrajectoriesStock[i].begin()
                              : sensitivityTimeTrajectoriesStock[i].begin() + postEventIndicesStock[i][j - 1];
      endTimeItr =
          (j == NE) ? sensitivityTimeTrajectoriesStock[i].end() : sensitivityTimeTrajectoriesStock[i].begin() + postEventIndicesStock[i][j];

      // if it should be integrated
      if (endTimeItr != beginTimeItr) {
        // finding the current active subsystem
        scalar_t midTime = 0.5 * (*beginTimeItr + *(endTimeItr - 1));
        auto activeSubsystem = static_cast<size_t>(lookup::findIndexInTimeArray(eventTimes_, midTime));

        // compute multiplier of the equivalent system
        scalar_t multiplier;
        computeEquivalentSystemMultiplier(eventTimeIndex, activeSubsystem, multiplier);
        rolloutSensitivityEquationsPtrStock_[workerIndex]->setMultiplier(multiplier);

        Observer<STATE_DIM> observer(&sensitivityStateTrajectoriesStock[i]);  // concat trajectory

        // solve sensitivity ODE
        rolloutSensitivityIntegratorsPtrStock_[workerIndex]->integrate_times(
            *rolloutSensitivityEquationsPtrStock_[workerIndex], observer, nabla_xInit, beginTimeItr, endTimeItr, gddpSettings_.timeStep_,
            gddpSettings_.absTolODE_, gddpSettings_.relTolODE_, maxNumSteps);

        // compute input sensitivity
        for (; k_u < sensitivityStateTrajectoriesStock[i].size(); k_u++) {
          sensitivityInputTrajectoriesStock[i].emplace_back(
              rolloutSensitivityEquationsPtrStock_[workerIndex]->controllerPtr()->computeInput(sensitivityTimeTrajectoriesStock[i][k_u],
                                                                                               sensitivityStateTrajectoriesStock[i][k_u]));
        }  // end of k loop
      }

      // compute jump map
      if (j < NE) {
        nabla_xInit = sensitivityStateTrajectoriesStock[i].back();
      }

    }  // end of j loop

    // reset the initial state
    nabla_xInit = sensitivityStateTrajectoriesStock[i].back();

  }  // end of i loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void GDDP<STATE_DIM, INPUT_DIM>::approximateNominalLQPSensitivity2EventTime(
    const state_vector_array2_t& sensitivityStateTrajectoriesStock, const input_vector_array2_t& sensitivityInputTrajectoriesStock,
    scalar_array2_t& nablaqTrajectoriesStock, state_vector_array2_t& nablaQvTrajectoriesStock,
    input_vector_array2_t& nablaRvTrajectoriesStock, scalar_array2_t& nablaqFinalStock, state_vector_array2_t& nablaQvFinalStock) const {
  // resizing
  nablaqTrajectoriesStock.resize(numPartitions_);
  nablaQvTrajectoriesStock.resize(numPartitions_);
  nablaRvTrajectoriesStock.resize(numPartitions_);
  nablaqFinalStock.resize(numPartitions_);
  nablaQvFinalStock.resize(numPartitions_);

  for (size_t i = 0; i < numPartitions_; i++) {
    // skip inactive partitions
    if (i < dataCollectorPtr_->initActivePartition_ || i > dataCollectorPtr_->finalActivePartition_) {
      nablaqTrajectoriesStock[i].clear();
      nablaQvTrajectoriesStock[i].clear();
      nablaRvTrajectoriesStock[i].clear();
      nablaqFinalStock[i].clear();
      nablaQvFinalStock[i].clear();
      continue;
    }

    const int N = dataCollectorPtr_->nominalTimeTrajectoriesStock_[i].size();
    const int NE = dataCollectorPtr_->nominalPostEventIndicesStock_[i].size();
    auto postEventIndexItr = dataCollectorPtr_->nominalPostEventIndicesStock_[i].begin();

    // resizing
    nablaqTrajectoriesStock[i].resize(N);
    nablaQvTrajectoriesStock[i].resize(N);
    nablaRvTrajectoriesStock[i].resize(N);
    nablaqFinalStock[i].resize(NE);
    nablaQvFinalStock[i].resize(NE);

    for (int k = 0; k < N; k++) {
      const auto& Qv = dataCollectorPtr_->modelDataTrajectoriesStock_[i][k].costStateDerivative_;
      const auto& Rv = dataCollectorPtr_->modelDataTrajectoriesStock_[i][k].costInputDerivative_;
      const auto& Qm = dataCollectorPtr_->modelDataTrajectoriesStock_[i][k].costStateSecondDerivative_;
      const auto& Rm = dataCollectorPtr_->modelDataTrajectoriesStock_[i][k].costInputSecondDerivative_;
      const auto& Pm = dataCollectorPtr_->modelDataTrajectoriesStock_[i][k].costInputStateDerivative_;

      nablaqTrajectoriesStock[i][k] = Qv.dot(sensitivityStateTrajectoriesStock[i][k]) + Rv.dot(sensitivityInputTrajectoriesStock[i][k]);
      nablaQvTrajectoriesStock[i][k] =
          Qm * sensitivityStateTrajectoriesStock[i][k] + Pm.transpose() * sensitivityInputTrajectoriesStock[i][k];
      nablaRvTrajectoriesStock[i][k] = Pm * sensitivityStateTrajectoriesStock[i][k] + Rm * sensitivityInputTrajectoriesStock[i][k];

      // terminal cost sensitivity to switching times
      if (postEventIndexItr != dataCollectorPtr_->nominalPostEventIndicesStock_[i].end() && k + 1 == *postEventIndexItr) {
        const size_t eventIndex = postEventIndexItr - dataCollectorPtr_->nominalPostEventIndicesStock_[i].begin();
        const size_t timeIndex = *postEventIndexItr - 1;
        const auto& QvFinal = dataCollectorPtr_->modelDataEventTimesStock_[i][eventIndex].costStateDerivative_;
        const auto& QmFinal = dataCollectorPtr_->modelDataEventTimesStock_[i][eventIndex].costStateSecondDerivative_;

        nablaqFinalStock[i][eventIndex] = QvFinal.dot(sensitivityStateTrajectoriesStock[i][timeIndex]);
        nablaQvFinalStock[i][eventIndex] = QmFinal * sensitivityStateTrajectoriesStock[i][timeIndex];

        postEventIndexItr++;
      }
    }  // end of k loop
  }    // end of i loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void GDDP<STATE_DIM, INPUT_DIM>::approximateNominalHeuristicsSensitivity2EventTime(const state_vector_t& sensitivityFinalState,
                                                                                   scalar_t& nablasHeuristics,
                                                                                   state_vector_t& nablaSvHeuristics) const {
  nablasHeuristics = dataCollectorPtr_->SvHeuristics_.dot(sensitivityFinalState);
  nablaSvHeuristics = dataCollectorPtr_->SmHeuristics_ * sensitivityFinalState;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void GDDP<STATE_DIM, INPUT_DIM>::solveSensitivityRiccatiEquations(
    size_t workerIndex, const size_t& eventTimeIndex, const scalar_t& learningRate, const scalar_t& nablasHeuristics,
    const state_vector_t& nablaSvHeuristics, const state_matrix_t& nablaSmHeuristics, scalar_array2_t& nablasTrajectoriesStock,
    state_vector_array2_t& nablaSvTrajectoriesStock, state_matrix_array2_t& nablaSmTrajectoriesStock) {
  if (eventTimeIndex < activeEventTimeBeginIndex_ || eventTimeIndex >= activeEventTimeEndIndex_) {
    throw std::runtime_error("The index is associated to an inactive event or it is out of range.");
  }

  using s_vector_t = typename riccati_sensitivity_equations_t::s_vector_t;
  using s_vector_array_t = typename riccati_sensitivity_equations_t::s_vector_array_t;

  // Resizing
  nablasTrajectoriesStock.resize(numPartitions_);
  nablaSvTrajectoriesStock.resize(numPartitions_);
  nablaSmTrajectoriesStock.resize(numPartitions_);

  // temporal final value for the last Riccati equations
  s_vector_t SsFinal;
  riccati_sensitivity_equations_t::convert2Vector(nablaSmHeuristics, nablaSvHeuristics, nablasHeuristics, SsFinal);
  // output containers which is reverse
  s_vector_array_t allSsTrajectory;

  for (int i = numEventTimes_; i >= 0; i--) {
    // skip inactive partitions
    if (i < dataCollectorPtr_->initActivePartition_ || i > dataCollectorPtr_->finalActivePartition_) {
      nablasTrajectoriesStock[i].clear();
      nablaSvTrajectoriesStock[i].clear();
      nablaSmTrajectoriesStock[i].clear();
      continue;
    }

    const int NS = dataCollectorPtr_->SsNormalizedTimeTrajectoriesStock_[i].size();
    const int NE = dataCollectorPtr_->SsNormalizedEventsPastTheEndIndecesStock_[i].size();

    // set data for Riccati sensitivity equations
    riccatiSensitivityEquationsPtrStock_[workerIndex]->resetNumFunctionCalls();
    riccatiSensitivityEquationsPtrStock_[workerIndex]->setData(
        learningRate, &dataCollectorPtr_->SsTimeTrajectoriesStock_[i], &dataCollectorPtr_->SmTrajectoriesStock_[i],
        &dataCollectorPtr_->SvTrajectoriesStock_[i], &dataCollectorPtr_->nominalTimeTrajectoriesStock_[i],
        &dataCollectorPtr_->modelDataTrajectoriesStock_[i], &dataCollectorPtr_->RmInverseTrajectoriesStock_[i],
        &nablaqTrajectoriesStockSet_[eventTimeIndex][i], &nablaQvTrajectoriesStockSet_[eventTimeIndex][i],
        &nablaRvTrajectoriesStockSet_[eventTimeIndex][i]);

    // max number of steps of integration
    const size_t maxNumSteps =
        gddpSettings_.maxNumStepsPerSecond_ * std::max(1.0, dataCollectorPtr_->SsNormalizedTimeTrajectoriesStock_[i].back() -
                                                                dataCollectorPtr_->SsNormalizedTimeTrajectoriesStock_[i].front());

    // output containers resizing
    allSsTrajectory.clear();
    allSsTrajectory.reserve(NS);

    // normalized switching times
    size_array_t SsNormalizedSwitchingTimesIndices;
    SsNormalizedSwitchingTimesIndices.reserve(NE + 2);
    SsNormalizedSwitchingTimesIndices.push_back(0);
    for (int k = 0; k < NE; k++) {
      const size_t& index = dataCollectorPtr_->SsNormalizedEventsPastTheEndIndecesStock_[i][k];
      SsNormalizedSwitchingTimesIndices.push_back(index);
    }
    SsNormalizedSwitchingTimesIndices.push_back(NS);

    // integrating the Riccati sensitivity equations
    typename scalar_array_t::const_iterator beginTimeItr, endTimeItr;
    for (int j = 0; j <= NE; j++) {
      beginTimeItr = dataCollectorPtr_->SsNormalizedTimeTrajectoriesStock_[i].begin() + SsNormalizedSwitchingTimesIndices[j];
      endTimeItr = dataCollectorPtr_->SsNormalizedTimeTrajectoriesStock_[i].begin() + SsNormalizedSwitchingTimesIndices[j + 1];

      // finding the current active subsystem
      scalar_t midNormalizedTime = 0.5 * (*beginTimeItr + *(endTimeItr - 1));
      scalar_t midTime = -midNormalizedTime;
      auto activeSubsystem = static_cast<size_t>(lookup::findIndexInTimeArray(eventTimes_, midTime));

      // compute multiplier of the equivalent system
      scalar_t multiplier;
      computeEquivalentSystemMultiplier(eventTimeIndex, activeSubsystem, multiplier);
      riccatiSensitivityEquationsPtrStock_[workerIndex]->setMultiplier(multiplier);

      Observer<riccati_sensitivity_equations_t::S_DIM_> observer(&allSsTrajectory);  // concatenate trajectory

      // solve Riccati sensitivity equations
      riccatiSensitivityIntegratorsPtrStock_[workerIndex]->integrate_times(*riccatiSensitivityEquationsPtrStock_[workerIndex], observer,
                                                                           SsFinal, beginTimeItr, endTimeItr, gddpSettings_.timeStep_,
                                                                           gddpSettings_.absTolODE_, gddpSettings_.relTolODE_, maxNumSteps);

      // final value of the next subsystem
      if (j < NE) {
        SsFinal = allSsTrajectory.back();

        s_vector_t SsFinalTemp = s_vector_t::Zero();
        riccati_sensitivity_equations_t::convert2Vector(state_matrix_t::Zero(), nablaQvFinalStockSet_[eventTimeIndex][i][NE - 1 - j],
                                                        nablaqFinalStockSet_[eventTimeIndex][i][NE - 1 - j], SsFinalTemp);

        SsFinal += SsFinalTemp;
      }

    }  // end of j loop

    // final value of the next partition
    SsFinal = allSsTrajectory.back();

    // check size
    if (allSsTrajectory.size() != NS) {
      throw std::runtime_error("allSsTrajectory size is incorrect.");
    }

    // construct 'nable_Sm', 'nable_Sv', and 'nable_s'
    nablasTrajectoriesStock[i].resize(NS);
    nablaSvTrajectoriesStock[i].resize(NS);
    nablaSmTrajectoriesStock[i].resize(NS);
    for (size_t k = 0; k < NS; k++) {
      riccati_sensitivity_equations_t::convert2Matrix(allSsTrajectory[NS - 1 - k], nablaSmTrajectoriesStock[i][k],
                                                      nablaSvTrajectoriesStock[i][k], nablasTrajectoriesStock[i][k]);
    }  // end of k loop

  }  // end of i loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void GDDP<STATE_DIM, INPUT_DIM>::solveSensitivityBVP(size_t workerIndex, const size_t& eventTimeIndex, const state_vector_t& MvFinal,
                                                     const state_vector_t& MveFinal, state_vector_array2_t& MvTrajectoriesStock,
                                                     state_vector_array2_t& MveTrajectoriesStock) {
  if (eventTimeIndex < activeEventTimeBeginIndex_ || eventTimeIndex >= activeEventTimeEndIndex_) {
    throw std::runtime_error("The index is associated to an inactive event or it is out of range.");
  }

  // resizing
  MvTrajectoriesStock.resize(numPartitions_);
  MveTrajectoriesStock.resize(numPartitions_);

  // temporal final value for the last Riccati equations
  state_vector_t MvFinalInternal = MvFinal;
  state_vector_t MveFinalInternal = MveFinal;
  // output containers which is a reverse container
  state_vector_array_t rMvTrajectory;
  state_vector_array_t rMveTrajectory;

  for (int i = numPartitions_ - 1; i >= 0; i--) {
    // skip inactive partitions
    if (i < dataCollectorPtr_->initActivePartition_ || i > dataCollectorPtr_->finalActivePartition_) {
      MvTrajectoriesStock[i].clear();
      MveTrajectoriesStock[i].clear();
      continue;
    }

    // set data for Riccati equations
    bvpSensitivityEquationsPtrStock_[workerIndex]->resetNumFunctionCalls();
    bvpSensitivityEquationsPtrStock_[workerIndex]->setData(
        &dataCollectorPtr_->nominalTimeTrajectoriesStock_[i], &dataCollectorPtr_->modelDataTrajectoriesStock_[i],
        &dataCollectorPtr_->projectedModelDataTrajectoriesStock_[i], &nominalCostateTrajectoriesStock_[i],
        &nominalLagrangianTrajectoriesStock_[i], &dataCollectorPtr_->optimizedControllersStock_[i].timeStamp_,
        &dataCollectorPtr_->optimizedControllersStock_[i].gainArray_, &dataCollectorPtr_->SmTrajectoriesStock_[i]);

    // set data for Riccati error equations
    bvpSensitivityErrorEquationsPtrStock_[workerIndex]->resetNumFunctionCalls();
    bvpSensitivityErrorEquationsPtrStock_[workerIndex]->setData(
        &dataCollectorPtr_->nominalTimeTrajectoriesStock_[i], &dataCollectorPtr_->modelDataTrajectoriesStock_[i],
        &dataCollectorPtr_->projectedModelDataTrajectoriesStock_[i], &dataCollectorPtr_->RmInverseTrajectoriesStock_[i],
        &dataCollectorPtr_->RmInvConstrainedCholTrajectoryStock_[i],
        &dataCollectorPtr_->EvDevEventTimesProjectedTrajectoriesStockSet_[eventTimeIndex][i],
        &dataCollectorPtr_->SsTimeTrajectoriesStock_[i], &dataCollectorPtr_->SmTrajectoriesStock_[i]);

    const int NS = dataCollectorPtr_->SsNormalizedTimeTrajectoriesStock_[i].size();
    const int NE = dataCollectorPtr_->SsNormalizedEventsPastTheEndIndecesStock_[i].size();

    // max number of steps of integration
    const size_t maxNumSteps =
        gddpSettings_.maxNumStepsPerSecond_ * std::max(1.0, dataCollectorPtr_->SsNormalizedTimeTrajectoriesStock_[i].back() -
                                                                dataCollectorPtr_->SsNormalizedTimeTrajectoriesStock_[i].front());

    // output containers resizing
    rMvTrajectory.clear();
    rMvTrajectory.reserve(NS);
    rMveTrajectory.clear();
    rMveTrajectory.reserve(NS);

    // normalized switching times
    size_array_t SsNormalizedSwitchingTimesIndices;
    SsNormalizedSwitchingTimesIndices.reserve(NE + 2);
    SsNormalizedSwitchingTimesIndices.push_back(0);
    for (int k = 0; k < NE; k++) {
      const auto& index = dataCollectorPtr_->SsNormalizedEventsPastTheEndIndecesStock_[i][k];
      SsNormalizedSwitchingTimesIndices.push_back(index);
    }
    SsNormalizedSwitchingTimesIndices.push_back(NS);

    // integrating the Riccati equations
    typename scalar_array_t::const_iterator beginTimeItr, endTimeItr;
    for (int j = 0; j <= NE; j++) {
      beginTimeItr = dataCollectorPtr_->SsNormalizedTimeTrajectoriesStock_[i].begin() + SsNormalizedSwitchingTimesIndices[j];
      endTimeItr = dataCollectorPtr_->SsNormalizedTimeTrajectoriesStock_[i].begin() + SsNormalizedSwitchingTimesIndices[j + 1];

      // finding the current active subsystem
      scalar_t midNormalizedTime = 0.5 * (*beginTimeItr + *(endTimeItr - 1));
      scalar_t midTime = -midNormalizedTime;
      auto activeSubsystem = static_cast<size_t>(lookup::findIndexInTimeArray(eventTimes_, midTime));

      // compute multiplier of the equivalent system
      scalar_t multiplier;
      computeEquivalentSystemMultiplier(eventTimeIndex, activeSubsystem, multiplier);
      bvpSensitivityEquationsPtrStock_[workerIndex]->setMultiplier(multiplier);

      Observer<STATE_DIM> rMvObserver(&rMvTrajectory);  // concatenate trajectory
      // solve Riccati equations for Mv
      bvpSensitivityIntegratorsPtrStock_[workerIndex]->integrate_times(*bvpSensitivityEquationsPtrStock_[workerIndex], rMvObserver,
                                                                       MvFinalInternal, beginTimeItr, endTimeItr, gddpSettings_.timeStep_,
                                                                       gddpSettings_.absTolODE_, gddpSettings_.relTolODE_, maxNumSteps);

      Observer<STATE_DIM> rMveObserver(&rMveTrajectory);  // concatenate trajectory
      // solve Riccati equations for Mve
      bvpSensitivityErrorIntegratorsPtrStock_[workerIndex]->integrate_times(
          *bvpSensitivityErrorEquationsPtrStock_[workerIndex], rMveObserver, MveFinalInternal, beginTimeItr, endTimeItr,
          gddpSettings_.timeStep_, gddpSettings_.absTolODE_, gddpSettings_.relTolODE_, maxNumSteps);

      // final value of the next subsystem
      if (j < NE) {
        MvFinalInternal = rMvTrajectory.back();
        // MvFinalInternal += dataCollectorPtr_->QvFinalStock_[i][NE-1-j];
        MveFinalInternal = rMveTrajectory.back();
      }
    }  // end of j loop

    // final value of the next partition
    MvFinalInternal = rMvTrajectory.back();
    MveFinalInternal = rMveTrajectory.back();

    // check sizes
    if (rMvTrajectory.size() != NS) {
      throw std::runtime_error("MvTrajectory size is incorrect.");
    }
    if (rMveTrajectory.size() != NS) {
      throw std::runtime_error("MveTrajectory size is incorrect.");
    }

    // constructing 'Mv' and 'Mve'
    MvTrajectoriesStock[i].resize(NS);
    std::reverse_copy(rMvTrajectory.begin(), rMvTrajectory.end(), MvTrajectoriesStock[i].begin());
    MveTrajectoriesStock[i].resize(NS);
    std::reverse_copy(rMveTrajectory.begin(), rMveTrajectory.end(), MveTrajectoriesStock[i].begin());

    // testing the numerical stability of the Riccati equations
    if (gddpSettings_.checkNumericalStability_) {
      for (int k = NS - 1; k >= 0; k--) {
        try {
          if (!MvTrajectoriesStock[i][k].allFinite()) {
            throw std::runtime_error("Mv is unstable.");
          }
          if (!MveTrajectoriesStock[i][k].allFinite()) {
            throw std::runtime_error("Mve is unstable.");
          }

        } catch (const std::exception& error) {
          std::cerr << "what(): " << error.what() << " at time " << dataCollectorPtr_->SsTimeTrajectoriesStock_[i][k] << " [sec]."
                    << std::endl;
          for (int kp = k; kp < k + 10; kp++) {
            if (kp >= NS) {
              continue;
            }
            std::cerr << "Mv[" << dataCollectorPtr_->SsTimeTrajectoriesStock_[i][kp] << "]:\t"
                      << MvTrajectoriesStock[i][kp].transpose().norm() << std::endl;
            std::cerr << "Mve[" << dataCollectorPtr_->SsTimeTrajectoriesStock_[i][kp] << "]:\t"
                      << MveTrajectoriesStock[i][kp].transpose().norm() << std::endl;
          }
          throw;
        }
      }  // end of k loop
    }

  }  // end of i loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void GDDP<STATE_DIM, INPUT_DIM>::calculateLQSensitivityControllerForward(size_t workerIndex, const size_t& eventTimeIndex,
                                                                         const scalar_array2_t& timeTrajectoriesStock,
                                                                         const state_vector_array2_t& nablaSvTrajectoriesStock,
                                                                         input_vector_array2_t& nablaLvTrajectoriesStock) {
  if (eventTimeIndex < activeEventTimeBeginIndex_ || eventTimeIndex >= activeEventTimeEndIndex_) {
    throw std::runtime_error("The index is associated to an inactive event or it is out of range.");
  }

  // resizing
  nablaLvTrajectoriesStock.resize(numPartitions_);

  for (size_t i = 0; i < numEventTimes_ + 1; i++) {
    // skip inactive partitions
    if (i < dataCollectorPtr_->initActivePartition_ || i > dataCollectorPtr_->finalActivePartition_) {
      nablaLvTrajectoriesStock[i].clear();
      continue;
    }

    // resizing
    const size_t N = nablaSvTrajectoriesStock[i].size();
    nablaLvTrajectoriesStock[i].resize(N);

    for (size_t k = 0; k < N; k++) {
      // time
      const scalar_t& t = timeTrajectoriesStock[i][k];
      auto indexAlpha = LinearInterpolation::timeSegment(t, &dataCollectorPtr_->nominalTimeTrajectoriesStock_[i]);
      // Bm
      dynamic_matrix_t Bm;
      ModelData::interpolate(indexAlpha, Bm, &dataCollectorPtr_->modelDataTrajectoriesStock_[i], ModelData::dynamicsInputDerivative);
      // RmInverse
      dynamic_matrix_t RmInverse;
      LinearInterpolation::interpolate(indexAlpha, RmInverse, &dataCollectorPtr_->RmInverseTrajectoriesStock_[i]);
      // nablaRv
      input_vector_t nablaRv;
      LinearInterpolation::interpolate(indexAlpha, nablaRv, &nablaRvTrajectoriesStockSet_[eventTimeIndex][i]);

      nablaLvTrajectoriesStock[i][k] = -RmInverse * (nablaRv + Bm.transpose() * nablaSvTrajectoriesStock[i][k]);
    }  // end of k loop
  }    // end of i loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void GDDP<STATE_DIM, INPUT_DIM>::calculateBVPSensitivityControllerForward(size_t workerIndex, const size_t& eventTimeIndex,
                                                                          const scalar_array2_t& timeTrajectoriesStock,
                                                                          const state_vector_array2_t& MvTrajectoriesStock,
                                                                          const state_vector_array2_t& MveTrajectoriesStock,
                                                                          input_vector_array2_t& LvTrajectoriesStock) {
  if (eventTimeIndex < activeEventTimeBeginIndex_ || eventTimeIndex >= activeEventTimeEndIndex_) {
    throw std::runtime_error("The index is associated to an inactive event or it is out of range.");
  }

  // resizing
  LvTrajectoriesStock.resize(numPartitions_);

  for (size_t i = 0; i < numPartitions_; i++) {
    if (i < dataCollectorPtr_->initActivePartition_ || i > dataCollectorPtr_->finalActivePartition_) {
      LvTrajectoriesStock[i].clear();
      continue;
    }

    const size_t N = timeTrajectoriesStock[i].size();
    LvTrajectoriesStock[i].resize(N);
    for (size_t k = 0; k < N; k++) {
      // time
      const scalar_t& t = timeTrajectoriesStock[i][k];
      auto indexAlpha = LinearInterpolation::timeSegment(t, &dataCollectorPtr_->nominalTimeTrajectoriesStock_[i]);
      // Bm
      dynamic_matrix_t Bm;
      ModelData::interpolate(indexAlpha, Bm, &(dataCollectorPtr_->modelDataTrajectoriesStock_[i]), ModelData::dynamicsInputDerivative);
      // RmInverse
      dynamic_matrix_t RmInverse;
      LinearInterpolation::interpolate(indexAlpha, RmInverse, &dataCollectorPtr_->RmInverseTrajectoriesStock_[i]);
      // DmProjected
      dynamic_matrix_t DmProjected;
      ModelData::interpolate(indexAlpha, DmProjected, &(dataCollectorPtr_->projectedModelDataTrajectoriesStock_[i]),
                             ModelData::stateInputEqConstrInputDerivative);
      // EvDevEventTimesProjected
      input_vector_t EvDevEventTimeProjected;
      LinearInterpolation::interpolate(indexAlpha, EvDevEventTimeProjected,
                                       &dataCollectorPtr_->EvDevEventTimesProjectedTrajectoriesStockSet_[eventTimeIndex][i]);

      LvTrajectoriesStock[i][k] = -(dynamic_matrix_t::Identity(INPUT_DIM, INPUT_DIM) - DmProjected) * RmInverse * Bm.transpose() *
                                      (MvTrajectoriesStock[i][k] + MveTrajectoriesStock[i][k]) -
                                  EvDevEventTimeProjected;
    }  // end of k loop
  }    // end of i loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void GDDP<STATE_DIM, INPUT_DIM>::getValueFuntionSensitivity(const size_t& eventTimeIndex, const scalar_t& time, const state_vector_t& state,
                                                            scalar_t& valueFunctionDerivative) {
  if (eventTimeIndex < activeEventTimeBeginIndex_ || eventTimeIndex >= activeEventTimeEndIndex_) {
    throw std::runtime_error("The index is associated to an inactive event or it is out of range.");
  }

  auto activePartition = static_cast<size_t>(lookup::findBoundedActiveIntervalInTimeArray(dataCollectorPtr_->partitioningTimes_, time));

  state_vector_t nominalState;
  LinearInterpolation::interpolate(time, nominalState, &dataCollectorPtr_->nominalTimeTrajectoriesStock_[activePartition],
                                   &dataCollectorPtr_->nominalStateTrajectoriesStock_[activePartition]);
  state_vector_t deltsState = state - nominalState;

  const auto indexAlpha = LinearInterpolation::timeSegment(time, &dataCollectorPtr_->SsTimeTrajectoriesStock_[activePartition]);

  scalar_t nablas;
  LinearInterpolation::interpolate(indexAlpha, nablas, &nablasTrajectoriesStockSet_[eventTimeIndex][activePartition]);

  state_vector_t nablaSv;
  LinearInterpolation::interpolate(indexAlpha, nablaSv, &nablaSvTrajectoriesStockSet_[eventTimeIndex][activePartition]);

  state_matrix_t nablaSm;
  LinearInterpolation::interpolate(indexAlpha, nablaSm, &nablaSmTrajectoriesStockSet_[eventTimeIndex][activePartition]);

  valueFunctionDerivative = nablas + deltsState.dot(nablaSv) + 0.5 * deltsState.dot(nablaSm * deltsState);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void GDDP<STATE_DIM, INPUT_DIM>::calculateCostDerivative(size_t workerIndex, const size_t& eventTimeIndex,
                                                         const state_vector_array2_t& sensitivityStateTrajectoriesStock,
                                                         const input_vector_array2_t& sensitivityInputTrajectoriesStock,
                                                         scalar_t& costDerivative) const {
  if (eventTimeIndex < activeEventTimeBeginIndex_ || eventTimeIndex >= activeEventTimeEndIndex_) {
    throw std::runtime_error("The index is associated to an inactive event or it is out of range.");
  }

  costDerivative = 0.0;
  scalar_t prevIntermediatecostDev = 0.0;
  scalar_t currIntermediatecostDev = 0.0;
  size_t beginIndex, endIndex;

  for (size_t i = dataCollectorPtr_->initActivePartition_; i <= dataCollectorPtr_->finalActivePartition_; i++) {
    const int N = dataCollectorPtr_->nominalTimeTrajectoriesStock_[i].size();
    const int NE = dataCollectorPtr_->nominalPostEventIndicesStock_[i].size();

    for (int j = 0; j <= NE; j++) {
      beginIndex = (j == 0) ? 0 : dataCollectorPtr_->nominalPostEventIndicesStock_[i][j - 1];
      endIndex = (j == NE) ? N : dataCollectorPtr_->nominalPostEventIndicesStock_[i][j];

      // integrates the intermediate cost sensitivity using the trapezoidal approximation method
      if (beginIndex != endIndex) {
        // finding the current active subsystem
        scalar_t midTime = 0.5 * (dataCollectorPtr_->nominalTimeTrajectoriesStock_[i][beginIndex] +
                                  dataCollectorPtr_->nominalTimeTrajectoriesStock_[i][endIndex - 1]);
        auto activeSubsystem = static_cast<size_t>(lookup::findIndexInTimeArray(eventTimes_, midTime));

        // compute multiplier of the equivalent system
        scalar_t multiplier;
        computeEquivalentSystemMultiplier(eventTimeIndex, activeSubsystem, multiplier);

        for (size_t k = beginIndex; k < endIndex; k++) {
          if (k > beginIndex) {
            prevIntermediatecostDev = currIntermediatecostDev;
          }
          const auto& q = dataCollectorPtr_->modelDataTrajectoriesStock_[i][k].cost_;
          const auto& Qv = dataCollectorPtr_->modelDataTrajectoriesStock_[i][k].costStateDerivative_;
          const auto& Rv = dataCollectorPtr_->modelDataTrajectoriesStock_[i][k].costInputDerivative_;
          currIntermediatecostDev =
              multiplier * q + Qv.dot(sensitivityStateTrajectoriesStock[i][k]) + Rv.dot(sensitivityInputTrajectoriesStock[i][k]);

          if (k > beginIndex) {
            costDerivative +=
                0.5 *
                (dataCollectorPtr_->nominalTimeTrajectoriesStock_[i][k] - dataCollectorPtr_->nominalTimeTrajectoriesStock_[i][k - 1]) *
                (currIntermediatecostDev + prevIntermediatecostDev);
          }
        }  // end of k loop
      }

      // cost sensitivity at switching times
      if (j < NE) {
        const auto& QvFinal = dataCollectorPtr_->modelDataEventTimesStock_[i][j].costStateDerivative_;
        costDerivative += sensitivityStateTrajectoriesStock[i].back().dot(QvFinal);
      }
    }  // end of j loop

  }  // end of i loop

  // add the Heuristics function sensitivity at the final time
  costDerivative +=
      sensitivityStateTrajectoriesStock[dataCollectorPtr_->finalActivePartition_].back().dot(dataCollectorPtr_->SvHeuristics_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void GDDP<STATE_DIM, INPUT_DIM>::runLQBasedMethod() {
  // display
  if (gddpSettings_.displayInfo_) {
    std::cerr << "LQ-based method is used for computing the gradient." << std::endl;
  }

  // resizing
  nablaLvTrajectoriesStockSet_.resize(numEventTimes_);
  sensitivityStateTrajectoriesStockSet_.resize(numEventTimes_);
  sensitivityInputTrajectoriesStockSet_.resize(numEventTimes_);
  nablaqTrajectoriesStockSet_.resize(numEventTimes_);
  nablaQvTrajectoriesStockSet_.resize(numEventTimes_);
  nablaRvTrajectoriesStockSet_.resize(numEventTimes_);
  nablaqFinalStockSet_.resize(numEventTimes_);
  nablaQvFinalStockSet_.resize(numEventTimes_);
  nablasHeuristics_.resize(numEventTimes_);
  nablaSvHeuristics_.resize(numEventTimes_);
  nablasTrajectoriesStockSet_.resize(numEventTimes_);
  nablaSvTrajectoriesStockSet_.resize(numEventTimes_);
  nablaSmTrajectoriesStockSet_.resize(numEventTimes_);
  nominalCostFuntionDerivative_.resize(numEventTimes_);

  size_t iteration = 0;
  while (iteration++ < gddpSettings_.maxNumIterationForLQ_) {
    // for each active event time
    for (size_t index = 0; index < numEventTimes_; index++) {
      if (activeEventTimeBeginIndex_ <= index && index < activeEventTimeEndIndex_) {
        // for the first iteration set Lv to zero
        if (iteration == 1) {
          nablaLvTrajectoriesStockSet_[index].resize(numPartitions_);
          for (size_t i = dataCollectorPtr_->initActivePartition_; i <= dataCollectorPtr_->finalActivePartition_; i++) {
            nablaLvTrajectoriesStockSet_[index][i] =
                input_vector_array_t(dataCollectorPtr_->optimizedControllersStock_[i].timeStamp_.size(), input_vector_t::Zero());
          }
        }

        const size_t workerIndex = 0;

        // calculate rollout sensitivity to event times
        propagateRolloutSensitivity(workerIndex, index, dataCollectorPtr_->optimizedControllersStock_, nablaLvTrajectoriesStockSet_[index],
                                    dataCollectorPtr_->nominalTimeTrajectoriesStock_, dataCollectorPtr_->nominalPostEventIndicesStock_,
                                    sensitivityStateTrajectoriesStockSet_[index], sensitivityInputTrajectoriesStockSet_[index]);

        // approximate the nominal LQ sensitivity to switching times
        approximateNominalLQPSensitivity2EventTime(sensitivityStateTrajectoriesStockSet_[index],
                                                   sensitivityInputTrajectoriesStockSet_[index], nablaqTrajectoriesStockSet_[index],
                                                   nablaQvTrajectoriesStockSet_[index], nablaRvTrajectoriesStockSet_[index],
                                                   nablaqFinalStockSet_[index], nablaQvFinalStockSet_[index]);

        // approximate Heuristics
        approximateNominalHeuristicsSensitivity2EventTime(
            sensitivityStateTrajectoriesStockSet_[index][dataCollectorPtr_->finalActivePartition_].back(), nablasHeuristics_[index],
            nablaSvHeuristics_[index]);

        // solve Riccati equations
        // prevents the changes in the nominal trajectories and just update the gains
        const scalar_t learningRateStar = 0.0;
        solveSensitivityRiccatiEquations(workerIndex, index, learningRateStar, nablasHeuristics_[index], nablaSvHeuristics_[index],
                                         state_matrix_t::Zero(), nablasTrajectoriesStockSet_[index], nablaSvTrajectoriesStockSet_[index],
                                         nablaSmTrajectoriesStockSet_[index]);

        // calculate sensitivity controller feedforward part
        calculateLQSensitivityControllerForward(workerIndex, index, dataCollectorPtr_->SsTimeTrajectoriesStock_,
                                                nablaSvTrajectoriesStockSet_[index], nablaLvTrajectoriesStockSet_[index]);

        // calculate the value function derivatives w.r.t. event times
        getValueFuntionSensitivity(index, dataCollectorPtr_->initTime_, dataCollectorPtr_->initState_,
                                   nominalCostFuntionDerivative_(index));

      } else if (iteration == 1) {
        nablaLvTrajectoriesStockSet_[index].clear();
        sensitivityStateTrajectoriesStockSet_[index].clear();
        sensitivityInputTrajectoriesStockSet_[index].clear();
        nablaqTrajectoriesStockSet_[index].clear();
        nablaQvTrajectoriesStockSet_[index].clear();
        nablaRvTrajectoriesStockSet_[index].clear();
        nablaqFinalStockSet_[index].clear();
        nablaQvFinalStockSet_[index].clear();
        nablasTrajectoriesStockSet_[index].clear();
        nablaSvTrajectoriesStockSet_[index].clear();
        nablaSmTrajectoriesStockSet_[index].clear();
        nominalCostFuntionDerivative_(index) = 0.0;
      }
    }  // end of index loop

  }  // end of while loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void GDDP<STATE_DIM, INPUT_DIM>::runSweepingBVPMethod() {
  // display
  if (gddpSettings_.displayInfo_) {
    std::cerr << "BVP-based method is used for computing the gradient." << std::endl;
  }

  // calculate costate
  calculateRolloutCostate(dataCollectorPtr_->nominalTimeTrajectoriesStock_, nominalCostateTrajectoriesStock_);

  // calculate Lagrangian
  calculateNominalRolloutLagrangeMultiplier(dataCollectorPtr_->nominalTimeTrajectoriesStock_, nominalLagrangianTrajectoriesStock_);

  // resizing
  MvTrajectoriesStockSet_.resize(numEventTimes_);
  MveTrajectoriesStockSet_.resize(numEventTimes_);
  LvTrajectoriesStockSet_.resize(numEventTimes_);
  sensitivityStateTrajectoriesStockSet_.resize(numEventTimes_);
  sensitivityInputTrajectoriesStockSet_.resize(numEventTimes_);
  nominalCostFuntionDerivative_.resize(numEventTimes_);

  // for each active event time
  for (size_t index = 0; index < numEventTimes_; index++) {
    if (activeEventTimeBeginIndex_ <= index && index < activeEventTimeEndIndex_) {
      const size_t workerIndex = 0;

      // solve BVP to compute 'Mv' and 'Mve'
      solveSensitivityBVP(workerIndex, index, state_vector_t::Zero() /*dataCollectorPtr_->SvHeuristics_*/,
                          state_vector_t::Zero() /*SveHeuristics_*/, MvTrajectoriesStockSet_[index], MveTrajectoriesStockSet_[index]);

      // calculates sensitivity controller feedforward part, 'Lv'
      calculateBVPSensitivityControllerForward(workerIndex, index, dataCollectorPtr_->SsTimeTrajectoriesStock_,
                                               MvTrajectoriesStockSet_[index], MveTrajectoriesStockSet_[index],
                                               LvTrajectoriesStockSet_[index]);

      // calculate rollout sensitivity to event times
      propagateRolloutSensitivity(workerIndex, index, dataCollectorPtr_->optimizedControllersStock_, LvTrajectoriesStockSet_[index],
                                  dataCollectorPtr_->nominalTimeTrajectoriesStock_, dataCollectorPtr_->nominalPostEventIndicesStock_,
                                  sensitivityStateTrajectoriesStockSet_[index], sensitivityInputTrajectoriesStockSet_[index]);

      // calculate the cost function derivatives w.r.t. event times
      calculateCostDerivative(workerIndex, index, sensitivityStateTrajectoriesStockSet_[index],
                              sensitivityInputTrajectoriesStockSet_[index], nominalCostFuntionDerivative_(index));

    } else {
      MvTrajectoriesStockSet_[index].clear();
      MveTrajectoriesStockSet_[index].clear();
      LvTrajectoriesStockSet_[index].clear();
      sensitivityStateTrajectoriesStockSet_[index].clear();
      sensitivityInputTrajectoriesStockSet_[index].clear();
      nominalCostFuntionDerivative_(index) = 0.0;
    }

  }  // end of index
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void GDDP<STATE_DIM, INPUT_DIM>::run(const ddp_data_collector_t* dataCollectorPtr) {
  // display
  if (gddpSettings_.displayInfo_) {
    std::cerr << "\n++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
    std::cerr << "+++++++++++++++ GDDP is initialized ++++++++++++++++++" << std::endl;
    std::cerr << "++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
  }

  // data collector pointer
  dataCollectorPtr_ = dataCollectorPtr;

  // event time an number of event and subsystems
  eventTimes_ = dataCollectorPtr_->modeSchedule_.eventTimes;
  numEventTimes_ = eventTimes_.size();

  // update sizes if number of partitions has been changed
  if (numPartitions_ != dataCollectorPtr_->numPartitions_) {
    numPartitions_ = dataCollectorPtr_->numPartitions_;
    setupOptimizer(numPartitions_);
  }

  // find active event times range: [activeEventTimeBeginIndex_, activeEventTimeEndIndex_)
  activeEventTimeBeginIndex_ = static_cast<size_t>(lookup::findIndexInTimeArray(eventTimes_, dataCollectorPtr_->initTime_));
  activeEventTimeEndIndex_ = static_cast<size_t>(lookup::findIndexInTimeArray(eventTimes_, dataCollectorPtr_->finalTime_));

  // use the LQ-based method or Sweeping-BVP method
  if (gddpSettings_.useLQForDerivatives_) {
    runLQBasedMethod();
  } else {
    runSweepingBVPMethod();
  }

  if (gddpSettings_.displayInfo_) {
    std::cerr << "\n++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
    std::cerr << "++++++++++++++++ GDDP is terminated ++++++++++++++++++" << std::endl;
    std::cerr << "++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
    std::cerr << "GDDP gradient: " << nominalCostFuntionDerivative_.transpose() << std::endl;
    std::cerr << std::endl;
  }
}

}  // namespace ocs2
