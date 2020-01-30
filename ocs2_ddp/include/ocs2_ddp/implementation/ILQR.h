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
ILQR<STATE_DIM, INPUT_DIM>::ILQR(const rollout_base_t* rolloutPtr, const derivatives_base_t* systemDerivativesPtr,
                                 const constraint_base_t* systemConstraintsPtr, const cost_function_base_t* costFunctionPtr,
                                 const operating_trajectories_base_t* operatingTrajectoriesPtr,
                                 const ILQR_Settings& settings /*= ILQR_Settings()*/,
                                 std::shared_ptr<HybridLogicRules> logicRulesPtr /*= nullptr*/,
                                 const cost_function_base_t* heuristicsFunctionPtr /* = nullptr*/)

    : BASE(rolloutPtr, systemDerivativesPtr, systemConstraintsPtr, costFunctionPtr, operatingTrajectoriesPtr, settings.ddpSettings_,
           heuristicsFunctionPtr, "ILQR", logicRulesPtr),
      settings_(settings) {
  // Riccati Solver
  riccatiEquationsPtrStock_.clear();
  riccatiEquationsPtrStock_.reserve(BASE::ddpSettings_.nThreads_);

  for (size_t i = 0; i < BASE::ddpSettings_.nThreads_; i++) {
    riccatiEquationsPtrStock_.emplace_back(new riccati_equations_t);
  }  // end of i loop

  Eigen::initParallel();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void ILQR<STATE_DIM, INPUT_DIM>::approximateIntermediateLQ(const scalar_array_t& timeTrajectory, const size_array_t& postEventIndices,
                                                           const state_vector_array_t& stateTrajectory,
                                                           const input_vector_array_t& inputTrajectory,
                                                           ModelDataBase::array_t& modelDataTrajectory) {
  BASE::nextTimeIndex_ = 0;
  BASE::nextTaskId_ = 0;
  std::function<void(void)> task = [&] {
    size_t timeIndex;
    size_t taskId = BASE::nextTaskId_++;  // assign task ID (atomic)

    ModelDataBase continuousTimeModelData;

    // get next time index is atomic
    while ((timeIndex = BASE::nextTimeIndex_++) < timeTrajectory.size()) {
      // execute continuous time LQ approximation for the given partition and time index
      continuousTimeModelData = modelDataTrajectory[timeIndex];
      BASE::linearQuadraticApproximatorPtrStock_[taskId]->approximateUnconstrainedLQProblem(
          timeTrajectory[timeIndex], stateTrajectory[timeIndex], inputTrajectory[timeIndex], continuousTimeModelData);

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

  BASE::runParallel(task, BASE::ddpSettings_.nThreads_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void ILQR<STATE_DIM, INPUT_DIM>::discreteLQWorker(size_t workerIndex, scalar_t timeStep, const ModelDataBase& continuousTimeModelData,
                                                  ModelDataBase& modelData) {
  /*
   * linearize system dynamics
   */
  modelData.dynamicsStateDerivative_ =
      dynamic_matrix_t::Identity(STATE_DIM, STATE_DIM) + continuousTimeModelData.dynamicsStateDerivative_ * timeStep;
  modelData.dynamicsInputDerivative_ = continuousTimeModelData.dynamicsInputDerivative_ * timeStep;

  /*
   * quadratic approximation to the cost function
   */
  modelData.cost_ = continuousTimeModelData.cost_ * timeStep;
  modelData.costStateDerivative_ = continuousTimeModelData.costStateDerivative_ * timeStep;
  modelData.costStateSecondDerivative_ = continuousTimeModelData.costStateSecondDerivative_ * timeStep;
  modelData.costInputDerivative_ = continuousTimeModelData.costInputDerivative_ * timeStep;
  modelData.costInputSecondDerivative_ = continuousTimeModelData.costInputSecondDerivative_ * timeStep;
  modelData.costInputStateDerivative_ = continuousTimeModelData.costInputStateDerivative_ * timeStep;

  /*
   * linearize constraints
   */
  // state equality constraints
  modelData.numStateEqConstr_ = continuousTimeModelData.numStateEqConstr_;
  modelData.stateEqConstr_ = continuousTimeModelData.stateEqConstr_;
  modelData.stateEqConstrStateDerivative_ = continuousTimeModelData.stateEqConstrStateDerivative_;

  // state-input equality constraints
  modelData.numStateInputEqConstr_ = continuousTimeModelData.numStateInputEqConstr_;
  modelData.stateInputEqConstr_ = continuousTimeModelData.stateInputEqConstr_;
  modelData.stateInputEqConstrStateDerivative_ = continuousTimeModelData.stateInputEqConstrStateDerivative_;
  modelData.stateInputEqConstrInputDerivative_ = continuousTimeModelData.stateInputEqConstrInputDerivative_;

  // inequality constraints
  modelData.numIneqConstr_ = continuousTimeModelData.numIneqConstr_;
  modelData.ineqConstr_.resize(modelData.numIneqConstr_);
  modelData.ineqConstrStateDerivative_.resize(modelData.numIneqConstr_);
  modelData.ineqConstrInputDerivative_.resize(modelData.numIneqConstr_);
  modelData.ineqConstrStateSecondDerivative_.resize(modelData.numIneqConstr_);
  modelData.ineqConstrInputSecondDerivative_.resize(modelData.numIneqConstr_);
  modelData.ineqConstrInputStateDerivative_.resize(modelData.numIneqConstr_);
  for (size_t k = 0; k < modelData.numIneqConstr_; k++) {
    modelData.ineqConstr_[k] = continuousTimeModelData.ineqConstr_[k];
    modelData.ineqConstrStateDerivative_[k] = continuousTimeModelData.ineqConstrStateDerivative_[k];
    modelData.ineqConstrInputDerivative_[k] = continuousTimeModelData.ineqConstrInputDerivative_[k];
    modelData.ineqConstrStateSecondDerivative_[k] = continuousTimeModelData.ineqConstrStateSecondDerivative_[k];
    modelData.ineqConstrInputSecondDerivative_[k] = continuousTimeModelData.ineqConstrInputSecondDerivative_[k];
    modelData.ineqConstrInputStateDerivative_[k] = continuousTimeModelData.ineqConstrInputStateDerivative_[k];
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void ILQR<STATE_DIM, INPUT_DIM>::calculateController() {
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
template <size_t STATE_DIM, size_t INPUT_DIM>
void ILQR<STATE_DIM, INPUT_DIM>::calculateControllerWorker(size_t workerIndex, size_t partitionIndex, size_t timeIndex) {
  const auto i = partitionIndex;
  const auto k = timeIndex;

  const auto& nominalState = BASE::nominalStateTrajectoriesStock_[i][k];
  const auto& nominalInput = BASE::nominalInputTrajectoriesStock_[i][k];

  const auto& EvProjected = BASE::projectedModelDataTrajectoriesStock_[i][k].stateInputEqConstr_;
  const auto& CmProjected = BASE::projectedModelDataTrajectoriesStock_[i][k].stateInputEqConstrStateDerivative_;

  const auto& HmInverseConstrained = BASE::riccatiModificationTrajectoriesStock_[i][k].HmInverseConstrained_;

  // feedback gains
  BASE::nominalControllersStock_[i].gainArray_[k] = -CmProjected;
  BASE::nominalControllersStock_[i].gainArray_[k].noalias() -=
      HmInverseConstrained * (GmTrajectoriesStock_[i][k] + BASE::riccatiModificationTrajectoriesStock_[i][k].deltaPm_);

  // bias input
  BASE::nominalControllersStock_[i].deltaBiasArray_[k] = -EvProjected;
  BASE::nominalControllersStock_[i].deltaBiasArray_[k].noalias() -= HmInverseConstrained * GvTrajectoriesStock_[i][k];
  BASE::nominalControllersStock_[i].biasArray_[k] = nominalInput - BASE::nominalControllersStock_[i].gainArray_[k] * nominalState;

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
      std::cerr << "what(): " << error.what() << " at time " << BASE::nominalControllersStock_[i].timeStamp_[k] << " [sec]." << std::endl;
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <size_t STATE_DIM, size_t INPUT_DIM>
typename ILQR<STATE_DIM, INPUT_DIM>::scalar_t ILQR<STATE_DIM, INPUT_DIM>::solveSequentialRiccatiEquations(const state_matrix_t& SmFinal,
                                                                                                          const state_vector_t& SvFinal,
                                                                                                          const scalar_t& sFinal) {
  return BASE::solveSequentialRiccatiEquationsImpl(SmFinal, SvFinal, sFinal);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void ILQR<STATE_DIM, INPUT_DIM>::constrainedRiccatiEquationsWorker(size_t workerIndex, size_t partitionIndex,
                                                                   const dynamic_matrix_t& SmFinal, const dynamic_vector_t& SvFinal,
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

  GvTrajectoriesStock_[partitionIndex].resize(N);
  GmTrajectoriesStock_[partitionIndex].resize(N);

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
  state_matrix_t SmFinalTemp = SmFinal;
  state_vector_t SvFinalTemp = SvFinal;
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

    // continuous-time riccatiModification
    dynamic_matrix_t Hm = BASE::modelDataTrajectoriesStock_[partitionIndex][endTimeItr - 1].costInputSecondDerivative_;
    BASE::computeRiccatiModification(BASE::ddpSettings_.strategy_, Hm, BASE::modelDataTrajectoriesStock_[partitionIndex][endTimeItr - 1],
                                     BASE::riccatiModificationTrajectoriesStock_[partitionIndex][endTimeItr - 1]);

    // project LQ coefficients to constrained ones
    BASE::projectLQ(BASE::modelDataTrajectoriesStock_[partitionIndex][endTimeItr - 1],
                    BASE::riccatiModificationTrajectoriesStock_[partitionIndex][endTimeItr - 1].DmDagger_,
                    BASE::projectedModelDataTrajectoriesStock_[partitionIndex][endTimeItr - 1]);

    const ModelDataBase& md = BASE::projectedModelDataTrajectoriesStock_[partitionIndex][endTimeItr - 1];
    GvTrajectoriesStock_[partitionIndex][endTimeItr - 1] =
        md.costInputDerivative_ + md.dynamicsInputDerivative_.transpose() * BASE::SvTrajectoryStock_[partitionIndex][endTimeItr - 1];
    GmTrajectoriesStock_[partitionIndex][endTimeItr - 1] =
        md.costInputStateDerivative_ + BASE::riccatiModificationTrajectoriesStock_[partitionIndex][endTimeItr - 1].deltaPm_ +
        md.dynamicsInputDerivative_.transpose() * BASE::SmTrajectoryStock_[partitionIndex][endTimeItr - 1];

    /*
     * solve Riccati equations and compute projected model data and RiccatiModification for the intermediate times
     */
    dynamic_matrix_t Bm_T_Sm(INPUT_DIM, STATE_DIM);
    for (int k = endTimeItr - 2; k >= beginTimeItr; k--) {
      const auto& modelData = BASE::modelDataTrajectoriesStock_[partitionIndex][k];

      // Hm
      Hm = modelData.costInputSecondDerivative_;
      Bm_T_Sm.noalias() = modelData.dynamicsInputDerivative_.transpose() * BASE::SmTrajectoryStock_[partitionIndex][k + 1];
      Hm.noalias() += Bm_T_Sm * modelData.dynamicsInputDerivative_;

      // riccatiModification
      BASE::computeRiccatiModification(BASE::ddpSettings_.strategy_, Hm, modelData,
                                       BASE::riccatiModificationTrajectoriesStock_[partitionIndex][k]);

      // project LQ coefficients to constrained ones
      BASE::projectLQ(modelData, BASE::riccatiModificationTrajectoriesStock_[partitionIndex][k].DmDagger_,
                      BASE::projectedModelDataTrajectoriesStock_[partitionIndex][k]);

      // compute one step of Riccati difference equations
      riccatiEquationsPtrStock_[workerIndex]->computeMap(
          BASE::projectedModelDataTrajectoriesStock_[partitionIndex][k], BASE::riccatiModificationTrajectoriesStock_[partitionIndex][k],
          BASE::SmTrajectoryStock_[partitionIndex][k + 1], BASE::SvTrajectoryStock_[partitionIndex][k + 1],
          BASE::sTrajectoryStock_[partitionIndex][k + 1], GmTrajectoriesStock_[partitionIndex][k], GvTrajectoriesStock_[partitionIndex][k],
          BASE::SmTrajectoryStock_[partitionIndex][k], BASE::SvTrajectoryStock_[partitionIndex][k],
          BASE::sTrajectoryStock_[partitionIndex][k]);
    }  // end of k loop

    if (i > 0) {
      const auto& qFinal = BASE::modelDataEventTimesStock_[partitionIndex][i - 1].cost_;
      const auto& QvFinal = BASE::modelDataEventTimesStock_[partitionIndex][i - 1].costStateDerivative_;
      const auto& QmFinal = BASE::modelDataEventTimesStock_[partitionIndex][i - 1].costStateSecondDerivative_;
      sFinalTemp = BASE::sTrajectoryStock_[partitionIndex][beginTimeItr] + qFinal;
      SvFinalTemp = BASE::SvTrajectoryStock_[partitionIndex][beginTimeItr] + QvFinal;
      SmFinalTemp = BASE::SmTrajectoryStock_[partitionIndex][beginTimeItr] + QmFinal;
    }
  }  // end of i loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
ILQR_Settings& ILQR<STATE_DIM, INPUT_DIM>::settings() {
  return settings_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void ILQR<STATE_DIM, INPUT_DIM>::setupOptimizer(size_t numPartitions) {
  BASE::setupOptimizer(numPartitions);

  GvTrajectoriesStock_.resize(numPartitions);
  GmTrajectoriesStock_.resize(numPartitions);
}

}  // namespace ocs2
