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
                                 const cost_function_base_t* heuristicsFunctionPtr /* = nullptr*/)

    : BASE(rolloutPtr, systemDerivativesPtr, systemConstraintsPtr, costFunctionPtr, operatingTrajectoriesPtr, settings.ddpSettings_,
           heuristicsFunctionPtr, "ILQR"),
      settings_(settings) {
  Eigen::initParallel();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void ILQR<STATE_DIM, INPUT_DIM>::approximateOptimalControlProblem() {
  for (size_t i = 0; i < BASE::numPartitions_; i++) {
    size_t N = BASE::nominalTimeTrajectoriesStock_[i].size();
    // resizing the new variables containers
    discreteModelDataTrajectoriesStock_[i].resize(N);

    RmInverseDtimeTrajectoryStock_[i].resize(N);
  }  // end of i loop

  // base method should be called after the resizes
  BASE::approximateOptimalControlProblem();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void ILQR<STATE_DIM, INPUT_DIM>::approximateLQWorker(size_t workerIndex, size_t partitionIndex, size_t timeIndex) {
  // unconstrained LQ problem
  approximateUnconstrainedLQWorker(workerIndex, partitionIndex, timeIndex);

  // discretize LQ problem
  discreteLQWorker(workerIndex, partitionIndex, timeIndex);

  const scalar_t stateConstraintPenalty =
      BASE::ddpSettings_.stateConstraintPenaltyCoeff_ * pow(BASE::ddpSettings_.stateConstraintPenaltyBase_, BASE::iteration_);

  // // modify the unconstrained LQ coefficients to constrained ones
  // approximateConstrainedLQWorker(workerIndex, partitionIndex, timeIndex, stateConstraintPenalty);

  // calculate an LQ approximate of the event times process.
  BASE::approximateEventsLQWorker(workerIndex, partitionIndex, timeIndex, stateConstraintPenalty);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void ILQR<STATE_DIM, INPUT_DIM>::approximateUnconstrainedLQWorker(size_t workerIndex, size_t i, size_t k) {
  BASE::approximateUnconstrainedLQWorker(workerIndex, i, k);

  // making sure that constrained Qm is PSD
  if (BASE::ddpSettings_.useMakePSD_) {
    LinearAlgebra::makePSD(BASE::modelDataTrajectoriesStock_[i][k].costStateSecondDerivative_);
  }

  // TODO: add support for the constrained ILQR
  if (BASE::modelDataTrajectoriesStock_[i][k].numStateEqConstr_ != 0) {
    throw std::runtime_error("We currently only support unconstrained ILQR.");
  }
  if (BASE::modelDataTrajectoriesStock_[i][k].numStateInputEqConstr_ != 0) {
    throw std::runtime_error("We currently only support unconstrained ILQR.");
  }
  if (BASE::modelDataTrajectoriesStock_[i][k].numIneqConstr_ != 0) {
    throw std::runtime_error("We currently only support unconstrained ILQR.");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void ILQR<STATE_DIM, INPUT_DIM>::discreteLQWorker(size_t workerIndex, size_t i, size_t k) {
  // time step
  scalar_t dt = 0.0;
  if (k + 1 < BASE::nominalTimeTrajectoriesStock_[i].size()) {
    dt = BASE::nominalTimeTrajectoriesStock_[i][k + 1] - BASE::nominalTimeTrajectoriesStock_[i][k];
  }

  /*
   * linearize system dynamics
   */
  discreteModelDataTrajectoriesStock_[i][k].dynamicsStateDerivative_ =
      BASE::modelDataTrajectoriesStock_[i][k].dynamicsStateDerivative_ * dt + dynamic_matrix_t::Identity(STATE_DIM, STATE_DIM);
  discreteModelDataTrajectoriesStock_[i][k].dynamicsInputDerivative_ =
      BASE::modelDataTrajectoriesStock_[i][k].dynamicsInputDerivative_ * dt;

  /*
   * quadratic approximation to the cost function
   */
  discreteModelDataTrajectoriesStock_[i][k].cost_ = BASE::modelDataTrajectoriesStock_[i][k].cost_ * dt;
  discreteModelDataTrajectoriesStock_[i][k].costStateDerivative_ = BASE::modelDataTrajectoriesStock_[i][k].costStateDerivative_ * dt;
  discreteModelDataTrajectoriesStock_[i][k].costStateSecondDerivative_ =
      BASE::modelDataTrajectoriesStock_[i][k].costStateSecondDerivative_ * dt;
  discreteModelDataTrajectoriesStock_[i][k].costInputDerivative_ = BASE::modelDataTrajectoriesStock_[i][k].costInputDerivative_ * dt;
  discreteModelDataTrajectoriesStock_[i][k].costInputSecondDerivative_ =
      BASE::modelDataTrajectoriesStock_[i][k].costInputSecondDerivative_ * dt;
  discreteModelDataTrajectoriesStock_[i][k].costInputStateDerivative_ =
      BASE::modelDataTrajectoriesStock_[i][k].costInputStateDerivative_ * dt;
  RmInverseDtimeTrajectoryStock_[i][k] =
      discreteModelDataTrajectoriesStock_[i][k].costInputSecondDerivative_.ldlt().solve(dynamic_matrix_t::Identity(INPUT_DIM, INPUT_DIM));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void ILQR<STATE_DIM, INPUT_DIM>::getStateInputConstraintLagrangian(scalar_t time, const state_vector_t& state, dynamic_vector_t& nu) const {
  throw std::runtime_error("Not implemented");
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

  const state_vector_t& nominalState = BASE::nominalStateTrajectoriesStock_[i][k];
  const input_vector_t& nominalInput = BASE::nominalInputTrajectoriesStock_[i][k];

  input_state_matrix_t Lm = HmInverseTrajectoryStock_[i][k] * GmTrajectoryStock_[i][k];
  input_vector_t Lv = HmInverseTrajectoryStock_[i][k] * GvTrajectoryStock_[i][k];
  input_vector_t Lve = input_vector_t::Zero();

  BASE::nominalControllersStock_[i].gainArray_[k] = -Lm;
  BASE::nominalControllersStock_[i].biasArray_[k] = nominalInput - BASE::nominalControllersStock_[i].gainArray_[k] * nominalState;
  BASE::nominalControllersStock_[i].deltaBiasArray_[k] = -Lv;

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
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void ILQR<STATE_DIM, INPUT_DIM>::riccatiSolverTask() {
  size_t taskId = BASE::nextTaskId_++;  // assign task ID (atomic)

  for (int i = BASE::endingIndicesRiccatiWorker_[taskId]; i >= BASE::startingIndicesRiccatiWorker_[taskId]; i--) {
    // for inactive subsystems
    if (i < BASE::initActivePartition_ || i > BASE::finalActivePartition_) {
      BASE::SsTimeTrajectoryStock_[i].clear();
      BASE::SmTrajectoryStock_[i].clear();
      BASE::SvTrajectoryStock_[i].clear();
      BASE::SveTrajectoryStock_[i].clear();
      BASE::sTrajectoryStock_[i].clear();

      {  // lock data
        std::lock_guard<std::mutex> lock(riccatiSolverDataMutex_);

        BASE::SmFinalStock_[i].setZero();
        BASE::SvFinalStock_[i].setZero();
        BASE::SveFinalStock_[i].setZero();
        BASE::sFinalStock_[i] = 0.0;
        BASE::xFinalStock_[i].setZero();
      }

    } else {
      state_matrix_t SmFinal;
      state_vector_t SvFinal;
      state_vector_t SveFinal;
      scalar_t sFinal;
      state_vector_t xFinal;

      {  // lock data
        std::lock_guard<std::mutex> lock(riccatiSolverDataMutex_);

        SmFinal = BASE::SmFinalStock_[i];
        SvFinal = BASE::SvFinalStock_[i];
        SveFinal = BASE::SveFinalStock_[i];
        sFinal = BASE::sFinalStock_[i];
        xFinal = BASE::xFinalStock_[i];
      }

      // modify the end subsystem final values based on the cached values for asynchronous run
      if (i == BASE::endingIndicesRiccatiWorker_[taskId] && i < BASE::finalActivePartition_) {
        const state_vector_t deltaState = BASE::nominalStateTrajectoriesStock_[i + 1].front() - xFinal;
        sFinal += deltaState.dot(0.5 * SmFinal * deltaState + SvFinal);
        SvFinal += SmFinal * deltaState;
      }

      // solve the backward pass
      riccatiEquationsWorker(taskId, i, SmFinal, SvFinal, sFinal);

      // set the final value for next Riccati equation
      if (i > BASE::initActivePartition_) {
        // lock data
        std::lock_guard<std::mutex> lock(riccatiSolverDataMutex_);

        BASE::SmFinalStock_[i - 1] = BASE::SmTrajectoryStock_[i].front();
        BASE::SvFinalStock_[i - 1] = BASE::SvTrajectoryStock_[i].front();
        // BASE::SveFinalStock_[i - 1] = BASE::SveTrajectoryStock_[i].front();
        BASE::sFinalStock_[i - 1] = BASE::sTrajectoryStock_[i].front();
        BASE::xFinalStock_[i - 1] = BASE::nominalStateTrajectoriesStock_[i].front();
      }
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void ILQR<STATE_DIM, INPUT_DIM>::riccatiEquationsWorker(size_t workerIndex, size_t partitionIndex, const state_matrix_t& SmFinal,
                                                        const state_vector_t& SvFinal, const scalar_t& sFinal) {
  const size_t N = BASE::nominalTimeTrajectoriesStock_[partitionIndex].size();
  const size_t NE = BASE::nominalPostEventIndicesStock_[partitionIndex].size();

  const scalar_t scalingStart = BASE::partitioningTimes_[partitionIndex];
  const scalar_t scalingFinal = BASE::partitioningTimes_[partitionIndex + 1];
  const scalar_t scalingFactor = scalingStart - scalingFinal;  // this is negative

  // normalized time
  BASE::SsNormalizedTimeTrajectoryStock_[partitionIndex].resize(N);
  for (size_t k = 0; k < N; k++) {
    BASE::SsNormalizedTimeTrajectoryStock_[partitionIndex][N - 1 - k] =
        (BASE::nominalTimeTrajectoriesStock_[partitionIndex][k] - scalingFinal) / scalingFactor;
  }

  // normalized event past the index
  BASE::SsNormalizedEventsPastTheEndIndecesStock_[partitionIndex].resize(NE);
  for (size_t k = 0; k < NE; k++) {
    BASE::SsNormalizedEventsPastTheEndIndecesStock_[partitionIndex][NE - 1 - k] =
        N - BASE::nominalPostEventIndicesStock_[partitionIndex][k];
  }

  // output containers resizing
  BASE::SsTimeTrajectoryStock_[partitionIndex] = BASE::nominalTimeTrajectoriesStock_[partitionIndex];
  BASE::sTrajectoryStock_[partitionIndex].resize(N);
  BASE::SvTrajectoryStock_[partitionIndex].resize(N);
  BASE::SmTrajectoryStock_[partitionIndex].resize(N);

  HmTrajectoryStock_[partitionIndex].resize(N);
  HmInverseTrajectoryStock_[partitionIndex].resize(N);
  GmTrajectoryStock_[partitionIndex].resize(N);
  GvTrajectoryStock_[partitionIndex].resize(N);

  // terminate if the partition is not active
  if (N == 0) {
    return;
  }

  // switching times
  size_array_t SsSwitchingTimesIndices;
  SsSwitchingTimesIndices.reserve(NE + 2);
  SsSwitchingTimesIndices.push_back(0);
  for (size_t k = 0; k < NE; k++) {
    const auto index = BASE::nominalPostEventIndicesStock_[partitionIndex][k];
    SsSwitchingTimesIndices.push_back(index);
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
     * solution at final time of an interval (uses the continuous-time formulation)
     */
    const size_t finalIndex = endTimeItr - 1;
    // note that these are the continuous time coefficients
    const auto& Bmc = BASE::modelDataTrajectoriesStock_[partitionIndex][finalIndex].dynamicsInputDerivative_;
    const auto& Rvc = BASE::modelDataTrajectoriesStock_[partitionIndex][finalIndex].costInputDerivative_;
    const auto& Rmc = BASE::modelDataTrajectoriesStock_[partitionIndex][finalIndex].costInputSecondDerivative_;
    const auto& Pmc = BASE::modelDataTrajectoriesStock_[partitionIndex][finalIndex].costInputStateDerivative_;
    const input_matrix_t RmcInverse = Rmc.ldlt().solve(input_matrix_t::Identity());

    BASE::sTrajectoryStock_[partitionIndex][finalIndex] = sFinalTemp;
    BASE::SvTrajectoryStock_[partitionIndex][finalIndex] = SvFinalTemp;
    BASE::SmTrajectoryStock_[partitionIndex][finalIndex] = SmFinalTemp;

    HmTrajectoryStock_[partitionIndex][finalIndex] = Rmc;
    HmInverseTrajectoryStock_[partitionIndex][finalIndex] = RmcInverse;
    GmTrajectoryStock_[partitionIndex][finalIndex] = Pmc + Bmc.transpose() * SmFinalTemp;
    GvTrajectoryStock_[partitionIndex][finalIndex] = Rvc + Bmc.transpose() * SvFinalTemp;

    // solve Riccati equations if interval length is not zero
    if (beginTimeItr < endTimeItr - 1) {
      for (int k = endTimeItr - 2; k >= beginTimeItr; k--) {
        const auto& Am = discreteModelDataTrajectoriesStock_[partitionIndex][k].dynamicsStateDerivative_;
        const auto& Bm = discreteModelDataTrajectoriesStock_[partitionIndex][k].dynamicsInputDerivative_;
        const auto& q = discreteModelDataTrajectoriesStock_[partitionIndex][k].cost_;
        const auto& Qv = discreteModelDataTrajectoriesStock_[partitionIndex][k].costStateDerivative_;
        const auto& Qm = discreteModelDataTrajectoriesStock_[partitionIndex][k].costStateSecondDerivative_;
        const auto& Rv = discreteModelDataTrajectoriesStock_[partitionIndex][k].costInputDerivative_;
        const auto& Rm = discreteModelDataTrajectoriesStock_[partitionIndex][k].costInputSecondDerivative_;
        const auto& Pm = discreteModelDataTrajectoriesStock_[partitionIndex][k].costInputStateDerivative_;

        auto& Hm = HmTrajectoryStock_[partitionIndex][k];
        auto& HmInverse = HmInverseTrajectoryStock_[partitionIndex][k];
        auto& Gv = GvTrajectoryStock_[partitionIndex][k];
        auto& Gm = GmTrajectoryStock_[partitionIndex][k];

        Hm = Rm + Bm.transpose() * BASE::SmTrajectoryStock_[partitionIndex][k + 1] * Bm;
        if (BASE::ddpSettings_.useMakePSD_) {
          LinearAlgebra::makePSD(Hm);
        } else {
          Hm += BASE::ddpSettings_.addedRiccatiDiagonal_ * input_matrix_t::Identity();
        }
        HmInverse = Hm.ldlt().solve(input_matrix_t::Identity());
        Gm = Pm + Bm.transpose() * BASE::SmTrajectoryStock_[partitionIndex][k + 1] * Am;
        Gv = Rv + Bm.transpose() * BASE::SvTrajectoryStock_[partitionIndex][k + 1];

        BASE::sTrajectoryStock_[partitionIndex][k] = q + BASE::sTrajectoryStock_[partitionIndex][k + 1] - 0.5 * Gv.dot(HmInverse * Gv);
        BASE::SvTrajectoryStock_[partitionIndex][k] =
            Qv + Am.transpose() * BASE::SvTrajectoryStock_[partitionIndex][k + 1] - Gm.transpose() * HmInverse * Gv;
        BASE::SmTrajectoryStock_[partitionIndex][k] =
            Qm + Am.transpose() * BASE::SmTrajectoryStock_[partitionIndex][k + 1] * Am - Gm.transpose() * HmInverse * Gm;
      }
    }

    if (i > 0) {
      sFinalTemp = BASE::sTrajectoryStock_[partitionIndex][beginTimeItr] + BASE::qFinalStock_[partitionIndex][i - 1];
      SvFinalTemp = BASE::SvTrajectoryStock_[partitionIndex][beginTimeItr] + BASE::QvFinalStock_[partitionIndex][i - 1];
      SmFinalTemp = BASE::SmTrajectoryStock_[partitionIndex][beginTimeItr] + BASE::QmFinalStock_[partitionIndex][i - 1];
    }

  }  // end of i loop

  // testing the numerical stability of the Riccati equations
  if (BASE::ddpSettings_.checkNumericalStability_) {
    for (int k = N - 1; k >= 0; k--) {
      try {
        if (!BASE::SmTrajectoryStock_[partitionIndex][k].allFinite()) {
          throw std::runtime_error("Sm is unstable.");
        }
        if (BASE::SmTrajectoryStock_[partitionIndex][k].eigenvalues().real().minCoeff() < -Eigen::NumTraits<scalar_t>::epsilon()) {
          throw std::runtime_error("Sm matrix is not positive semi-definite. It's smallest eigenvalue is " +
                                   std::to_string(BASE::SmTrajectoryStock_[partitionIndex][k].eigenvalues().real().minCoeff()) + ".");
        }
        if (!BASE::SvTrajectoryStock_[partitionIndex][k].allFinite()) {
          throw std::runtime_error("Sv is unstable.");
        }
        if (BASE::sTrajectoryStock_[partitionIndex][k] != BASE::sTrajectoryStock_[partitionIndex][k]) {
          throw std::runtime_error("s is unstable");
        }
      } catch (const std::exception& error) {
        std::cerr << "what(): " << error.what() << " at time " << BASE::SsTimeTrajectoryStock_[partitionIndex][k] << " [sec]." << std::endl;
        for (int kp = k; kp < k + 10; kp++) {
          if (kp >= N) {
            continue;
          }
          std::cerr << "Sm[" << BASE::SsTimeTrajectoryStock_[partitionIndex][kp] << "]:\n"
                    << BASE::SmTrajectoryStock_[partitionIndex][kp].norm() << std::endl;
          std::cerr << "Sv[" << BASE::SsTimeTrajectoryStock_[partitionIndex][kp] << "]:\t"
                    << BASE::SvTrajectoryStock_[partitionIndex][kp].transpose().norm() << std::endl;
          std::cerr << "s[" << BASE::SsTimeTrajectoryStock_[partitionIndex][kp] << "]:\t" << BASE::sTrajectoryStock_[partitionIndex][kp]
                    << std::endl;
        }
        throw;
      }
    }
  }
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

  discreteModelDataTrajectoriesStock_.resize(numPartitions);

  RmInverseDtimeTrajectoryStock_.resize(numPartitions);
  HmTrajectoryStock_.resize(numPartitions);
  HmInverseTrajectoryStock_.resize(numPartitions);
  GmTrajectoryStock_.resize(numPartitions);
  GvTrajectoryStock_.resize(numPartitions);
}

}  // namespace ocs2
