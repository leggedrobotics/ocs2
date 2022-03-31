/******************************************************************************
Copyright (c) 2022, Farbod Farshidian. All rights reserved.

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

#pragma once

#include <ocs2_core/thread_support/ThreadPool.h>

#include "ocs2_mpcnet_core/rollout/MpcnetDataGeneration.h"
#include "ocs2_mpcnet_core/rollout/MpcnetPolicyEvaluation.h"

namespace ocs2 {
namespace mpcnet {

/**
 *  A class to manage the data generation and policy evaluation rollouts for MPC-Net.
 */
class MpcnetRolloutManager {
 public:
  /**
   * Constructor.
   * @note The first nDataGenerationThreads pointers will be used for the data generation and the next nPolicyEvaluationThreads pointers for
   * the policy evaluation.
   * @param [in] nDataGenerationThreads : Number of data generation threads.
   * @param [in] nPolicyEvaluationThreads : Number of policy evaluation threads.
   * @param [in] mpcPtrs : Pointers to the MPC solvers to be used.
   * @param [in] mpcnetPtrs : Pointers to the MPC-Net policies to be used.
   * @param [in] rolloutPtrs : Pointers to the rollouts to be used.
   * @param [in] mpcnetDefinitionPtrs : Pointers to the MPC-Net definitions to be used.
   * @param [in] referenceManagerPtrs : Pointers to the reference managers to be used.
   */
  MpcnetRolloutManager(size_t nDataGenerationThreads, size_t nPolicyEvaluationThreads, std::vector<std::unique_ptr<MPC_BASE>> mpcPtrs,
                       std::vector<std::unique_ptr<MpcnetControllerBase>> mpcnetPtrs, std::vector<std::unique_ptr<RolloutBase>> rolloutPtrs,
                       std::vector<std::shared_ptr<MpcnetDefinitionBase>> mpcnetDefinitionPtrs,
                       std::vector<std::shared_ptr<ReferenceManagerInterface>> referenceManagerPtrs);

  /**
   * Default destructor.
   */
  virtual ~MpcnetRolloutManager() = default;

  /**
   * Starts the data genration forward simulated by a behavioral controller.
   * @param [in] alpha : The mixture parameter for the behavioral controller.
   * @param [in] policyFilePath : The path to the file with the learned policy for the behavioral controller.
   * @param [in] timeStep : The time step for the forward simulation of the system with the behavioral controller.
   * @param [in] dataDecimation : The integer factor used for downsampling the data signal.
   * @param [in] nSamples : The number of samples drawn from a multivariate normal distribution around the nominal states.
   * @param [in] samplingCovariance : The covariance matrix used for sampling from a multivariate normal distribution.
   * @param [in] initialObservations : The initial system observations to start from (time and state required).
   * @param [in] modeSchedules : The mode schedules providing the event times and mode sequence.
   * @param [in] targetTrajectories : The target trajectories to be tracked.
   */
  void startDataGeneration(scalar_t alpha, const std::string& policyFilePath, scalar_t timeStep, size_t dataDecimation, size_t nSamples,
                           const matrix_t& samplingCovariance, const std::vector<SystemObservation>& initialObservations,
                           const std::vector<ModeSchedule>& modeSchedules, const std::vector<TargetTrajectories>& targetTrajectories);

  /**
   * Check if data generation is done.
   * @return True if done.
   */
  bool isDataGenerationDone();

  /**
   * Get the data generated from the data generation rollout.
   * @return The generated data.
   */
  const data_array_t& getGeneratedData();

  /**
   * Starts the policy evaluation forward simulated by a behavioral controller.
   * @param [in] alpha : The mixture parameter for the behavioral controller.
   * @param [in] policyFilePath : The path to the file with the learned policy for the behavioral controller.
   * @param [in] timeStep : The time step for the forward simulation of the system with the behavioral controller.
   * @param [in] initialObservations : The initial system observations to start from (time and state required).
   * @param [in] modeSchedules : The mode schedules providing the event times and mode sequence.
   * @param [in] targetTrajectories : The target trajectories to be tracked.
   */
  void startPolicyEvaluation(scalar_t alpha, const std::string& policyFilePath, scalar_t timeStep,
                             const std::vector<SystemObservation>& initialObservations, const std::vector<ModeSchedule>& modeSchedules,
                             const std::vector<TargetTrajectories>& targetTrajectories);

  /**
   * Check if policy evaluation is done.
   * @return True if done.
   */
  bool isPolicyEvaluationDone();

  /**
   * Get the metrics computed from the policy evaluation rollout.
   * @return The computed metrics.
   */
  metrics_array_t getComputedMetrics();

 private:
  // data generation variables
  size_t nDataGenerationThreads_;
  std::atomic_int nDataGenerationTasksDone_;
  std::unique_ptr<ThreadPool> dataGenerationThreadPoolPtr_;
  std::vector<std::unique_ptr<MpcnetDataGeneration>> dataGenerationPtrs_;
  std::vector<std::future<const data_array_t*>> dataGenerationFtrs_;
  data_array_t dataArray_;
  // policy evaluation variables
  size_t nPolicyEvaluationThreads_;
  std::atomic_int nPolicyEvaluationTasksDone_;
  std::unique_ptr<ThreadPool> policyEvaluationThreadPoolPtr_;
  std::vector<std::unique_ptr<MpcnetPolicyEvaluation>> policyEvaluationPtrs_;
  std::vector<std::future<metrics_t>> policyEvaluationFtrs_;
};

}  // namespace mpcnet
}  // namespace ocs2
