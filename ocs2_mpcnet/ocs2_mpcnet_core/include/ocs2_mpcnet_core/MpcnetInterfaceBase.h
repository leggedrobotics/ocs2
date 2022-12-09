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

#include "ocs2_mpcnet_core/rollout/MpcnetRolloutManager.h"

namespace ocs2 {
namespace mpcnet {

/**
 *  Base class for all MPC-Net interfaces between C++ and Python.
 */
class MpcnetInterfaceBase {
 public:
  /**
   * Default destructor.
   */
  virtual ~MpcnetInterfaceBase() = default;

  /**
   * @see MpcnetRolloutManager::startDataGeneration()
   */
  void startDataGeneration(scalar_t alpha, const std::string& policyFilePath, scalar_t timeStep, size_t dataDecimation, size_t nSamples,
                           const matrix_t& samplingCovariance, const std::vector<SystemObservation>& initialObservations,
                           const std::vector<ModeSchedule>& modeSchedules, const std::vector<TargetTrajectories>& targetTrajectories);

  /**
   * @see MpcnetRolloutManager::isDataGenerationDone()
   */
  bool isDataGenerationDone();

  /**
   * @see MpcnetRolloutManager::getGeneratedData()
   */
  data_array_t getGeneratedData();

  /**
   * @see MpcnetRolloutManager::startPolicyEvaluation()
   */
  void startPolicyEvaluation(scalar_t alpha, const std::string& policyFilePath, scalar_t timeStep,
                             const std::vector<SystemObservation>& initialObservations, const std::vector<ModeSchedule>& modeSchedules,
                             const std::vector<TargetTrajectories>& targetTrajectories);

  /**
   * @see MpcnetRolloutManager::isPolicyEvaluationDone()
   */
  bool isPolicyEvaluationDone();

  /**
   * @see MpcnetRolloutManager::getComputedMetrics()
   */
  metrics_array_t getComputedMetrics();

 protected:
  /**
   * Default constructor.
   */
  MpcnetInterfaceBase() = default;

  std::unique_ptr<MpcnetRolloutManager> mpcnetRolloutManagerPtr_;
};

}  // namespace mpcnet
}  // namespace ocs2
