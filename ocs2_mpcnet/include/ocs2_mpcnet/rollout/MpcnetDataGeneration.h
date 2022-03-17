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

#include <ocs2_core/reference/ModeSchedule.h>
#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_mpc/MPC_BASE.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_oc/rollout/RolloutBase.h>
#include <ocs2_oc/synchronized_module/ReferenceManagerInterface.h>

#include "ocs2_mpcnet/MpcnetDefinitionBase.h"
#include "ocs2_mpcnet/control/MpcnetControllerBase.h"

namespace ocs2 {

/**
 *  A class for generating MPC data from a system that is forward simulated with a behavioral controller.
 *  The behavioral policy is a mixture of an MPC policy and an MPC-Net policy (e.g. a neural network).
 */
class MpcnetDataGeneration {
 public:
  struct DataPoint {
    size_t mode;
    scalar_t t;
    vector_t x;
    vector_t u;
    vector_t generalizedTime;
    vector_t relativeState;
    matrix_t inputTransformation;
    ScalarFunctionQuadraticApproximation hamiltonian;
  };
  using DataArray = std::vector<DataPoint>;
  using DataPtr = std::unique_ptr<DataArray>;

  /**
   * Constructor.
   * @param [in] mpcPtr : Pointer to the MPC solver to be used (this class takes ownership).
   * @param [in] mpcnetPtr : Pointer to the MPC-Net policy to be used (this class takes ownership).
   * @param [in] rolloutPtr : Pointer to the rollout to be used (this class takes ownership).
   * @param [in] mpcnetDefinitionPtr : Pointer to the MPC-Net definitions to be used (shared ownership).
   * @param [in] referenceManagerPtr : Pointer to the reference manager to be used (shared ownership).
   */
  MpcnetDataGeneration(std::unique_ptr<MPC_BASE> mpcPtr, std::unique_ptr<MpcnetControllerBase> mpcnetPtr,
                       std::unique_ptr<RolloutBase> rolloutPtr, std::shared_ptr<MpcnetDefinitionBase> mpcnetDefinitionPtr,
                       std::shared_ptr<ReferenceManagerInterface> referenceManagerPtr)
      : mpcPtr_(std::move(mpcPtr)),
        mpcnetPtr_(std::move(mpcnetPtr)),
        rolloutPtr_(std::move(rolloutPtr)),
        mpcnetDefinitionPtr_(std::move(mpcnetDefinitionPtr)),
        referenceManagerPtr_(std::move(referenceManagerPtr)) {}

  /**
   * Default destructor.
   */
  virtual ~MpcnetDataGeneration() = default;

  /**
   * Deleted copy constructor.
   */
  MpcnetDataGeneration(const MpcnetDataGeneration&) = delete;

  /**
   * Deleted copy assignment.
   */
  MpcnetDataGeneration& operator=(const MpcnetDataGeneration&) = delete;

  /**
   * Run the data generation.
   * @param [in] alpha : The mixture parameter for the behavioral controller.
   * @param [in] policyFilePath : The path to the file with the learned policy for the behavioral controller.
   * @param [in] timeStep : The time step for the forward simulation of the system with the behavioral controller.
   * @param [in] dataDecimation : The integer factor used for downsampling the data signal.
   * @param [in] nSamples : The number of samples drawn from a multivariate normal distribution around the nominal states.
   * @param [in] samplingCovariance : The covariance matrix used for sampling from a multivariate normal distribution.
   * @param [in] initialObservation : The initial system observation to start from (time and state required).
   * @param [in] modeSchedule : The mode schedule providing the event times and mode sequence.
   * @param [in] targetTrajectories : The target trajectories to be tracked.
   * @return Pointer to the generated data.
   */
  DataPtr run(scalar_t alpha, const std::string& policyFilePath, scalar_t timeStep, size_t dataDecimation, size_t nSamples,
              const matrix_t& samplingCovariance, const SystemObservation& initialObservation, const ModeSchedule& modeSchedule,
              const TargetTrajectories& targetTrajectories);

 private:
  std::unique_ptr<MPC_BASE> mpcPtr_;
  std::unique_ptr<MpcnetControllerBase> mpcnetPtr_;
  std::unique_ptr<RolloutBase> rolloutPtr_;
  std::shared_ptr<MpcnetDefinitionBase> mpcnetDefinitionPtr_;
  std::shared_ptr<ReferenceManagerInterface> referenceManagerPtr_;
};

}  // namespace ocs2
