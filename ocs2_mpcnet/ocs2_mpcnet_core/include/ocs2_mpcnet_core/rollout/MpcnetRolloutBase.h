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
#include <ocs2_oc/oc_data/PrimalSolution.h>
#include <ocs2_oc/rollout/RolloutBase.h>
#include <ocs2_oc/synchronized_module/ReferenceManagerInterface.h>

#include "ocs2_mpcnet_core/MpcnetDefinitionBase.h"
#include "ocs2_mpcnet_core/control/MpcnetBehavioralController.h"
#include "ocs2_mpcnet_core/control/MpcnetControllerBase.h"

namespace ocs2 {
namespace mpcnet {

/**
 *  The base class for doing rollouts for a system that is forward simulated with a behavioral controller.
 *  The behavioral policy is a mixture of an MPC policy and an MPC-Net policy (e.g. a neural network).
 */
class MpcnetRolloutBase {
 public:
  /**
   * Constructor.
   * @param [in] mpcPtr : Pointer to the MPC solver to be used (this class takes ownership).
   * @param [in] mpcnetPtr : Pointer to the MPC-Net policy to be used (this class takes ownership).
   * @param [in] rolloutPtr : Pointer to the rollout to be used (this class takes ownership).
   * @param [in] mpcnetDefinitionPtr : Pointer to the MPC-Net definitions to be used (shared ownership).
   * @param [in] referenceManagerPtr : Pointer to the reference manager to be used (shared ownership).
   */
  MpcnetRolloutBase(std::unique_ptr<MPC_BASE> mpcPtr, std::unique_ptr<MpcnetControllerBase> mpcnetPtr,
                    std::unique_ptr<RolloutBase> rolloutPtr, std::shared_ptr<MpcnetDefinitionBase> mpcnetDefinitionPtr,
                    std::shared_ptr<ReferenceManagerInterface> referenceManagerPtr)
      : mpcPtr_(std::move(mpcPtr)),
        mpcnetPtr_(std::move(mpcnetPtr)),
        rolloutPtr_(std::move(rolloutPtr)),
        mpcnetDefinitionPtr_(std::move(mpcnetDefinitionPtr)),
        referenceManagerPtr_(std::move(referenceManagerPtr)),
        behavioralControllerPtr_(new MpcnetBehavioralController()) {}

  /**
   * Default destructor.
   */
  virtual ~MpcnetRolloutBase() = default;

  /**
   * Deleted copy constructor.
   */
  MpcnetRolloutBase(const MpcnetRolloutBase&) = delete;

  /**
   * Deleted copy assignment.
   */
  MpcnetRolloutBase& operator=(const MpcnetRolloutBase&) = delete;

 protected:
  /**
   * (Re)set system components.
   * @param [in] alpha : The mixture parameter for the behavioral controller.
   * @param [in] policyFilePath : The path to the file with the learned policy for the controller.
   * @param [in] initialObservation : The initial system observation to start from (time and state required).
   * @param [in] modeSchedule : The mode schedule providing the event times and mode sequence.
   * @param [in] targetTrajectories : The target trajectories to be tracked.
   */
  void set(scalar_t alpha, const std::string& policyFilePath, const SystemObservation& initialObservation, const ModeSchedule& modeSchedule,
           const TargetTrajectories& targetTrajectories);

  /**
   * Simulate the system one step forward.
   * @param [in] timeStep : The time step for the forward simulation of the system with the behavioral controller.
   */
  void step(scalar_t timeStep);

  std::unique_ptr<MPC_BASE> mpcPtr_;
  std::shared_ptr<MpcnetDefinitionBase> mpcnetDefinitionPtr_;
  std::unique_ptr<MpcnetBehavioralController> behavioralControllerPtr_;
  SystemObservation systemObservation_;
  PrimalSolution primalSolution_;

 private:
  std::unique_ptr<MpcnetControllerBase> mpcnetPtr_;
  std::unique_ptr<RolloutBase> rolloutPtr_;
  std::shared_ptr<ReferenceManagerInterface> referenceManagerPtr_;
};

}  // namespace mpcnet
}  // namespace ocs2
