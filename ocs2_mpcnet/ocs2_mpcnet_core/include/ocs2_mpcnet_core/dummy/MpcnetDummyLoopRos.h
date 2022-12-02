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

#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_oc/rollout/RolloutBase.h>
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>
#include <ocs2_ros_interfaces/mrt/DummyObserver.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>

#include "ocs2_mpcnet_core/control/MpcnetControllerBase.h"

namespace ocs2 {
namespace mpcnet {

/**
 * Dummy loop to test a robot controlled by an MPC-Net policy.
 */
class MpcnetDummyLoopRos {
 public:
  /**
   * Constructor.
   * @param [in] controlFrequency : Minimum frequency at which the MPC-Net policy should be called.
   * @param [in] rosFrequency : Frequency at which the ROS observers are updated.
   * @param [in] mpcnetPtr : Pointer to the MPC-Net policy to be used (this class takes ownership).
   * @param [in] rolloutPtr : Pointer to the rollout to be used (this class takes ownership).
   * @param [in] rosReferenceManagerPtr : Pointer to the reference manager to be used (shared ownership).
   */
  MpcnetDummyLoopRos(scalar_t controlFrequency, scalar_t rosFrequency, std::unique_ptr<MpcnetControllerBase> mpcnetPtr,
                     std::unique_ptr<RolloutBase> rolloutPtr, std::shared_ptr<RosReferenceManager> rosReferenceManagerPtr);

  /**
   * Default destructor.
   */
  virtual ~MpcnetDummyLoopRos() = default;

  /**
   * Runs the dummy loop.
   * @param [in] systemObservation: The initial system observation.
   * @param [in] targetTrajectories: The initial target trajectories.
   */
  void run(const SystemObservation& systemObservation, const TargetTrajectories& targetTrajectories);

  /**
   * Adds one observer to the vector of observers that need to be informed about the system observation, primal solution and command data.
   * Each observer is updated once after running a rollout.
   * @param [in] observer : The observer to add.
   */
  void addObserver(std::shared_ptr<DummyObserver> observer);

  /**
   * Adds one module to the vector of modules that need to be synchronized with the policy.
   * Each module is updated once before calling the policy.
   * @param [in] synchronizedModule : The module to add.
   */
  void addSynchronizedModule(std::shared_ptr<SolverSynchronizedModule> synchronizedModule);

 protected:
  /**
   * Runs a rollout.
   * @param [in] duration : The duration of the run.
   * @param [in] initialSystemObservation : The initial system observation.
   * @param [out] finalSystemObservation : The final system observation.
   */
  void rollout(scalar_t duration, const SystemObservation& initialSystemObservation, SystemObservation& finalSystemObservation);

  /**
   * Update the reference manager and the synchronized modules.
   * @param [in] time : The current time.
   * @param [in] state : The cuurent state.
   */
  void preSolverRun(scalar_t time, const vector_t& state);

 private:
  scalar_t controlFrequency_;
  scalar_t rosFrequency_;
  std::unique_ptr<MpcnetControllerBase> mpcnetPtr_;
  std::unique_ptr<RolloutBase> rolloutPtr_;
  std::shared_ptr<RosReferenceManager> rosReferenceManagerPtr_;
  std::vector<std::shared_ptr<DummyObserver>> observerPtrs_;
  std::vector<std::shared_ptr<SolverSynchronizedModule>> synchronizedModulePtrs_;
};

}  // namespace mpcnet
}  // namespace ocs2
