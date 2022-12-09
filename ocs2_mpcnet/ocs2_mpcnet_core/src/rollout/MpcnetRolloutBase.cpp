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

#include "ocs2_mpcnet_core/rollout/MpcnetRolloutBase.h"

#include "ocs2_mpcnet_core/control/MpcnetBehavioralController.h"

namespace ocs2 {
namespace mpcnet {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MpcnetRolloutBase::set(scalar_t alpha, const std::string& policyFilePath, const SystemObservation& initialObservation,
                            const ModeSchedule& modeSchedule, const TargetTrajectories& targetTrajectories) {
  // init system observation
  systemObservation_ = initialObservation;

  // reset mpc
  mpcPtr_->reset();

  // prepare learned controller
  mpcnetPtr_->loadPolicyModel(policyFilePath);

  // reset rollout, i.e. reset the internal simulator state (e.g. relevant for RaiSim)
  rolloutPtr_->resetRollout();

  // update the reference manager
  referenceManagerPtr_->setModeSchedule(modeSchedule);
  referenceManagerPtr_->setTargetTrajectories(targetTrajectories);

  // set up behavioral controller with mixture parameter alpha and learned controller
  behavioralControllerPtr_->setAlpha(alpha);
  behavioralControllerPtr_->setLearnedController(*mpcnetPtr_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MpcnetRolloutBase::step(scalar_t timeStep) {
  // run mpc
  if (!mpcPtr_->run(systemObservation_.time, systemObservation_.state)) {
    throw std::runtime_error("[MpcnetRolloutBase::step] main routine of MPC returned false.");
  }

  // update primal solution
  primalSolution_ = mpcPtr_->getSolverPtr()->primalSolution(mpcPtr_->getSolverPtr()->getFinalTime());

  // update behavioral controller with MPC controller
  behavioralControllerPtr_->setOptimalController(*primalSolution_.controllerPtr_);

  // forward simulate system with behavioral controller
  scalar_array_t timeTrajectory;
  size_array_t postEventIndicesStock;
  vector_array_t stateTrajectory;
  vector_array_t inputTrajectory;
  rolloutPtr_->run(primalSolution_.timeTrajectory_.front(), primalSolution_.stateTrajectory_.front(),
                   primalSolution_.timeTrajectory_.front() + timeStep, behavioralControllerPtr_.get(), primalSolution_.modeSchedule_,
                   timeTrajectory, postEventIndicesStock, stateTrajectory, inputTrajectory);

  // update system observation
  systemObservation_.time = timeTrajectory.back();
  systemObservation_.state = stateTrajectory.back();
  systemObservation_.input = inputTrajectory.back();
  systemObservation_.mode = primalSolution_.modeSchedule_.modeAtTime(systemObservation_.time);

  // check forward simulated system
  if (!mpcnetDefinitionPtr_->isValid(systemObservation_.time, systemObservation_.state, referenceManagerPtr_->getModeSchedule(),
                                     referenceManagerPtr_->getTargetTrajectories())) {
    throw std::runtime_error("MpcnetDataGeneration::run Tuple (time, state, modeSchedule, targetTrajectories) is not valid.");
  }
}

}  // namespace mpcnet
}  // namespace ocs2
