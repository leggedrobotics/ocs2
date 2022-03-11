/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

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

#include "ocs2_oc/rollout/TimeTriggeredRollout.h"

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
TimeTriggeredRollout::TimeTriggeredRollout(const ControlledSystemBase& systemDynamics, rollout::Settings rolloutSettings)
    : RolloutBase(std::move(rolloutSettings)), systemDynamicsPtr_(systemDynamics.clone()), systemEventHandlersPtr_(new SystemEventHandler) {
  // construct dynamicsIntegratorsPtr
  dynamicsIntegratorPtr_ = std::move(newIntegrator(this->settings().integratorType, systemEventHandlersPtr_));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t TimeTriggeredRollout::run(scalar_t initTime, const vector_t& initState, scalar_t finalTime, ControllerBase* controller,
                                   ModeSchedule& modeSchedule, scalar_array_t& timeTrajectory, size_array_t& postEventIndices,
                                   vector_array_t& stateTrajectory, vector_array_t& inputTrajectory) {
  if (initTime > finalTime) {
    throw std::runtime_error("[TimeTriggeredRollout::run] The initial time should be less-equal to the final time!");
  }
  if (controller == nullptr) {
    throw std::runtime_error("[TimeTriggeredRollout::run] Controller is not set!");
  }

  // extract sub-systems
  const auto timeIntervalArray = findActiveModesTimeInterval(initTime, finalTime, modeSchedule.eventTimes);
  const int numSubsystems = timeIntervalArray.size();
  const int numEvents = numSubsystems - 1;

  // max number of steps for integration
  const auto maxNumSteps = static_cast<size_t>(this->settings().maxNumStepsPerSecond * std::max(1.0, finalTime - initTime));

  // clearing the output trajectories
  timeTrajectory.clear();
  timeTrajectory.reserve(maxNumSteps + 1);
  stateTrajectory.clear();
  stateTrajectory.reserve(maxNumSteps + 1);
  inputTrajectory.clear();
  inputTrajectory.reserve(maxNumSteps + 1);
  postEventIndices.clear();
  postEventIndices.reserve(numEvents);

  // set controller
  systemDynamicsPtr_->setController(controller);

  // reset function calls counter
  systemDynamicsPtr_->resetNumFunctionCalls();

  // reset the event class
  systemEventHandlersPtr_->reset();

  vector_t beginState = initState;
  int k_u = 0;  // control input iterator
  for (int i = 0; i < numSubsystems; i++) {
    if (timeIntervalArray[i].first < timeIntervalArray[i].second) {
      Observer observer(&stateTrajectory, &timeTrajectory);  // concatenate trajectory
      // integrate controlled system
      dynamicsIntegratorPtr_->integrateAdaptive(*systemDynamicsPtr_, observer, beginState, timeIntervalArray[i].first,
                                                timeIntervalArray[i].second, this->settings().timeStep, this->settings().absTolODE,
                                                this->settings().relTolODE, maxNumSteps);
    } else {
      timeTrajectory.push_back(timeIntervalArray[i].second);
      stateTrajectory.push_back(beginState);
    }

    // compute control input trajectory and concatenate to inputTrajectory
    if (this->settings().reconstructInputTrajectory) {
      for (; k_u < timeTrajectory.size(); k_u++) {
        inputTrajectory.emplace_back(systemDynamicsPtr_->controllerPtr()->computeInput(timeTrajectory[k_u], stateTrajectory[k_u]));
      }  // end of k_u loop
    }

    // a jump has taken place
    if (i < numEvents) {
      postEventIndices.push_back(stateTrajectory.size());
      // jump map
      beginState = systemDynamicsPtr_->computeJumpMap(timeTrajectory.back(), stateTrajectory.back());
    }
  }  // end of i loop

  // check for the numerical stability
  this->checkNumericalStability(*controller, timeTrajectory, postEventIndices, stateTrajectory, inputTrajectory);

  return stateTrajectory.back();
}

}  // namespace ocs2
