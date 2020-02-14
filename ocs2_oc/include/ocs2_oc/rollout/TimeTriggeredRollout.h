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

#pragma once

#include <memory>

#include <ocs2_core/dynamics/ControlledSystemBase.h>
#include <ocs2_core/integration/Integrator.h>
#include <ocs2_core/integration/StateTriggeredEventHandler.h>
#include <ocs2_core/integration/SystemEventHandler.h>

#include "RolloutBase.h"

namespace ocs2 {

/**
 * This class is an interface class for forward rollout of the system dynamics.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class TimeTriggeredRollout : public RolloutBase<STATE_DIM, INPUT_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using BASE = RolloutBase<STATE_DIM, INPUT_DIM>;
  using typename BASE::controller_t;
  using typename BASE::input_vector_array_t;
  using typename BASE::input_vector_t;
  using typename BASE::scalar_array_t;
  using typename BASE::scalar_t;
  using typename BASE::size_array_t;
  using typename BASE::state_vector_array_t;
  using typename BASE::state_vector_t;
  using typename BASE::time_interval_array_t;

  using event_handler_t = SystemEventHandler<STATE_DIM>;
  using controlled_system_base_t = ControlledSystemBase<STATE_DIM, INPUT_DIM>;
  using ode_base_t = IntegratorBase<STATE_DIM>;

  /**
   * Constructor.
   *
   * @param [in] systemDynamics: The system dynamics for forward rollout.
   * @param [in] rolloutSettings: The rollout settings.
   */
  explicit TimeTriggeredRollout(const controlled_system_base_t& systemDynamics, Rollout_Settings rolloutSettings = Rollout_Settings())
      : BASE(std::move(rolloutSettings)), systemDynamicsPtr_(systemDynamics.clone()), systemEventHandlersPtr_(new event_handler_t) {
    // construct dynamicsIntegratorsPtr
    dynamicsIntegratorPtr_ = std::move(newIntegrator<STATE_DIM>(this->settings().integratorType_, systemEventHandlersPtr_));
  }

  /**
   * Default destructor.
   */
  ~TimeTriggeredRollout() override = default;

  TimeTriggeredRollout(const TimeTriggeredRollout&) = delete;

  TimeTriggeredRollout& operator=(const TimeTriggeredRollout&) = delete;

  TimeTriggeredRollout<STATE_DIM, INPUT_DIM>* clone() const override {
    return new TimeTriggeredRollout<STATE_DIM, INPUT_DIM>(*systemDynamicsPtr_, this->settings());
  }

  /**
   * Returns the underlying dynamics.
   */
  controlled_system_base_t* systemDynamicsPtr() { return systemDynamicsPtr_.get(); }

 protected:
  state_vector_t runImpl(time_interval_array_t timeIntervalArray, const state_vector_t& initState, controller_t* controller,
                         scalar_array_t& timeTrajectory, size_array_t& postEventIndicesStock, state_vector_array_t& stateTrajectory,
                         input_vector_array_t& inputTrajectory) override {
    if (!controller) {
      throw std::runtime_error("The input controller is not set.");
    }

    const int numSubsystems = timeIntervalArray.size();
    const int numEvents = numSubsystems - 1;

    // max number of steps for integration
    const auto maxNumSteps = static_cast<size_t>(this->settings().maxNumStepsPerSecond_ *
                                                 std::max(1.0, timeIntervalArray.back().second - timeIntervalArray.front().first));

    // clearing the output trajectories
    timeTrajectory.clear();
    timeTrajectory.reserve(maxNumSteps + 1);
    stateTrajectory.clear();
    stateTrajectory.reserve(maxNumSteps + 1);
    inputTrajectory.clear();
    inputTrajectory.reserve(maxNumSteps + 1);
    postEventIndicesStock.clear();
    postEventIndicesStock.reserve(numEvents);

    // set controller
    systemDynamicsPtr_->setController(controller);

    // reset function calls counter
    systemDynamicsPtr_->resetNumFunctionCalls();

    // reset the event class
    systemEventHandlersPtr_->reset();

    state_vector_t beginState = initState;
    int k_u = 0;  // control input iterator
    for (int i = 0; i < numSubsystems; i++) {
      Observer<STATE_DIM> observer(&stateTrajectory, &timeTrajectory);  // concatenate trajectory
      // integrate controlled system
      dynamicsIntegratorPtr_->integrate_adaptive(*systemDynamicsPtr_, observer, beginState, timeIntervalArray[i].first,
                                                 timeIntervalArray[i].second, this->settings().minTimeStep_, this->settings().absTolODE_,
                                                 this->settings().relTolODE_, maxNumSteps);

      // compute control input trajectory and concatenate to inputTrajectory
      if (this->settings().reconstructInputTrajectory_) {
        for (; k_u < timeTrajectory.size(); k_u++) {
          inputTrajectory.emplace_back(systemDynamicsPtr_->controllerPtr()->computeInput(timeTrajectory[k_u], stateTrajectory[k_u]));
        }  // end of k loop
      }

      // a jump has taken place
      if (i < numEvents) {
        postEventIndicesStock.push_back(stateTrajectory.size());
        // jump map
        systemDynamicsPtr_->computeJumpMap(timeTrajectory.back(), stateTrajectory.back(), beginState);
      }
    }  // end of i loop

    // check for the numerical stability
    this->checkNumericalStability(controller, timeTrajectory, postEventIndicesStock, stateTrajectory, inputTrajectory);

    return stateTrajectory.back();
  }

 private:
  std::unique_ptr<controlled_system_base_t> systemDynamicsPtr_;

  std::shared_ptr<event_handler_t> systemEventHandlersPtr_;

  std::unique_ptr<ode_base_t> dynamicsIntegratorPtr_;
};

}  // namespace ocs2
