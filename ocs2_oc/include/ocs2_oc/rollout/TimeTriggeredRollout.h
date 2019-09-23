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

  using event_handler_t = SystemEventHandler<STATE_DIM>;
  using controlled_system_base_t = ControlledSystemBase<STATE_DIM, INPUT_DIM>;
  using ode_base_t = IntegratorBase<STATE_DIM>;

  /**
   * Constructor.
   *
   * @param [in] systemDynamics: The system dynamics for forward rollout.
   * @param [in] rolloutSettings: The rollout settings.
   * @param [in] algorithmName: The algorithm that calls this class (default not defined).
   */
  explicit TimeTriggeredRollout(const controlled_system_base_t& systemDynamics,
                                const Rollout_Settings& rolloutSettings = Rollout_Settings(), const char algorithmName[] = nullptr)
      : BASE(rolloutSettings, algorithmName), systemDynamicsPtr_(systemDynamics.clone()), systemEventHandlersPtr_(new event_handler_t) {
    switch (rolloutSettings.integratorType_) {
      case (IntegratorType::EULER): {
        dynamicsIntegratorsPtr_.reset(new IntegratorEuler<STATE_DIM>(systemDynamicsPtr_, systemEventHandlersPtr_));
        break;
      }
      case (IntegratorType::MODIFIED_MIDPOINT): {
        dynamicsIntegratorsPtr_.reset(new IntegratorModifiedMidpoint<STATE_DIM>(systemDynamicsPtr_, systemEventHandlersPtr_));
        break;
      }
      case (IntegratorType::RK4): {
        dynamicsIntegratorsPtr_.reset(new IntegratorRK4<STATE_DIM>(systemDynamicsPtr_, systemEventHandlersPtr_));
        break;
      }
      case (IntegratorType::RK5_VARIABLE): {
        dynamicsIntegratorsPtr_.reset(new IntegratorRK5Variable<STATE_DIM>(systemDynamicsPtr_, systemEventHandlersPtr_));
        break;
      }
      case (IntegratorType::ODE45): {
        dynamicsIntegratorsPtr_.reset(new ODE45<STATE_DIM>(systemDynamicsPtr_, systemEventHandlersPtr_));
        break;
      }
      case (IntegratorType::ADAMS_BASHFORTH): {
        const size_t numberSteps = 1;
        dynamicsIntegratorsPtr_.reset(new IntegratorAdamsBashforth<STATE_DIM, numberSteps>(systemDynamicsPtr_, systemEventHandlersPtr_));
        break;
      }
      case (IntegratorType::BULIRSCH_STOER): {
        dynamicsIntegratorsPtr_.reset(new IntegratorBulirschStoer<STATE_DIM>(systemDynamicsPtr_, systemEventHandlersPtr_));
        break;
      }
#if (BOOST_VERSION / 100000 == 1 && BOOST_VERSION / 100 % 1000 > 55)
      case (IntegratorType::ADAMS_BASHFORTH_MOULTON): {
        const size_t numberSteps = 1;  // maximum is 8
        dynamicsIntegratorsPtr_.reset(
            new IntegratorAdamsBashforthMoulton<STATE_DIM, numberSteps>(systemDynamicsPtr_, systemEventHandlersPtr_));
        break;
      }
#endif
      default: {
        throw std::runtime_error("Integrator of type " +
                                 std::to_string(static_cast<std::underlying_type<IntegratorType>::type>(rolloutSettings.integratorType_)) +
                                 " not supported in TimeTriggeredRollout.");
      }
    }
  }

  /**
   * Default destructor.
   */
  ~TimeTriggeredRollout() override = default;

 protected:
  state_vector_t runImpl(scalar_array_t& switchingTimes, const state_vector_t& initState, controller_t* controller,
                         scalar_array_t& timeTrajectory, size_array_t& eventsPastTheEndIndeces, state_vector_array_t& stateTrajectory,
                         input_vector_array_t& inputTrajectory) override {
    if (controller == nullptr) {
      throw std::runtime_error("The input controller is not set.");
    }

    const int numEvents = switchingTimes.size() - 2;
    const int numSubsystems = switchingTimes.size() - 1;

    // max number of steps for integration
    const auto maxNumSteps =
        static_cast<size_t>(BASE::settings().maxNumStepsPerSecond_ * std::max(1.0, switchingTimes.back() - switchingTimes.front()));

    // clearing the output trajectories
    timeTrajectory.clear();
    timeTrajectory.reserve(maxNumSteps + 1);
    stateTrajectory.clear();
    stateTrajectory.reserve(maxNumSteps + 1);
    inputTrajectory.clear();
    inputTrajectory.reserve(maxNumSteps + 1);
    eventsPastTheEndIndeces.clear();
    eventsPastTheEndIndeces.reserve(numEvents);

    // set controller
    systemDynamicsPtr_->setController(controller);

    // reset function calls counter
    systemDynamicsPtr_->resetNumFunctionCalls();

    // Reset the event class
    systemEventHandlersPtr_->reset();

    state_vector_t beginState = initState;
    size_t k_u = 0;  // control input iterator
    for (int i = 0; i < numSubsystems; i++) {
      scalar_t beginTime = switchingTimes[i];
      scalar_t endTime = switchingTimes[i + 1];

      const scalar_t eps = OCS2NumericTraits<scalar_t>::weakEpsilon();
      if (endTime - beginTime > eps) {
        // in order to correctly detect the next subsystem (right limit)
        beginTime += eps;

        // integrate controlled system
        dynamicsIntegratorsPtr_->integrate(beginState, beginTime, endTime, stateTrajectory, timeTrajectory, BASE::settings().minTimeStep_,
                                           BASE::settings().absTolODE_, BASE::settings().relTolODE_, maxNumSteps, true);
      } else {
        timeTrajectory.insert(timeTrajectory.end(), 2, endTime);       // integration would have added two points
        stateTrajectory.insert(stateTrajectory.end(), 2, beginState);  // integration would have added two points
      }

      // compute control input trajectory and concatenate to inputTrajectory
      if (BASE::settings().reconstructInputTrajectory_) {
        for (; k_u < timeTrajectory.size(); k_u++) {
          inputTrajectory.emplace_back(systemDynamicsPtr_->controllerPtr()->computeInput(timeTrajectory[k_u], stateTrajectory[k_u]));
        }  // end of k loop
      }

      // a jump has taken place
      if (i < numEvents) {
        eventsPastTheEndIndeces.push_back(stateTrajectory.size());
        // jump map
        systemDynamicsPtr_->computeJumpMap(endTime, stateTrajectory.back(), beginState);
      }
    }  // end of i loop

    // check for the numerical stability
    this->checkNumericalStability(controller, timeTrajectory, eventsPastTheEndIndeces, stateTrajectory, inputTrajectory);

    return stateTrajectory.back();
  }

 private:
  std::shared_ptr<controlled_system_base_t> systemDynamicsPtr_;

  std::shared_ptr<event_handler_t> systemEventHandlersPtr_;

  std::unique_ptr<ode_base_t> dynamicsIntegratorsPtr_;
};

}  // namespace ocs2
