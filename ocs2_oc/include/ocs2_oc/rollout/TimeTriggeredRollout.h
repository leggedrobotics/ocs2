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

#ifndef TIMETRIGGERED_ROLLOUT_OCS2_H_
#define TIMETRIGGERED_ROLLOUT_OCS2_H_

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

  typedef RolloutBase<STATE_DIM, INPUT_DIM> BASE;

  using controller_t = typename BASE::controller_t;
  using size_array_t = typename BASE::size_array_t;
  using scalar_t = typename BASE::scalar_t;
  using scalar_array_t = typename BASE::scalar_array_t;
  using state_vector_t = typename BASE::state_vector_t;
  using state_vector_array_t = typename BASE::state_vector_array_t;
  using input_vector_t = typename BASE::input_vector_t;
  using input_vector_array_t = typename BASE::input_vector_array_t;

  using event_handler_t = SystemEventHandler<STATE_DIM>;
  typedef ControlledSystemBase<STATE_DIM, INPUT_DIM> controlled_system_base_t;

  using logic_rules_machine_t = HybridLogicRulesMachine;

  using ode_base_t = IntegratorBase<STATE_DIM>;

  /**
   * Constructor.
   *
   * @param [in] systemDynamics: The system dynamics for forward rollout.
   * @param [in] rolloutSettings: The rollout settings.
   * @param [in] algorithmName: The algorithm that calls this class (default not defined).
   */
  TimeTriggeredRollout(const controlled_system_base_t& systemDynamics, const Rollout_Settings& rolloutSettings = Rollout_Settings(),
                       const char algorithmName[] = nullptr)

      : BASE(rolloutSettings, algorithmName),
        systemDynamicsPtr_(systemDynamics.clone()),
        systemEventHandlersPtr_(new event_handler_t),
        reconstructInputTrajectory_(rolloutSettings.reconstructInputTrajectory_) {
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
        // TODO(jcarius) the number of steps should not be hardcoded
        dynamicsIntegratorsPtr_.reset(new IntegratorAdamsBashforth<STATE_DIM, 1>(systemDynamicsPtr_, systemEventHandlersPtr_));
        break;
      }
      case (IntegratorType::BULIRSCH_STOER): {
        dynamicsIntegratorsPtr_.reset(new IntegratorBulirschStoer<STATE_DIM>(systemDynamicsPtr_, systemEventHandlersPtr_));
        break;
      }
#if (BOOST_VERSION / 100000 == 1 && BOOST_VERSION / 100 % 1000 > 55)
      case (IntegratorType::ADAMS_BASHFORTH_MOULTON): {
        // TODO(jcarius) the number of steps should not be hardcoded
        dynamicsIntegratorsPtr_.reset(new IntegratorAdamsBashforthMoulton<STATE_DIM, 1>(systemDynamicsPtr_, systemEventHandlersPtr_));
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
  ~TimeTriggeredRollout() = default;

  /**
   * Forward integrate the system dynamics with given controller. It uses the given control policies and initial state,
   * to integrate the system dynamics in time period [initTime, finalTime].
   *
   * @param [in] partitionIndex: Time partition index.
   * @param [in] initTime: The initial time.
   * @param [in] initState: The initial state.
   * @param [in] finalTime: The final time.
   * @param [in] controller: control policy.
   * @param [in] logicRulesMachine: logic rules machine.
   * @param [out] timeTrajectory: The time trajectory stamp.
   * @param [out] eventsPastTheEndIndeces: Indices containing past-the-end index of events trigger.
   * @param [out] stateTrajectory: The state trajectory.
   * @param [out] inputTrajectory: The control input trajectory.
   *
   * @return The final state (state jump is considered if it took place)
   */
  state_vector_t run(const size_t& partitionIndex, const scalar_t& initTime, const state_vector_t& initState, const scalar_t& finalTime,
                     controller_t* controller, logic_rules_machine_t& logicRulesMachine, scalar_array_t& timeTrajectory,
                     size_array_t& eventsPastTheEndIndeces, state_vector_array_t& stateTrajectory,
                     input_vector_array_t& inputTrajectory) override {
    if (initTime > finalTime) {
      throw std::runtime_error("Initial time should be less-equal to final time.");
    }

    if (controller == nullptr) {
      throw std::runtime_error("The input controller is not set.");
    }

    const size_t numEvents = logicRulesMachine.getNumEvents(partitionIndex);
    const size_t numSubsystems = logicRulesMachine.getNumEventCounters(partitionIndex);
    const scalar_array_t& switchingTimes = logicRulesMachine.getSwitchingTimes(partitionIndex);

    // max number of steps for integration
    const size_t maxNumSteps = BASE::settings().maxNumStepsPerSecond_ * std::max(1.0, finalTime - initTime);

    // index of the first subsystem
    size_t beginItr = lookup::findActiveIntervalInTimeArray(switchingTimes, initTime);
    // index of the last subsystem
    size_t finalItr = lookup::findActiveIntervalInTimeArray(switchingTimes, finalTime);

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
    scalar_t beginTime, endTime;
    size_t k_u = 0;  // control input iterator
    for (size_t i = beginItr; i <= finalItr; i++) {
      beginTime = i == beginItr ? initTime : switchingTimes[i];
      endTime = i == finalItr ? finalTime : switchingTimes[i + 1];

      // in order to correctly detect the next subsystem (right limit)
      beginTime += 10 * OCS2NumericTraits<scalar_t>::weakEpsilon();

      // integrate controlled system
      dynamicsIntegratorsPtr_->integrate(beginState, beginTime, endTime, stateTrajectory, timeTrajectory, BASE::settings().minTimeStep_,
                                         BASE::settings().absTolODE_, BASE::settings().relTolODE_, maxNumSteps, true);

      if (reconstructInputTrajectory_) {
        // compute control input trajectory and concatenate to inputTrajectory
        for (; k_u < timeTrajectory.size(); k_u++) {
          inputTrajectory.emplace_back(systemDynamicsPtr_->controllerPtr()->computeInput(timeTrajectory[k_u], stateTrajectory[k_u]));
        }  // end of k loop
      }

      if (i < finalItr) {
        eventsPastTheEndIndeces.push_back(stateTrajectory.size());
        systemDynamicsPtr_->computeJumpMap(timeTrajectory.back(), stateTrajectory.back(), beginState);
      }

    }  // end of i loop

    // If an event has happened at the final time push it to the eventsPastTheEndIndeces
    // numEvents>finalItr means that there the final active subsystem is before an event time.
    // Note: we don't push the state because the input is not yet defined since the next control
    // policy is available)
    bool eventAtFinalTime = numEvents > finalItr && logicRulesMachine.getEventTimes(partitionIndex)[finalItr] <
                                                        finalTime + OCS2NumericTraits<scalar_t>::limitEpsilon();

    // terminal state and event
    state_vector_t terminalState;
    if (eventAtFinalTime) {
      eventsPastTheEndIndeces.push_back(stateTrajectory.size());
      systemDynamicsPtr_->computeJumpMap(timeTrajectory.back(), stateTrajectory.back(), terminalState);
    } else {
      terminalState = stateTrajectory.back();
    }

    // check for the numerical stability
    BASE::checkNumericalStability(partitionIndex, controller, timeTrajectory, eventsPastTheEndIndeces, stateTrajectory, inputTrajectory);

    return terminalState;
  }

 private:
  std::shared_ptr<controlled_system_base_t> systemDynamicsPtr_;

  std::shared_ptr<event_handler_t> systemEventHandlersPtr_;

  std::unique_ptr<ode_base_t> dynamicsIntegratorsPtr_;

  bool reconstructInputTrajectory_;
};

}  // namespace ocs2

#endif /* TIMETRIGGERED_ROLLOUT_OCS2_H_ */
