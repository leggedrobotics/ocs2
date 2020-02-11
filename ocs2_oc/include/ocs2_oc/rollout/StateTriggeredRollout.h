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

#include <ocs2_core/control/StateBasedLinearController.h>
#include <ocs2_core/dynamics/ControlledSystemBase.h>
#include <ocs2_core/integration/Integrator.h>
#include <ocs2_core/integration/StateTriggeredEventHandler.h>

#include "ocs2_oc/rollout/RolloutBase.h"
#include "ocs2_oc/rollout/RootFinder.h"

namespace ocs2 {

/**
 * This class is an interface class for forward rollout of the system dynamics.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class StateTriggeredRollout : public RolloutBase<STATE_DIM, INPUT_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using BASE = RolloutBase<STATE_DIM, INPUT_DIM>;

  using controller_t = typename BASE::controller_t;
  using size_array_t = typename BASE::size_array_t;
  using scalar_t = typename BASE::scalar_t;
  using scalar_array_t = typename BASE::scalar_array_t;
  using state_vector_t = typename BASE::state_vector_t;
  using state_vector_array_t = typename BASE::state_vector_array_t;
  using input_vector_t = typename BASE::input_vector_t;
  using input_vector_array_t = typename BASE::input_vector_array_t;
  using time_interval_array_t = typename BASE::time_interval_array_t;
  using dynamic_vector_t = typename BASE::dynamic_vector_t;

  using state_triggered_event_handler_t = StateTriggeredEventHandler<STATE_DIM>;
  using controlled_system_base_t = ControlledSystemBase<STATE_DIM, INPUT_DIM>;
  using ode_base_t = IntegratorBase<STATE_DIM>;

  using trajectory_spreading_controller_t = stateBasedLinearController<STATE_DIM, INPUT_DIM>;
  /**
   * Constructor.
   *
   * @param [in] systemDynamics: The system dynamics for forward rollout.
   * @param [in] rolloutSettings: The rollout settings.
   */
  explicit StateTriggeredRollout(const controlled_system_base_t& systemDynamics, Rollout_Settings rolloutSettings = Rollout_Settings())
      : BASE(std::move(rolloutSettings)),
        systemDynamicsPtr_(systemDynamics.clone()),
        systemEventHandlersPtr_(new state_triggered_event_handler_t(this->settings().minTimeStep_)) {
    // construct dynamicsIntegratorsPtr
    dynamicsIntegratorPtr_ = std::move(newIntegrator<STATE_DIM>(this->settings().integratorType_, systemEventHandlersPtr_));
  }

  /**
   * Default destructor.
   */
  ~StateTriggeredRollout() override = default;

  StateTriggeredRollout(const StateTriggeredRollout&) = delete;

  StateTriggeredRollout& operator=(const StateTriggeredRollout&) = delete;

  StateTriggeredRollout<STATE_DIM, INPUT_DIM>* clone() const override {
    return new StateTriggeredRollout<STATE_DIM, INPUT_DIM>(*systemDynamicsPtr_, this->settings());
  }

  /**
   * Returns the underlying dynamics
   */
  controlled_system_base_t* systemDynamicsPtr() { return systemDynamicsPtr_.get(); }

 protected:
  state_vector_t runImpl(time_interval_array_t timeIntervalArray, const state_vector_t& initState, controller_t* controller,
                         scalar_array_t& timeTrajectory, size_array_t& eventsPastTheEndIndeces, state_vector_array_t& stateTrajectory,
                         input_vector_array_t& inputTrajectory) override {
    if (!controller) {
      throw std::runtime_error("The input controller is not set.");
    }

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
    eventsPastTheEndIndeces.clear();
    eventsPastTheEndIndeces.reserve(maxNumSteps);

    // set controller
    trajectory_spreading_controller_t trajectorySpreadingController;
    if (this->settings().useTrajectorySpreadingController_) {
      trajectorySpreadingController.setController(controller);
      systemDynamicsPtr_->setController(&trajectorySpreadingController);
    } else {
      systemDynamicsPtr_->setController(controller);
    }

    // reset function calls counter
    systemDynamicsPtr_->resetNumFunctionCalls();

    // reset the event class
    systemEventHandlersPtr_->reset();

    state_vector_t x0 = initState;
    int k_u = 0;  // control input iterator

    size_t eventID = 0;
    scalar_t t0 = timeIntervalArray.front().first;
    scalar_t t1 = timeIntervalArray.back().second;
    const scalar_t finalTime = t1;  // stored separately due to overwriting t1 when refining

    bool refining = false;
    int singleEventIterations = 0;  // iterations for a single event
    int numTotalIterations = 0;     // overall number of iterations

    RootFinder rootFinder(this->settings().rootFindingAlgorithm_);  // root-finding algorithm

    while (true) {  // keeps looping until end time condition is fulfilled, after which the loop is broken
      bool triggered = false;
      try {
        Observer<STATE_DIM> observer(&stateTrajectory, &timeTrajectory);  // concatenate trajectory
        dynamicsIntegratorPtr_->integrate_adaptive(*systemDynamicsPtr_, observer, x0, t0, t1, this->settings().minTimeStep_,
                                                   this->settings().absTolODE_, this->settings().relTolODE_, maxNumSteps);
      } catch (const size_t& e) {
        eventID = e;
        triggered = true;
      }
      // calculate guard surface value of last query state and time
      const scalar_t queryTime = timeTrajectory.back();
      const state_vector_t queryState = stateTrajectory.back();
      dynamic_vector_t guardSurfaces;
      systemDynamicsPtr_->computeGuardSurfaces(queryTime, queryState, guardSurfaces);
      const scalar_t queryGuard = guardSurfaces[eventID];

      // accuracy conditions on the obtained query guard and width of time window
      const bool guardAccuracyCondition = std::fabs(queryGuard) < this->settings().absTolODE_;
      const bool timeAccuracyCondition = std::fabs(t1 - t0) < this->settings().absTolODE_;
      const bool accuracyCondition = guardAccuracyCondition || timeAccuracyCondition;
      // condition to check whether max number of iterations has not been reached, to prevent an infinite loop
      const bool maxNumIterationsReached = singleEventIterations >= this->settings().maxSingleEventIterations_;

      // remove the element past the guard surface if the event handler was triggered
      // (Due to checking in EventHandler this can only happen to the last element of the trajectory)
      // Exception is when the element is outside the guardSurface but within tolerance
      if (triggered && !accuracyCondition) {
        stateTrajectory.pop_back();
        timeTrajectory.pop_back();
      }
      triggered = false;

      // compute control input trajectory and concatenate to inputTrajectory
      if (this->settings().reconstructInputTrajectory_) {
        for (; k_u < timeTrajectory.size(); k_u++) {
          inputTrajectory.emplace_back(systemDynamicsPtr_->controllerPtr()->computeInput(timeTrajectory[k_u], stateTrajectory[k_u]));
        }  // end of k loop
      }

      // end time condition to detect end of simulation, means iteration procedure is done
      if (numerics::almost_eq(finalTime, timeTrajectory.back())) {
        break;
      }

      // accuracy condition for event refinement. If sufficiently accurate crossing location has been determined
      if (accuracyCondition || maxNumIterationsReached) {
        // set new begin/end time and begin state
        t0 = queryTime + OCS2NumericTraits<scalar_t>::weakEpsilon();
        t1 = finalTime;
        // compute jump
        systemDynamicsPtr_->computeJumpMap(queryTime, queryState, x0);

        // append the event to array with event indices
        eventsPastTheEndIndeces.push_back(stateTrajectory.size());

        // determine guard surface cross value and update the eventHandler
        dynamic_vector_t guardSurfacesCross;
        systemDynamicsPtr_->computeGuardSurfaces(t0, x0, guardSurfacesCross);
        // updates the last event triggering times of Event Handler
        systemEventHandlersPtr_->setLastEvent(t0, guardSurfacesCross);

        // reset relevant boolean and counter
        refining = false;
        singleEventIterations = 0;
      } else {           // otherwise keep or start refining
        if (refining) {  // apply the rules of the root-finding method to continue refining
          rootFinder.updateBracket(queryTime, queryGuard);
        } else {  // properly configure root-finding method to start refining
          const scalar_t& timeBefore = timeTrajectory.back();
          const state_vector_t& stateBefore = stateTrajectory.back();
          dynamic_vector_t guardSurfacesBefore;
          systemDynamicsPtr_->computeGuardSurfaces(timeBefore, stateBefore, guardSurfacesBefore);
          const scalar_t& guardBefore = guardSurfacesBefore[eventID];

          rootFinder.setInitBracket(timeBefore, queryTime, guardBefore, queryGuard);
          refining = true;
        }
        t1 = rootFinder.getNewQuery();
        t0 = timeTrajectory.back();
        x0 = stateTrajectory.back();

        stateTrajectory.pop_back();
        timeTrajectory.pop_back();
        inputTrajectory.pop_back();
        k_u--;
      }
      singleEventIterations++;
      numTotalIterations++;
    }  // end of while loop

    // check for the numerical stability
    this->checkNumericalStability(controller, timeTrajectory, eventsPastTheEndIndeces, stateTrajectory, inputTrajectory);

    return stateTrajectory.back();
  }  // end of function

 private:
  std::unique_ptr<controlled_system_base_t> systemDynamicsPtr_;

  std::shared_ptr<state_triggered_event_handler_t> systemEventHandlersPtr_;

  std::unique_ptr<ode_base_t> dynamicsIntegratorPtr_;
};

}  // namespace ocs2
