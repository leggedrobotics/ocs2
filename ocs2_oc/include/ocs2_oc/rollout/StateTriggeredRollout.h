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
#include <ocs2_core/logic/machine/HybridLogicRulesMachine.h>

#include "RolloutBase.h"

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

  using event_handler_t = SystemEventHandler<STATE_DIM>;
  using state_triggered_event_handler_t = StateTriggeredEventHandler<STATE_DIM>;
  using controlled_system_base_t = ControlledSystemBase<STATE_DIM, INPUT_DIM>;

  using logic_rules_machine_t = HybridLogicRulesMachine;
  //	using hybrid_logic_rules_machine_t = HybridLogicRulesMachine;

  using ode_base_t = IntegratorBase<STATE_DIM>;

  /**
   * Constructor.
   *
   * @param [in] systemDynamics: The system dynamics for forward rollout.
   * @param [in] rolloutSettings: The rollout settings.
   */

  explicit StateTriggeredRollout(const controlled_system_base_t& systemDynamics, const Rollout_Settings& rolloutSettings = Rollout_Settings())
      : BASE(std::move(rolloutSettings)),
        systemDynamicsPtr_(systemDynamics.clone()),
        systemEventHandlersPtr_(new state_triggered_event_handler_t),
        dynamicsIntegratorPtr_(this->settings().integratorType_) {}

  /**
   * Default destructor.
   */
  ~StateTriggeredRollout() = default;

  /**
   * Returns the underlying dynamics
   */

  controlled_system_base_t* systemDynamicsPtr() {return systemDynamicsPtr_.get(); }

  StateTriggeredRollout<STATE_DIM, INPUT_DIM>* clone() const override {
    return new StateTriggeredRollout<STATE_DIM, INPUT_DIM> (*systemDynamicsPtr_, this->settings());
  }

  /**
   * Forward integrate the system dynamics with given controller. It uses the given control policies and initial state,
   * to integrate the system dynamics in time period [initTime, finalTime].
   *
   * @param [in] partitionIndex: Time partition index.
   * @param [in] initTime: The initial time.
   * @param [in] initState: The initial state.
   * @param [in] finalTime: The final time.
   * @param [in] controller: control policy.
   * @param [in] hybridLlogicRulesMachine: logic rules machine.
   * @param [out] timeTrajectory: The time trajectory stamp.
   * @param [out] eventsPastTheEndIndeces: Indices containing past-the-end index of events trigger.
   * @param [out] stateTrajectory: The state trajectory.
   * @param [out] inputTrajectory: The control input trajectory.
   *
   * @return The final state (state jump is considered if it took place)
   */

 protected:
   state_vector_t runImpl(time_interval_array_t timeIntervalArray, const state_vector_t& initState, controller_t* controller,
                          scalar_array_t& timeTrajectory, size_array_t& eventsPastTheEndIndeces, state_vector_array_t& stateTrajectory,
                          input_vector_array_t& inputTrajectory) override {
    //
    //		scalar_array_t guardSurfacesValues;
    //		scalar_array_t eventTimes;
    //		size_array_t subsystemID;
    //
    //		if (initTime > finalTime)
    //			throw std::runtime_error("Initial time should be less-equal to final time.");
    //
    //		if (controller.empty() == true)
    //			throw std::runtime_error("The input controller is empty.");
    //
    //		if (eventTimes.empty()==false && guardSurfacesValues.empty()==true)
    //			throw std::runtime_error("Since the event times array is not empty, "
    //					"the last update of the guard functions value should be provided.");
    //
    //		// max number of steps for integration
    //		const size_t maxNumSteps =
    //				BASE::settings().maxNumStepsPerSecond_ * std::max(1.0, finalTime-initTime);
    //
    //		// clearing the output trajectories
    //		timeTrajectory.clear();
    //		timeTrajectory.reserve(maxNumSteps+1);
    //		stateTrajectory.clear();
    //		stateTrajectory.reserve(maxNumSteps+1);
    //		inputTrajectory.clear();
    //		inputTrajectory.reserve(maxNumSteps+1);
    //		eventsPastTheEndIndeces.clear();
    //
    //		// initialize the model and set controller
    //		if (controller.empty()==false) {
    //			// init Hybrid Logic Machine
    //			static_cast<hybrid_logic_rules_machine_t>(hybridLlogicRulesMachine).initLogicMachine(partitionIndex);
    ////			std::cerr << std::endl << "+++++++++++++ partitionIndex: " << partitionIndex;
    ////			static_cast<hybrid_logic_rules_machine_t>(hybridLlogicRulesMachine).display();
    //			// set controller
    //			systemDynamicsPtr_->setController(controller);
    //			// reset function calls counter
    //			systemDynamicsPtr_->resetNumFunctionCalls();
    //			// Set event times control parameters
    //			if (eventTimes.empty()==true)
    //				systemEventHandlersPtr_->setEventTimesGuard(
    //						BASE::settings().minEventTimeDifference_);
    //			else
    //				systemEventHandlersPtr_->setEventTimesGuard(
    //						BASE::settings().minEventTimeDifference_, eventTimes.back(), guardSurfacesValues);
    //		}
    //
    //		// initial values of the guard surfaces
    //		if (subsystemID.empty()==true) {
    //			size_t activeSubsystem = 0;
    //			scalar_array_t initGuardSurfacesValue;
    //			systemDynamicsPtr_->computeGuardSurfaces(initTime, initState, initGuardSurfacesValue);
    //			for (size_t i=0; i<initGuardSurfacesValue.size(); i++)
    //				if (initGuardSurfacesValue[i]<0)
    //					activeSubsystem = i;
    //
    //			subsystemID.push_back(activeSubsystem);
    //		}
    //
    //		scalar_t t0 = initTime;
    //		state_vector_t x0 = initState;
    //
    //		while (t0 < finalTime-OCS2NumericTraits<scalar_t>::weakEpsilon()) {
    //
    //			try {
    //				// integrate controlled system
    //				dynamicsIntegratorPtr_->integrate(
    //						x0, t0, finalTime,
    //						stateTrajectory,
    //						timeTrajectory,
    //						BASE::settings().minTimeStep_,
    //						BASE::settings().absTolODE_,
    //						BASE::settings().relTolODE_,
    //						maxNumSteps,
    //						true);
    //
    //			} catch (const size_t& eventID) {
    //
    //				eventsPastTheEndIndeces.push_back( timeTrajectory.size() );
    //				systemDynamicsPtr_->computeJumpMap(timeTrajectory.back(), stateTrajectory.back(), x0);
    //
    //				eventTimes.push_back(timeTrajectory.back());
    //				subsystemID.push_back(eventID);
    //
    //				static_cast<hybrid_logic_rules_machine_t>(hybridLlogicRulesMachine).push_back(partitionIndex,
    // timeTrajectory.back(), eventID);
    ////				static_cast<hybrid_logic_rules_machine_t>(hybridLlogicRulesMachine).display();
    //			}
    //
    //			t0 = timeTrajectory.back();
    //
    //		}  // end of while loop
    //
    //		// compute control input trajectory and concatenate to inputTrajectory
    //		for (size_t k_u=0; k_u<timeTrajectory.size(); k_u++) {
    //			inputTrajectory.emplace_back( systemDynamicsPtr_->computeInput(
    //					timeTrajectory[k_u], stateTrajectory[k_u]) );
    //		} // end of k loop
    //
    //		// get the guardSurfacesValues
    //		guardSurfacesValues = systemEventHandlersPtr_->getGuardSurfacesValues();
    //
    //		return stateTrajectory.back();

    return state_vector_t::Zero();
  }

   /**
      * Constructs dynamicsIntegratorPtr_ based on the integratorType.
      *
      * @param [in] integratorType: Integrator type.
      */
     void constructDynamicsIntegrator(IntegratorType integratorType) {
       switch (integratorType) {
         case (IntegratorType::EULER): {
           dynamicsIntegratorPtr_.reset(new IntegratorEuler<STATE_DIM>(systemDynamicsPtr_, systemEventHandlersPtr_));
           break;
         }
         case (IntegratorType::MODIFIED_MIDPOINT): {
           dynamicsIntegratorPtr_.reset(new IntegratorModifiedMidpoint<STATE_DIM>(systemDynamicsPtr_, systemEventHandlersPtr_));
           break;
         }
         case (IntegratorType::RK4): {
           dynamicsIntegratorPtr_.reset(new IntegratorRK4<STATE_DIM>(systemDynamicsPtr_, systemEventHandlersPtr_));
           break;
         }
         case (IntegratorType::RK5_VARIABLE): {
           dynamicsIntegratorPtr_.reset(new IntegratorRK5Variable<STATE_DIM>(systemDynamicsPtr_, systemEventHandlersPtr_));
           break;
         }
         case (IntegratorType::ODE45): {
           dynamicsIntegratorPtr_.reset(new ODE45<STATE_DIM>(systemDynamicsPtr_, systemEventHandlersPtr_));
           break;
         }
         case (IntegratorType::ADAMS_BASHFORTH): {
           const size_t numberSteps = 1;
           dynamicsIntegratorPtr_.reset(new IntegratorAdamsBashforth<STATE_DIM, numberSteps>(systemDynamicsPtr_, systemEventHandlersPtr_));
           break;
         }
         case (IntegratorType::BULIRSCH_STOER): {
           dynamicsIntegratorPtr_.reset(new IntegratorBulirschStoer<STATE_DIM>(systemDynamicsPtr_, systemEventHandlersPtr_));
           break;
         }
   #if (BOOST_VERSION / 100000 == 1 && BOOST_VERSION / 100 % 1000 > 55)
         case (IntegratorType::ADAMS_BASHFORTH_MOULTON): {
           const size_t numberSteps = 1;  // maximum is 8
           dynamicsIntegratorPtr_.reset(
               new IntegratorAdamsBashforthMoulton<STATE_DIM, numberSteps>(systemDynamicsPtr_, systemEventHandlersPtr_));
           break;
         }
   #endif
         default: {
           throw std::runtime_error("Integrator of type " +
                                    std::to_string(static_cast<std::underlying_type<IntegratorType>::type>(integratorType)) +
                                    " not supported in TimeTriggeredRollout.");
         }
       }
     }

 private:
  std::shared_ptr<controlled_system_base_t> systemDynamicsPtr_;

  std::shared_ptr<event_handler_t> systemEventHandlersPtr_;

  std::unique_ptr<ode_base_t> dynamicsIntegratorPtr_;
};

}  // namespace ocs2
