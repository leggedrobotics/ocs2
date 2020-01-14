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

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <int STATE_DIM>
std::unique_ptr<IntegratorBase<STATE_DIM>> newIntegrator(IntegratorType integratorType,
                                                         const std::shared_ptr<SystemEventHandler<STATE_DIM>>& eventHandlerPtr) {
  switch (integratorType) {
    case (IntegratorType::EULER):
      return std::unique_ptr<IntegratorBase<STATE_DIM>>(new IntegratorEuler<STATE_DIM>(eventHandlerPtr));
    case (IntegratorType::ODE45):
      return std::unique_ptr<IntegratorBase<STATE_DIM>>(new ODE45<STATE_DIM>(eventHandlerPtr));
    case (IntegratorType::ADAMS_BASHFORTH):
      return std::unique_ptr<IntegratorBase<STATE_DIM>>(new IntegratorAdamsBashforth<STATE_DIM, 1>(eventHandlerPtr));
    case (IntegratorType::BULIRSCH_STOER):
      return std::unique_ptr<IntegratorBase<STATE_DIM>>(new IntegratorBulirschStoer<STATE_DIM>(eventHandlerPtr));
    case (IntegratorType::MODIFIED_MIDPOINT):
      return std::unique_ptr<IntegratorBase<STATE_DIM>>(new IntegratorModifiedMidpoint<STATE_DIM>(eventHandlerPtr));
    case (IntegratorType::RK4):
      return std::unique_ptr<IntegratorBase<STATE_DIM>>(new IntegratorRK4<STATE_DIM>(eventHandlerPtr));
    case (IntegratorType::RK5_VARIABLE):
      return std::unique_ptr<IntegratorBase<STATE_DIM>>(new IntegratorRK5Variable<STATE_DIM>(eventHandlerPtr));
#if (BOOST_VERSION / 100000 == 1 && BOOST_VERSION / 100 % 1000 > 55)
    case (IntegratorType::ADAMS_BASHFORTH_MOULTON):
      return std::unique_ptr<IntegratorBase<STATE_DIM>>(new IntegratorAdamsBashforthMoulton<STATE_DIM, 1>(eventHandlerPtr));
#endif
    default:
      throw std::runtime_error("Integrator of type " + integrator_type::toString(integratorType) + " not supported.");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <int STATE_DIM, class Stepper>
void Integrator<STATE_DIM, Stepper>::run_integrate_const(system_func_t system, observer_func_t observer, const state_vector_t& initialState,
                                                         scalar_t startTime, scalar_t finalTime, scalar_t dt) {
  // TODO(mspieler): initializeStepper not used, why?
  // /*
  //  * use a temporary state for initialization, the state returned by initialize is different
  //  * from the real init state (already forward integrated)
  //  */
  // state_vector_t initialStateInternal_init_temp = initialState;
  // initializeStepper(initialStateInternal_init_temp, startTime, dt);

  state_vector_t initialStateInternal = initialState;
  // Ensure that finalTime is included by adding a fraction of dt such that: N * dt <= finalTime < (N + 1) * dt.
  finalTime += 0.1 * dt;
  boost::numeric::odeint::integrate_const(stepper_, system, initialStateInternal, startTime, finalTime, dt, observer);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <int STATE_DIM, class Stepper>
void Integrator<STATE_DIM, Stepper>::run_integrate_adaptive(system_func_t system, observer_func_t observer,
                                                            const state_vector_t& initialState, scalar_t startTime, scalar_t finalTime,
                                                            scalar_t dtInitial, scalar_t AbsTol, scalar_t RelTol) {
  state_vector_t internalStartState = initialState;
  integrate_adaptive_specialized<Stepper>(system, observer, internalStartState, startTime, finalTime, dtInitial, AbsTol, RelTol);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <int STATE_DIM, class Stepper>
void Integrator<STATE_DIM, Stepper>::run_integrate_times(system_func_t system, observer_func_t observer, const state_vector_t& initialState,
                                                         typename scalar_array_t::const_iterator beginTimeItr,
                                                         typename scalar_array_t::const_iterator endTimeItr, scalar_t dtInitial,
                                                         scalar_t AbsTol, scalar_t RelTol) {
  state_vector_t internalStartState = initialState;
  integrate_times_specialized<Stepper>(system, observer, internalStartState, beginTimeItr, endTimeItr, dtInitial, AbsTol, RelTol);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <int STATE_DIM, class Stepper>
template <typename S>
typename std::enable_if<std::is_same<S, runge_kutta_dopri5_t<STATE_DIM>>::value, void>::type
Integrator<STATE_DIM, Stepper>::integrate_adaptive_specialized(system_func_t system, observer_func_t observer, state_vector_t& initialState,
                                                               scalar_t startTime, scalar_t finalTime, scalar_t dtInitial, scalar_t AbsTol,
                                                               scalar_t RelTol) {
  boost::numeric::odeint::integrate_adaptive(boost::numeric::odeint::make_controlled<S>(AbsTol, RelTol), system, initialState, startTime,
                                             finalTime, dtInitial, observer);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <int STATE_DIM, class Stepper>
template <typename S>
typename std::enable_if<!std::is_same<S, runge_kutta_dopri5_t<STATE_DIM>>::value, void>::type
Integrator<STATE_DIM, Stepper>::integrate_adaptive_specialized(system_func_t system, observer_func_t observer, state_vector_t& initialState,
                                                               scalar_t startTime, scalar_t finalTime, scalar_t dtInitial, scalar_t AbsTol,
                                                               scalar_t RelTol) {
  boost::numeric::odeint::integrate_adaptive(stepper_, system, initialState, startTime, finalTime, dtInitial, observer);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <int STATE_DIM, class Stepper>
template <typename S>
typename std::enable_if<std::is_same<S, runge_kutta_dopri5_t<STATE_DIM>>::value, void>::type
Integrator<STATE_DIM, Stepper>::integrate_times_specialized(system_func_t system, observer_func_t observer, state_vector_t& initialState,
                                                            typename scalar_array_t::const_iterator beginTimeItr,
                                                            typename scalar_array_t::const_iterator endTimeItr, scalar_t dtInitial,
                                                            scalar_t AbsTol, scalar_t RelTol) {
#if (BOOST_VERSION / 100000 == 1 && BOOST_VERSION / 100 % 1000 > 60)
  boost::numeric::odeint::max_step_checker maxStepChecker(
      std::numeric_limits<int>::max());  // maxNumSteps is already checked by event handler.

  boost::numeric::odeint::integrate_times(boost::numeric::odeint::make_controlled<S>(AbsTol, RelTol), system, initialState, beginTimeItr,
                                          endTimeItr, dtInitial, observer, maxStepChecker);
#else
  boost::numeric::odeint::integrate_times(boost::numeric::odeint::make_controlled<S>(AbsTol, RelTol), system, initialState, beginTimeItr,
                                          endTimeItr, dtInitial, observer);
#endif
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <int STATE_DIM, class Stepper>
template <typename S>
typename std::enable_if<!std::is_same<S, runge_kutta_dopri5_t<STATE_DIM>>::value, void>::type
Integrator<STATE_DIM, Stepper>::integrate_times_specialized(system_func_t system, observer_func_t observer, state_vector_t& initialState,
                                                            typename scalar_array_t::const_iterator beginTimeItr,
                                                            typename scalar_array_t::const_iterator endTimeItr, scalar_t dtInitial,
                                                            scalar_t AbsTol, scalar_t RelTol) {
#if (BOOST_VERSION / 100000 == 1 && BOOST_VERSION / 100 % 1000 > 60)
  boost::numeric::odeint::max_step_checker maxStepChecker(
      std::numeric_limits<int>::max());  // maxNumSteps is already checked by event handler.

  boost::numeric::odeint::integrate_times(stepper_, system, initialState, beginTimeItr, endTimeItr, dtInitial, observer, maxStepChecker);

#else
  boost::numeric::odeint::integrate_times(stepper_, system, initialState, beginTimeItr, endTimeItr, dtInitial, observer);
#endif
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <int STATE_DIM, class Stepper>
template <typename S>
typename std::enable_if<std::is_same<S, runge_kutta_dopri5_t<STATE_DIM>>::value, void>::type
Integrator<STATE_DIM, Stepper>::initializeStepper(state_vector_t& initialState, scalar_t t, scalar_t dt) {
  /**do nothing, runge_kutta_5_t does not have a init method */
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <int STATE_DIM, class Stepper>
template <typename S>
typename std::enable_if<!(std::is_same<S, runge_kutta_dopri5_t<STATE_DIM>>::value), void>::type
Integrator<STATE_DIM, Stepper>::initializeStepper(state_vector_t& initialState, scalar_t t, scalar_t dt) {
  stepper_.initialize(runge_kutta_dopri5_t<STATE_DIM>(), system, initialState, t, dt);
}

}  // namespace ocs2
