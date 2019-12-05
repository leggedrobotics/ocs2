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
                                                         const std::shared_ptr<OdeBase<STATE_DIM>>& systemPtr,
                                                         const std::shared_ptr<SystemEventHandler<STATE_DIM>>& eventHandlerPtr) {
  IntegratorBase<STATE_DIM>* integrator = nullptr;
  switch (integratorType) {
    case (IntegratorType::EULER):
      integrator = new IntegratorEuler<STATE_DIM>(systemPtr, eventHandlerPtr);
      break;
    case (IntegratorType::ODE45):
      integrator = new ODE45<STATE_DIM>(systemPtr, eventHandlerPtr);
      break;
    case (IntegratorType::ADAMS_BASHFORTH): {
      const size_t numberSteps = 1;
      integrator = new IntegratorAdamsBashforth<STATE_DIM, numberSteps>(systemPtr, eventHandlerPtr);
      break;
    }
    case (IntegratorType::BULIRSCH_STOER):
      integrator = new IntegratorBulirschStoer<STATE_DIM>(systemPtr, eventHandlerPtr);
      break;
    case (IntegratorType::MODIFIED_MIDPOINT):
      integrator = new IntegratorModifiedMidpoint<STATE_DIM>(systemPtr, eventHandlerPtr);
      break;
    case (IntegratorType::RK4):
      integrator = new IntegratorRK4<STATE_DIM>(systemPtr, eventHandlerPtr);
      break;
    case (IntegratorType::RK5_VARIABLE):
      integrator = new IntegratorRK5Variable<STATE_DIM>(systemPtr, eventHandlerPtr);
      break;
#if (BOOST_VERSION / 100000 == 1 && BOOST_VERSION / 100 % 1000 > 55)
    case (IntegratorType::ADAMS_BASHFORTH_MOULTON): {
      const size_t numberSteps = 1;  // maximum is 8
      integrator = new IntegratorAdamsBashforthMoulton<STATE_DIM, numberSteps>(systemPtr, eventHandlerPtr);
      break;
    }
#endif
    default:
      throw std::runtime_error("Integrator of type " + toString(integratorType) + " not supported.");
  }
  return std::unique_ptr<IntegratorBase<STATE_DIM>>(integrator);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <int STATE_DIM, class Stepper>
Integrator<STATE_DIM, Stepper>::Integrator(const std::shared_ptr<OdeBase<STATE_DIM>>& systemPtr,
                                           const std::shared_ptr<SystemEventHandler<STATE_DIM>>& eventHandlerPtr)

    : BASE(systemPtr, eventHandlerPtr), stepperPtr_(new Stepper) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <int STATE_DIM, class Stepper>
void Integrator<STATE_DIM, Stepper>::run_integrate_const(const state_vector_t& initialState, const scalar_t& startTime,
                                                         const scalar_t& finalTime, scalar_t dt, observer_func_t observerFunc) {
  // TODO(mspieler): initializeStepper not used, why?
  // /*
  //  * use a temporary state for initialization, the state returned by initialize is different
  //  * from the real init state (already forward integrated)
  //  */
  // state_vector_t initialStateInternal_init_temp = initialState;
  // scalar_t startTime_temp = startTime;
  // initializeStepper(initialStateInternal_init_temp, startTime_temp, dt);

  state_vector_t initialStateInternal = initialState;
  // Ensure that finalTime is included by adding a fraction of dt.
  boost::numeric::odeint::integrate_const(*stepperPtr_, BASE::systemFunction_, initialStateInternal, startTime, finalTime + 0.1 * dt, dt,
                                          observerFunc);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <int STATE_DIM, class Stepper>
void Integrator<STATE_DIM, Stepper>::run_integrate_adaptive(const state_vector_t& initialState, const scalar_t& startTime,
                                                            const scalar_t& finalTime, scalar_t dtInitial, scalar_t AbsTol, scalar_t RelTol,
                                                            observer_func_t observerFunc) {
  state_vector_t internalStartState = initialState;
  integrate_adaptive_specialized<Stepper>(internalStartState, startTime, finalTime, dtInitial, AbsTol, RelTol, observerFunc);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <int STATE_DIM, class Stepper>
void Integrator<STATE_DIM, Stepper>::run_integrate_times(const state_vector_t& initialState,
                                                         typename scalar_array_t::const_iterator beginTimeItr,
                                                         typename scalar_array_t::const_iterator endTimeItr, scalar_t dtInitial,
                                                         scalar_t AbsTol, scalar_t RelTol, observer_func_t observerFunc) {
  state_vector_t internalStartState = initialState;
  integrate_times_specialized<Stepper>(internalStartState, beginTimeItr, endTimeItr, dtInitial, AbsTol, RelTol, observerFunc);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <int STATE_DIM, class Stepper>
template <typename S>
typename std::enable_if<std::is_same<S, runge_kutta_dopri5_t<STATE_DIM>>::value, void>::type
Integrator<STATE_DIM, Stepper>::integrate_adaptive_specialized(state_vector_t& initialState, const scalar_t& startTime,
                                                               const scalar_t& finalTime, scalar_t dtInitial, scalar_t AbsTol,
                                                               scalar_t RelTol, observer_func_t observerFunc) {
  boost::numeric::odeint::integrate_adaptive(boost::numeric::odeint::make_controlled<S>(AbsTol, RelTol), BASE::systemFunction_,
                                             initialState, startTime, finalTime, dtInitial, observerFunc);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <int STATE_DIM, class Stepper>
template <typename S>
typename std::enable_if<!std::is_same<S, runge_kutta_dopri5_t<STATE_DIM>>::value, void>::type
Integrator<STATE_DIM, Stepper>::integrate_adaptive_specialized(state_vector_t& initialState, const scalar_t& startTime,
                                                               const scalar_t& finalTime, scalar_t dtInitial, scalar_t AbsTol,
                                                               scalar_t RelTol, observer_func_t observerFunc) {
  boost::numeric::odeint::integrate_adaptive(*stepperPtr_, BASE::systemFunction_, initialState, startTime, finalTime, dtInitial,
                                             observerFunc);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <int STATE_DIM, class Stepper>
template <typename S>
typename std::enable_if<std::is_same<S, runge_kutta_dopri5_t<STATE_DIM>>::value, void>::type
Integrator<STATE_DIM, Stepper>::integrate_times_specialized(state_vector_t& initialState,
                                                            typename scalar_array_t::const_iterator beginTimeItr,
                                                            typename scalar_array_t::const_iterator endTimeItr, scalar_t dtInitial,
                                                            scalar_t AbsTol, scalar_t RelTol, observer_func_t observerFunc) {
#if (BOOST_VERSION / 100000 == 1 && BOOST_VERSION / 100 % 1000 > 60)
  boost::numeric::odeint::max_step_checker maxStepChecker(
      std::numeric_limits<int>::max());  // maxNumSteps is already checked by event handler.

  boost::numeric::odeint::integrate_times(boost::numeric::odeint::make_controlled<S>(AbsTol, RelTol), BASE::systemFunction_, initialState,
                                          beginTimeItr, endTimeItr, dtInitial, observerFunc, maxStepChecker);
#else
  boost::numeric::odeint::integrate_times(boost::numeric::odeint::make_controlled<S>(AbsTol, RelTol), BASE::systemFunction_, initialState,
                                          beginTimeItr, endTimeItr, dtInitial, observerFunc);
#endif
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <int STATE_DIM, class Stepper>
template <typename S>
typename std::enable_if<!std::is_same<S, runge_kutta_dopri5_t<STATE_DIM>>::value, void>::type
Integrator<STATE_DIM, Stepper>::integrate_times_specialized(state_vector_t& initialState,
                                                            typename scalar_array_t::const_iterator beginTimeItr,
                                                            typename scalar_array_t::const_iterator endTimeItr, scalar_t dtInitial,
                                                            scalar_t AbsTol, scalar_t RelTol, observer_func_t observerFunc) {
#if (BOOST_VERSION / 100000 == 1 && BOOST_VERSION / 100 % 1000 > 60)
  boost::numeric::odeint::max_step_checker maxStepChecker(
      std::numeric_limits<int>::max());  // maxNumSteps is already checked by event handler.

  boost::numeric::odeint::integrate_times(*stepperPtr_, BASE::systemFunction_, initialState, beginTimeItr, endTimeItr, dtInitial,
                                          observerFunc, maxStepChecker);

#else
  boost::numeric::odeint::integrate_times(*stepperPtr_, BASE::systemFunction_, initialState, beginTimeItr, endTimeItr, dtInitial,
                                          observerFunc);
#endif
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <int STATE_DIM, class Stepper>
template <typename S>
typename std::enable_if<std::is_same<S, runge_kutta_dopri5_t<STATE_DIM>>::value, void>::type
Integrator<STATE_DIM, Stepper>::initializeStepper(state_vector_t& initialState, scalar_t& t, scalar_t dt) {
  /**do nothing, runge_kutta_5_t does not have a init method */
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <int STATE_DIM, class Stepper>
template <typename S>
typename std::enable_if<!(std::is_same<S, runge_kutta_dopri5_t<STATE_DIM>>::value), void>::type
Integrator<STATE_DIM, Stepper>::initializeStepper(state_vector_t& initialState, scalar_t& t, scalar_t dt) {
  stepperPtr_->initialize(runge_kutta_dopri5_t<STATE_DIM>(), BASE::systemFunction_, initialState, t, dt);
}

}  // namespace ocs2
