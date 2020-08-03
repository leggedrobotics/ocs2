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

#include <cmath>
#include <limits>
#include <type_traits>

#include <boost/numeric/odeint.hpp>

#include <ocs2_core/integration/Integrator.h>
#include <ocs2_core/integration/eigenIntegration.h>
#include <ocs2_core/integration/steppers.h>

namespace ocs2 {

/**
 * Integrator class for autonomous systems.
 * @tparam Stepper: Stepper class type to be used.
 */
template <class Stepper>
class Integrator final : public IntegratorBase {
 public:
  using observer_func_t = typename IntegratorBase::observer_func_t;
  using system_func_t = typename IntegratorBase::system_func_t;

  /**
   * Default constructor
   */
  explicit Integrator(std::shared_ptr<SystemEventHandler> eventHandlerPtr = nullptr) : IntegratorBase(std::move(eventHandlerPtr)){};

  /**
   * Default destructor
   */
  ~Integrator() override = default;

 private:
  /**
   * Equidistant integration based on initial and final time as well as step length.
   *
   * @param [in] system: System function
   * @param [in] observer: Observer callback
   * @param [in] initialState: Initial state.
   * @param [in] startTime: Initial time.
   * @param [in] finalTime: Final time.
   * @param [in] dt: Time step.
   */
  void runIntegrateConst(system_func_t system, observer_func_t observer, const vector_t& initialState, scalar_t startTime,
                         scalar_t finalTime, scalar_t dt) override;

  /**
   * Adaptive time integration based on start time and final time.
   *
   * @param [in] system: System function
   * @param [in] observer: Observer callback
   * @param [in] initialState: Initial state.
   * @param [in] startTime: Initial time.
   * @param [in] finalTime: Final time.
   * @param [in] dtInitial: Initial time step.
   * @param [in] AbsTol: The absolute tolerance error for ode solver.
   * @param [in] RelTol: The relative tolerance error for ode solver.
   */
  void runIntegrateAdaptive(system_func_t system, observer_func_t observer, const vector_t& initialState, scalar_t startTime,
                            scalar_t finalTime, scalar_t dtInitial, scalar_t AbsTol, scalar_t RelTol) override;

  /**
   * Output integration based on a given time trajectory.
   *
   * @param [in] system: System function
   * @param [in] observer: Observer callback
   * @param [in] initialState: Initial state.
   * @param [in] beginTimeItr: The iterator to the beginning of the time stamp trajectory.
   * @param [in] endTimeItr: The iterator to the end of the time stamp trajectory.
   * @param [in] dtInitial: Initial time step.
   * @param [in] AbsTol: The absolute tolerance error for ode solver.
   * @param [in] RelTol: The relative tolerance error for ode solver.
   */
  void runIntegrateTimes(system_func_t system, observer_func_t observer, const vector_t& initialState,
                         typename scalar_array_t::const_iterator beginTimeItr, typename scalar_array_t::const_iterator endTimeItr,
                         scalar_t dtInitial, scalar_t AbsTol, scalar_t RelTol) override;

  /**
   * Integrate adaptive specialized.
   *
   * @tparam S: stepper type.
   * @param [in] system: System function
   * @param [in] observer: Observer callback
   * @param [in] initialState: Initial state.
   * @param [in] startTime: Initial time.
   * @param [in] finalTime: Final time.
   * @param [in] dtInitial: Initial time step.
   * @param [in] AbsTol: The absolute tolerance error for ode solver.
   * @param [in] RelTol: The relative tolerance error for ode solver.
   */
  template <typename S>
  typename std::enable_if<std::is_same<S, runge_kutta_dopri5_t>::value, void>::type integrateAdaptiveSpecialized(
      system_func_t system, observer_func_t observer, vector_t& initialState, scalar_t startTime, scalar_t finalTime, scalar_t dtInitial,
      scalar_t AbsTol, scalar_t RelTol);

  /**
   * Integrate adaptive specialized,
   *
   * @tparam S: stepper type.
   * @param [in] system: System function
   * @param [in] observer: Observer callback
   * @param [in] initialState: Initial state.
   * @param [in] startTime: Initial time.
   * @param [in] finalTime: Final time.
   * @param [in] dtInitial: Initial time step.
   * @param [in] AbsTol: The absolute tolerance error for ode solver.
   * @param [in] RelTol: The relative tolerance error for ode solver.
   */
  template <typename S>
  typename std::enable_if<!std::is_same<S, runge_kutta_dopri5_t>::value, void>::type integrateAdaptiveSpecialized(
      system_func_t system, observer_func_t observer, vector_t& initialState, scalar_t startTime, scalar_t finalTime, scalar_t dtInitial,
      scalar_t AbsTol, scalar_t RelTol);

  /**
   * Integrate times specialized function
   * @tparam S: stepper type.
   * @param [in] system: System function
   * @param [in] observer: Observer callback
   * @param [in] initialState: Initial state.
   * @param [in] beginTimeItr: The iterator to the beginning of the time stamp trajectory.
   * @param [in] endTimeItr: The iterator to the end of the time stamp trajectory.
   * @param [in] dtInitial: Initial time step.
   * @param [in] AbsTol: The absolute tolerance error for ode solver.
   * @param [in] RelTol: The relative tolerance error for ode solver.
   */
  template <typename S = Stepper>
  typename std::enable_if<std::is_same<S, runge_kutta_dopri5_t>::value, void>::type integrateTimesSpecialized(
      system_func_t system, observer_func_t observer, vector_t& initialState, typename scalar_array_t::const_iterator beginTimeItr,
      typename scalar_array_t::const_iterator endTimeItr, scalar_t dtInitial, scalar_t AbsTol, scalar_t RelTol);

  /**
   * Integrate times specialized function
   * @tparam S: stepper type.
   * @param [in] system: System function
   * @param [in] observer: Observer callback
   * @param [in] initialState: Initial state.
   * @param [in] beginTimeItr: The iterator to the beginning of the time stamp trajectory.
   * @param [in] endTimeItr: The iterator to the end of the time stamp trajectory.
   * @param [in] dtInitial: Initial time step.
   * @param [in] AbsTol: The absolute tolerance error for ode solver.
   * @param [in] RelTol: The relative tolerance error for ode solver.
   */
  template <typename S = Stepper>
  typename std::enable_if<!std::is_same<S, runge_kutta_dopri5_t>::value, void>::type integrateTimesSpecialized(
      system_func_t system, observer_func_t observer, vector_t& initialState, typename scalar_array_t::const_iterator beginTimeItr,
      typename scalar_array_t::const_iterator endTimeItr, scalar_t dtInitial, scalar_t AbsTol, scalar_t RelTol);

  /**
   * Functionality to reset stepper. If we integrate with ODE45, we don't need to reset the stepper, hence specialize empty function
   * @tparam S: stepper type.
   * @param [in] initialState: Initial state.
   * @param [in] t: Time.
   * @param [in] dt: Time step.
   */
  template <typename S = Stepper>
  typename std::enable_if<std::is_same<S, runge_kutta_dopri5_t>::value, void>::type initializeStepper(vector_t& initialState, scalar_t t,
                                                                                                      scalar_t dt);

  /**
   * Functionality to reset stepper. If we integrate with some other method, e.g.
   * adams_bashforth, we need to reset the stepper, hence specialize with initialize call
   *
   * @tparam S
   * @param [in] initialState
   * @param [in] t: Time.
   * @param [in] dt: Time step.
   */
  template <typename S = Stepper>
  typename std::enable_if<!(std::is_same<S, runge_kutta_dopri5_t>::value), void>::type initializeStepper(vector_t& initialState, scalar_t t,
                                                                                                         scalar_t dt);

  /*
   * Variables
   */
  Stepper stepper_;
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Stepper>
inline void Integrator<Stepper>::runIntegrateConst(system_func_t system, observer_func_t observer, const vector_t& initialState,
                                                   scalar_t startTime, scalar_t finalTime, scalar_t dt) {
  // TODO(mspieler): initializeStepper not used, why?
  // /*
  //  * use a temporary state for initialization, the state returned by initialize is different
  //  * from the real init state (already forward integrated)
  //  */
  // vector_t initialStateInternal_init_temp = initialState;
  // initializeStepper(initialStateInternal_init_temp, startTime, dt);

  vector_t initialStateInternal = initialState;
  // Ensure that finalTime is included by adding a fraction of dt such that: N * dt <= finalTime < (N + 1) * dt.
  finalTime += 0.1 * dt;
  boost::numeric::odeint::integrate_const(stepper_, system, initialStateInternal, startTime, finalTime, dt, observer);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Stepper>
inline void Integrator<Stepper>::runIntegrateAdaptive(system_func_t system, observer_func_t observer, const vector_t& initialState,
                                                      scalar_t startTime, scalar_t finalTime, scalar_t dtInitial, scalar_t AbsTol,
                                                      scalar_t RelTol) {
  vector_t internalStartState = initialState;
  integrateAdaptiveSpecialized<Stepper>(system, observer, internalStartState, startTime, finalTime, dtInitial, AbsTol, RelTol);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Stepper>
inline void Integrator<Stepper>::runIntegrateTimes(system_func_t system, observer_func_t observer, const vector_t& initialState,
                                                   typename scalar_array_t::const_iterator beginTimeItr,
                                                   typename scalar_array_t::const_iterator endTimeItr, scalar_t dtInitial, scalar_t AbsTol,
                                                   scalar_t RelTol) {
  vector_t internalStartState = initialState;
  integrateTimesSpecialized<Stepper>(system, observer, internalStartState, beginTimeItr, endTimeItr, dtInitial, AbsTol, RelTol);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Stepper>
template <typename S>
inline typename std::enable_if<std::is_same<S, runge_kutta_dopri5_t>::value, void>::type Integrator<Stepper>::integrateAdaptiveSpecialized(
    system_func_t system, observer_func_t observer, vector_t& initialState, scalar_t startTime, scalar_t finalTime, scalar_t dtInitial,
    scalar_t AbsTol, scalar_t RelTol) {
  boost::numeric::odeint::integrate_adaptive(boost::numeric::odeint::make_controlled<S>(AbsTol, RelTol), system, initialState, startTime,
                                             finalTime, dtInitial, observer);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Stepper>
template <typename S>
inline typename std::enable_if<!std::is_same<S, runge_kutta_dopri5_t>::value, void>::type Integrator<Stepper>::integrateAdaptiveSpecialized(
    system_func_t system, observer_func_t observer, vector_t& initialState, scalar_t startTime, scalar_t finalTime, scalar_t dtInitial,
    scalar_t AbsTol, scalar_t RelTol) {
  boost::numeric::odeint::integrate_adaptive(stepper_, system, initialState, startTime, finalTime, dtInitial, observer);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Stepper>
template <typename S>
inline typename std::enable_if<std::is_same<S, runge_kutta_dopri5_t>::value, void>::type Integrator<Stepper>::integrateTimesSpecialized(
    system_func_t system, observer_func_t observer, vector_t& initialState, typename scalar_array_t::const_iterator beginTimeItr,
    typename scalar_array_t::const_iterator endTimeItr, scalar_t dtInitial, scalar_t AbsTol, scalar_t RelTol) {
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
template <class Stepper>
template <typename S>
inline typename std::enable_if<!std::is_same<S, runge_kutta_dopri5_t>::value, void>::type Integrator<Stepper>::integrateTimesSpecialized(
    system_func_t system, observer_func_t observer, vector_t& initialState, typename scalar_array_t::const_iterator beginTimeItr,
    typename scalar_array_t::const_iterator endTimeItr, scalar_t dtInitial, scalar_t AbsTol, scalar_t RelTol) {
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
template <class Stepper>
template <typename S>
inline typename std::enable_if<std::is_same<S, runge_kutta_dopri5_t>::value, void>::type Integrator<Stepper>::initializeStepper(
    vector_t& initialState, scalar_t t, scalar_t dt) {
  /**do nothing, runge_kutta_5_t does not have a init method */
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Stepper>
template <typename S>
typename std::enable_if<!(std::is_same<S, runge_kutta_dopri5_t>::value), void>::type Integrator<Stepper>::initializeStepper(
    vector_t& initialState, scalar_t t, scalar_t dt) {
  stepper_.initialize(runge_kutta_dopri5_t(), system, initialState, t, dt);
}

/**
 * Euler integrator.
 */
using IntegratorEuler = Integrator<euler_t>;

/**
 * Modified midpoint integrator.
 */
using IntegratorModifiedMidpoint = Integrator<modified_midpoint_t>;

/**
 * RK4 integrator.
 */
using IntegratorRK4 = Integrator<runge_kutta_4_t>;

/**
 * RK5 variable integrator.
 */
using IntegratorRK5Variable = Integrator<dense_runge_kutta5_t>;

/**
 * ode45 integrator.
 */
using ODE45 = Integrator<runge_kutta_dopri5_t>;

/**
 * Adams-Bashforth integrator.
 */
template <size_t STEPS>
using IntegratorAdamsBashforth = Integrator<adams_bashforth_uncontrolled_t<STEPS>>;

/**
 * Bulirsch-Stoer integrator.
 */
using IntegratorBulirschStoer = Integrator<bulirsch_stoer_t>;

/**
 * Adams-Bashforth-Moulton integrator (works only after boost 1.56)
 */
#if (BOOST_VERSION / 100000 == 1 && BOOST_VERSION / 100 % 1000 > 55)
template <size_t STEPS>
using IntegratorAdamsBashforthMoulton = Integrator<adams_bashforth_moulton_uncontrolled_t<STEPS>>;
#endif

}  // namespace ocs2
