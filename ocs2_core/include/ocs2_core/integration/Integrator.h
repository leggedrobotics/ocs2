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

#ifndef INTEGRATOR_OCS2_H_
#define INTEGRATOR_OCS2_H_

#include <cmath>
#include <functional>
#include <stdexcept>
#include <string>
#include <type_traits>

#include <boost/numeric/odeint.hpp>

#include "ocs2_core/OCS2NumericTraits.h"
#include "ocs2_core/integration/IntegratorBase.h"
#include "ocs2_core/integration/eigenIntegration.h"
#include "ocs2_core/integration/steppers.h"

namespace ocs2 {

/**
 * @brief The IntegratorType enum
 * Enum used in selecting a specific integrator.
 */
enum class IntegratorType { EULER, ODE45, ADAMS_BASHFORTH, BULIRSCH_STOER, MODIFIED_MIDPOINT, RK4, RK5_VARIABLE, ADAMS_BASHFORTH_MOULTON };

static std::string toString(IntegratorType integratorType) {
  switch (integratorType) {
    case IntegratorType::EULER:
      return std::string("EULER");
    case IntegratorType::ODE45:
      return std::string("ODE45");
    case IntegratorType::ADAMS_BASHFORTH:
      return std::string("ADAMS_BASHFORTH");
    case IntegratorType::BULIRSCH_STOER:
      return std::string("BULIRSCH_STOER");
    case IntegratorType::MODIFIED_MIDPOINT:
      return std::string("MODIFIED_MIDPOINT");
    case IntegratorType::RK4:
      return std::string("RK4");
    case IntegratorType::RK5_VARIABLE:
      return std::string("RK5_VARIABLE");
    case IntegratorType::ADAMS_BASHFORTH_MOULTON:
      return std::string("ADAMS_BASHFORTH_MOULTON");
    default:
      return std::string("UNKNOWN ") + std::to_string(static_cast<int>(integratorType));
  }
}

static IntegratorType fromString(const std::string& name) {
  if (name == "EULER") {
    return IntegratorType::EULER;
  } else if (name == "ODE45") {
    return IntegratorType::ODE45;
  } else if (name == "ADAMS_BASHFORTH") {
    return IntegratorType::ADAMS_BASHFORTH;
  } else if (name == "BULIRSCH_STOER") {
    return IntegratorType::BULIRSCH_STOER;
  } else if (name == "MODIFIED_MIDPOINT") {
    return IntegratorType::MODIFIED_MIDPOINT;
  } else if (name == "RK4") {
    return IntegratorType::RK4;
  } else if (name == "RK5_VARIABLE") {
    return IntegratorType::RK5_VARIABLE;
  } else if (name == "ADAMS_BASHFORTH_MOULTON") {
    return IntegratorType::ADAMS_BASHFORTH_MOULTON;
  } else {
    int val;
    try {
      val = std::stoi(name);
    } catch (std::invalid_argument const& e) {
      throw std::runtime_error("Unknown IntegratorType: " + name);
    }
    return static_cast<IntegratorType>(val);  // TODO(mspieler): workaround to allow integer values
  }
}

/**
 * Create Integrator of given type.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 *
 * @param [in] integratorType: The integrator type.
 * @param [in] system: The system dynamics.
 * @param [in] eventHandler: The integration event function.
 */
template <int STATE_DIM>
std::unique_ptr<IntegratorBase<STATE_DIM>> newIntegrator(IntegratorType integratorType,
                                                         const std::shared_ptr<OdeBase<STATE_DIM>>& systemPtr,
                                                         const std::shared_ptr<SystemEventHandler<STATE_DIM>>& eventHandlerPtr = nullptr);

/**
 * Integrator class for autonomous systems.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam Stepper: Stepper class type to be used.
 */
template <int STATE_DIM, class Stepper>
class Integrator : public IntegratorBase<STATE_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using BASE = IntegratorBase<STATE_DIM>;
  using scalar_t = typename BASE::scalar_t;
  using scalar_array_t = typename BASE::scalar_array_t;
  using state_vector_t = typename BASE::state_vector_t;
  using state_vector_array_t = typename BASE::state_vector_array_t;
  using observer_func_t = typename BASE::observer_func_t;

  /**
   * Constructor
   *
   * @param [in] system: The system dynamics.
   * @param [in] eventHandler: The integration event function.
   */
  explicit Integrator(const std::shared_ptr<OdeBase<STATE_DIM>>& systemPtr,
                      const std::shared_ptr<SystemEventHandler<STATE_DIM>>& eventHandlerPtr = nullptr);

  /**
   * Destructor
   */
  ~Integrator() = default;

 private:
  /**
   * Equidistant integration based on initial and final time as well as step length.
   *
   * @param [in] initialState: Initial state.
   * @param [in] startTime: Initial time.
   * @param [in] finalTime: Final time.
   * @param [in] dt: Time step.
   * @param [in] observerFunc: Observer callback
   */
  void run_integrate_const(const state_vector_t& initialState, const scalar_t& startTime, const scalar_t& finalTime, scalar_t dt,
                           observer_func_t observerFunc) final;

  /**
   * Adaptive time integration based on start time and final time. This method can
   * solve ODEs with time-dependent events, if eventsTime is not empty. In this case
   * the output time-trajectory contains two identical values at the moments
   * of event triggers. This method uses OdeBase::computeJumpMap() method for
   * state transition at events.
   *
   * @param [in] initialState: Initial state.
   * @param [in] startTime: Initial time.
   * @param [in] finalTime: Final time.
   * @param [in] dtInitial: Initial time step.
   * @param [in] AbsTol: The absolute tolerance error for ode solver.
   * @param [in] RelTol: The relative tolerance error for ode solver.
   * @param [in] observerFunc: Observer callback
   */
  void run_integrate_adaptive(const state_vector_t& initialState, const scalar_t& startTime, const scalar_t& finalTime, scalar_t dtInitial,
                              scalar_t AbsTol, scalar_t RelTol, observer_func_t observerFunc) final;

  /**
   * Output integration based on a given time trajectory. This method can solve ODEs
   * with time-dependent events. In this case, user should pass past-the-end indices
   * of events on the input time trajectory. Moreover, this method assumes that there
   * are two identical time values in the input time-trajectory at the moments of event
   * triggers. This method uses OdeBase::computeJumpMap() method for state
   * transition at events.
   *
   * @param [in] initialState: Initial state.
   * @param [in] beginTimeItr: The iterator to the beginning of the time stamp trajectory.
   * @param [in] endTimeItr: The iterator to the end of the time stamp trajectory.
   * @param [in] dtInitial: Initial time step.
   * @param [in] AbsTol: The absolute tolerance error for ode solver.
   * @param [in] RelTol: The relative tolerance error for ode solver.
   * @param [in] observerFunc: Observer callback
   */
  void run_integrate_times(const state_vector_t& initialState, typename scalar_array_t::const_iterator beginTimeItr,
                           typename scalar_array_t::const_iterator endTimeItr, scalar_t dtInitial, scalar_t AbsTol, scalar_t RelTol,
                           observer_func_t observerFunc) final;

  /**
   * Integrate adaptive specialized.
   *
   * @tparam S: stepper type.
   * @param [in] initialState: Initial state.
   * @param [in] startTime: Initial time.
   * @param [in] finalTime: Final time.
   * @param [in] dtInitial: Initial time step.
   * @param [in] AbsTol: The absolute tolerance error for ode solver.
   * @param [in] RelTol: The relative tolerance error for ode solver.
   * @param [in] observerFunc: Observer callback
   */
  template <typename S>
  typename std::enable_if<std::is_same<S, runge_kutta_dopri5_t<STATE_DIM>>::value, void>::type integrate_adaptive_specialized(
      state_vector_t& initialState, const scalar_t& startTime, const scalar_t& finalTime, scalar_t dtInitial, scalar_t AbsTol,
      scalar_t RelTol, observer_func_t observerFunc);

  /**
   * Integrate adaptive specialized,
   *
   * @tparam S: stepper type.
   * @param [in] initialState: Initial state.
   * @param [in] startTime: Initial time.
   * @param [in] finalTime: Final time.
   * @param [in] dtInitial: Initial time step.
   * @param [in] AbsTol: The absolute tolerance error for ode solver.
   * @param [in] RelTol: The relative tolerance error for ode solver.
   * @param [in] observerFunc: Observer callback
   */
  template <typename S>
  typename std::enable_if<!std::is_same<S, runge_kutta_dopri5_t<STATE_DIM>>::value, void>::type integrate_adaptive_specialized(
      state_vector_t& initialState, const scalar_t& startTime, const scalar_t& finalTime, scalar_t dtInitial, scalar_t AbsTol,
      scalar_t RelTol, observer_func_t observerFunc);

  /**
   * Integrate times specialized function
   * @tparam S: stepper type.
   * @param [in] initialState: Initial state.
   * @param [in] beginTimeItr: The iterator to the beginning of the time stamp trajectory.
   * @param [in] endTimeItr: The iterator to the end of the time stamp trajectory.
   * @param [in] dtInitial: Initial time step.
   * @param [in] AbsTol: The absolute tolerance error for ode solver.
   * @param [in] RelTol: The relative tolerance error for ode solver.
   * @param [in] observerFunc: Observer callback
   */
  template <typename S = Stepper>
  typename std::enable_if<std::is_same<S, runge_kutta_dopri5_t<STATE_DIM>>::value, void>::type integrate_times_specialized(
      state_vector_t& initialState, typename scalar_array_t::const_iterator beginTimeItr,
      typename scalar_array_t::const_iterator endTimeItr, scalar_t dtInitial, scalar_t AbsTol, scalar_t RelTol,
      observer_func_t observerFunc);

  /**
   * Integrate times specialized function
   * @tparam S: stepper type.
   * @param [in] initialState: Initial state.
   * @param [in] beginTimeItr: The iterator to the beginning of the time stamp trajectory.
   * @param [in] endTimeItr: The iterator to the end of the time stamp trajectory.
   * @param [in] dtInitial: Initial time step.
   * @param [in] AbsTol: The absolute tolerance error for ode solver.
   * @param [in] RelTol: The relative tolerance error for ode solver.
   * @param [in] observerFunc: Observer callback
   */
  template <typename S = Stepper>
  typename std::enable_if<!std::is_same<S, runge_kutta_dopri5_t<STATE_DIM>>::value, void>::type integrate_times_specialized(
      state_vector_t& initialState, typename scalar_array_t::const_iterator beginTimeItr,
      typename scalar_array_t::const_iterator endTimeItr, scalar_t dtInitial, scalar_t AbsTol, scalar_t RelTol,
      observer_func_t observerFunc);

  /**
   * Functionality to reset stepper. If we integrate with ODE45, we don't need to reset the stepper, hence specialize empty function
   * @tparam S: stepper type.
   * @param [in] initialState: Initial state.
   * @param [in] t: Time.
   * @param [in] dt: Time step.
   */
  template <typename S = Stepper>
  typename std::enable_if<std::is_same<S, runge_kutta_dopri5_t<STATE_DIM>>::value, void>::type initializeStepper(
      state_vector_t& initialState, scalar_t& t, scalar_t dt);

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
  typename std::enable_if<!(std::is_same<S, runge_kutta_dopri5_t<STATE_DIM>>::value), void>::type initializeStepper(
      state_vector_t& initialState, scalar_t& t, scalar_t dt);

  /********
   * Variables
   ********/
  std::unique_ptr<Stepper> stepperPtr_;
};

/**
 * Euler integrator.
 */
template <int STATE_DIM>
using IntegratorEuler = Integrator<STATE_DIM, euler_t<STATE_DIM>>;

/**
 * Modified midpoint integrator.
 */
template <int STATE_DIM>
using IntegratorModifiedMidpoint = Integrator<STATE_DIM, modified_midpoint_t<STATE_DIM>>;

/**
 * RK4 integrator.
 */
template <int STATE_DIM>
using IntegratorRK4 = Integrator<STATE_DIM, runge_kutta_4_t<STATE_DIM>>;

/**
 * RK5 variable integrator.
 */
template <int STATE_DIM>
using IntegratorRK5Variable = Integrator<STATE_DIM, dense_runge_kutta5_t<STATE_DIM>>;

/**
 * ode45 integrator.
 */
template <int STATE_DIM>
using ODE45 = Integrator<STATE_DIM, runge_kutta_dopri5_t<STATE_DIM>>;

/**
 * Adams-Bashforth integrator.
 */
template <int STATE_DIM, size_t STEPS>
using IntegratorAdamsBashforth = Integrator<STATE_DIM, adams_bashforth_uncontrolled_t<STATE_DIM, STEPS>>;

/**
 * Bulirsch-Stoer integrator.
 */
template <int STATE_DIM>
using IntegratorBulirschStoer = Integrator<STATE_DIM, bulirsch_stoer_t<STATE_DIM>>;

/**
 * Adams-Bashforth-Moulton integrator (works only after boost 1.56)
 */
#if (BOOST_VERSION / 100000 == 1 && BOOST_VERSION / 100 % 1000 > 55)
template <int STATE_DIM, size_t STEPS>
using IntegratorAdamsBashforthMoulton = Integrator<STATE_DIM, adams_bashforth_moulton_uncontrolled_t<STATE_DIM, STEPS>>;
#endif

}  // namespace ocs2

#include "implementation/Integrator.h"

#endif /* INTEGRATOR_OCS2_H_ */
