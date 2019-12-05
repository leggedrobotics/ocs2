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

#ifndef OCS2_INTEGRATORBASE_H_
#define OCS2_INTEGRATORBASE_H_

#include <limits>

#include "ocs2_core/integration/Observer.h"
#include "ocs2_core/integration/OdeBase.h"
#include "ocs2_core/integration/SystemEventHandler.h"

namespace ocs2 {

/**
 * The interface class for integration of autonomous systems.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 */
template <int STATE_DIM>
class IntegratorBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using scalar_t = double;
  using scalar_array_t = std::vector<scalar_t>;
  using state_vector_t = Eigen::Matrix<scalar_t, STATE_DIM, 1>;
  using state_vector_array_t = std::vector<state_vector_t, Eigen::aligned_allocator<state_vector_t>>;

  using observer_t = Observer<STATE_DIM>;
  using observer_func_t = std::function<void(const state_vector_t& x, const scalar_t& t)>;

  /**
   * Constructor
   * @param [in] system: The system dynamics.
   * @param [in] eventHandler: The integration event function.
   */
  explicit IntegratorBase(const std::shared_ptr<OdeBase<STATE_DIM>>& systemPtr,
                          const std::shared_ptr<SystemEventHandler<STATE_DIM>>& eventHandlerPtr = nullptr)

      : systemPtr_(systemPtr), eventHandlerPtr_(eventHandlerPtr) {
    if (eventHandlerPtr_) {
      eventHandlerPtr_->setSystem(systemPtr_);
    }

    // setup system function
    systemFunction_ = [this](const state_vector_t& x, state_vector_t& dxdt, const scalar_t& t) { systemPtr_->computeFlowMap(t, x, dxdt); };
  }

  /**
   * Default destructor
   */
  virtual ~IntegratorBase() = default;

  /**
   * Gets the system dynamics.
   *
   * @return A reference to the system dynamics.
   */
  OdeBase<STATE_DIM>& getSystem() { return *systemPtr_; }

  /**
   * Gets the system dynamics.
   *
   * @return A constant reference to the system dynamics.
   */
  const OdeBase<STATE_DIM>& getSystem() const { return *systemPtr_; }

  /**
   * Gets the event handler.
   *
   * @return A reference to the event handler.
   */
  SystemEventHandler<STATE_DIM>& getEventHandler() { return *eventHandlerPtr_; }

  /**
   * Gets the event handler.
   *
   * @return A constant reference to the event handler.
   */
  const SystemEventHandler<STATE_DIM>& getEventHandler() const { return *eventHandlerPtr_; }

  /**
   * Equidistant integration based on initial and final time as well as step length.
   *
   * @param [in] initialState: Initial state.
   * @param [in] startTime: Initial time.
   * @param [in] finalTime: Final time.
   * @param [in] dt: Time step.
   * @param [in] observer: Observer.
   * @param [in] maxNumSteps: The maximum number of integration points per a second for ode solver.
   */
  inline void integrate_const(const state_vector_t& initialState, scalar_t startTime, scalar_t finalTime, scalar_t dt, observer_t& observer,
                              int maxNumSteps = std::numeric_limits<int>::max()) {
    // setup observer function
    auto observerFunc = [&, this](const state_vector_t& x, const scalar_t& t) { observer.observe(systemPtr_, eventHandlerPtr_, x, t); };

    if (this->eventHandlerPtr_ && maxNumSteps < std::numeric_limits<int>::max()) {
      this->eventHandlerPtr_->setMaxNumSteps(maxNumSteps);
    }
    run_integrate_const(initialState, startTime, finalTime, dt, observerFunc);
  }

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
   * @param [in] observer: Observer.
   * @param [in] dtInitial: Initial time step.
   * @param [in] AbsTol: The absolute tolerance error for ode solver.
   * @param [in] RelTol: The relative tolerance error for ode solver.
   * @param [in] maxNumSteps: The maximum number of integration points per a second for ode solver.
   */
  inline void integrate_adaptive(const state_vector_t& initialState, scalar_t startTime, scalar_t finalTime, observer_t& observer,
                                 scalar_t dtInitial = 0.01, scalar_t AbsTol = 1e-6, scalar_t RelTol = 1e-3,
                                 int maxNumSteps = std::numeric_limits<int>::max()) {
    // setup observer function
    auto observerFunc = [&, this](const state_vector_t& x, const scalar_t& t) { observer.observe(systemPtr_, eventHandlerPtr_, x, t); };

    if (this->eventHandlerPtr_ && maxNumSteps < std::numeric_limits<int>::max()) {
      this->eventHandlerPtr_->setMaxNumSteps(maxNumSteps);
    }
    run_integrate_adaptive(initialState, startTime, finalTime, dtInitial, AbsTol, RelTol, observerFunc);
  }

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
   * @param [in] observer: Observer.
   * @param [in] dtInitial: Initial time step.
   * @param [in] AbsTol: The absolute tolerance error for ode solver.
   * @param [in] RelTol: The relative tolerance error for ode solver.
   * @param [in] maxNumSteps: The maximum number of integration points per a second for ode solver.
   */
  inline void integrate_times(const state_vector_t& initialState, typename scalar_array_t::const_iterator beginTimeItr,
                              typename scalar_array_t::const_iterator endTimeItr, observer_t& observer, scalar_t dtInitial = 0.01,
                              scalar_t AbsTol = 1e-9, scalar_t RelTol = 1e-6, int maxNumSteps = std::numeric_limits<int>::max()) {
    // setup observer function
    auto observerFunc = [&, this](const state_vector_t& x, const scalar_t& t) { observer.observe(systemPtr_, eventHandlerPtr_, x, t); };

    if (this->eventHandlerPtr_ && maxNumSteps < std::numeric_limits<int>::max()) {
      this->eventHandlerPtr_->setMaxNumSteps(maxNumSteps);
    }
    run_integrate_times(initialState, beginTimeItr, endTimeItr, dtInitial, AbsTol, RelTol, observerFunc);
  }

 protected:
  /**
   * Equidistant integration based on initial and final time as well as step length.
   *
   * @param [in] initialState: Initial state.
   * @param [in] startTime: Initial time.
   * @param [in] finalTime: Final time.
   * @param [in] dt: Time step.
   * @param [in] observerFunc: Observer callback
   */
  virtual void run_integrate_const(const state_vector_t& initialState, scalar_t startTime, scalar_t finalTime, scalar_t dt,
                                   observer_func_t observerFunc) = 0;

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
  virtual void run_integrate_adaptive(const state_vector_t& initialState, scalar_t startTime, scalar_t finalTime, scalar_t dtInitial,
                                      scalar_t AbsTol, scalar_t RelTol, observer_func_t observerFunc) = 0;

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
  virtual void run_integrate_times(const state_vector_t& initialState, typename scalar_array_t::const_iterator beginTimeItr,
                                   typename scalar_array_t::const_iterator endTimeItr, scalar_t dtInitial, scalar_t AbsTol, scalar_t RelTol,
                                   observer_func_t observerFunc) = 0;

  /**
   * Variables
   */
  std::shared_ptr<OdeBase<STATE_DIM>> systemPtr_;                   // System dynamics used by integrator.
  std::shared_ptr<SystemEventHandler<STATE_DIM>> eventHandlerPtr_;  // Event handler used by integrator.
  std::function<void(const state_vector_t&, state_vector_t&, const scalar_t&)> systemFunction_;
};

}  // namespace ocs2

#endif /* OCS2INTEGRATORBASE_H_ */
