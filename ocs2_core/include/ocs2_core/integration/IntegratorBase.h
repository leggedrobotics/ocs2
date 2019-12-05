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
 * @tparam STATE_DIM: Dimension of the state space, can be Eigen::Dynamic.
 */
template <int STATE_DIM>
class IntegratorBase {
 public:
  using scalar_t = double;
  using scalar_array_t = std::vector<scalar_t>;
  using state_vector_t = Eigen::Matrix<scalar_t, STATE_DIM, 1>;

  using system_func_t = std::function<void(const state_vector_t& x, state_vector_t& dxdt, scalar_t t)>;
  using observer_func_t = std::function<void(const state_vector_t& x, scalar_t t)>;

  /**
   * Default constructor
   */
  IntegratorBase() = default;

  /**
   * Default destructor
   */
  virtual ~IntegratorBase() = default;

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
  inline void integrate_const(system_func_t system, observer_func_t observer, const state_vector_t& initialState, scalar_t startTime,
                              scalar_t finalTime, scalar_t dt) {
    run_integrate_const(system, observer, initialState, startTime, finalTime, dt);
  }

  /**
   * Adaptive time integration based on start time and final time. This method can
   * solve ODEs with time-dependent events, if eventsTime is not empty. In this case
   * the output time-trajectory contains two identical values at the moments
   * of event triggers. This method uses OdeBase::computeJumpMap() method for
   * state transition at events.
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
  inline void integrate_adaptive(system_func_t system, observer_func_t observer, const state_vector_t& initialState, scalar_t startTime,
                                 scalar_t finalTime, scalar_t dtInitial = 0.01, scalar_t AbsTol = 1e-6, scalar_t RelTol = 1e-3) {
    run_integrate_adaptive(system, observer, initialState, startTime, finalTime, dtInitial, AbsTol, RelTol);
  }

  /**
   * Output integration based on a given time trajectory. This method can solve ODEs
   * with time-dependent events. In this case, user should pass past-the-end indices
   * of events on the input time trajectory. Moreover, this method assumes that there
   * are two identical time values in the input time-trajectory at the moments of event
   * triggers. This method uses OdeBase::computeJumpMap() method for state
   * transition at events.
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
  inline void integrate_times(system_func_t system, observer_func_t observer, const state_vector_t& initialState,
                              typename scalar_array_t::const_iterator beginTimeItr, typename scalar_array_t::const_iterator endTimeItr,
                              scalar_t dtInitial = 0.01, scalar_t AbsTol = 1e-6, scalar_t RelTol = 1e-3) {
    run_integrate_times(system, observer, initialState, beginTimeItr, endTimeItr, dtInitial, AbsTol, RelTol);
  }

 protected:
  virtual void run_integrate_const(system_func_t system, observer_func_t observer, const state_vector_t& initialState, scalar_t startTime,
                                   scalar_t finalTime, scalar_t dt) = 0;

  virtual void run_integrate_adaptive(system_func_t system, observer_func_t observer, const state_vector_t& initialState,
                                      scalar_t startTime, scalar_t finalTime, scalar_t dtInitial, scalar_t AbsTol, scalar_t RelTol) = 0;

  virtual void run_integrate_times(system_func_t system, observer_func_t observer, const state_vector_t& initialState,
                                   typename scalar_array_t::const_iterator beginTimeItr, typename scalar_array_t::const_iterator endTimeItr,
                                   scalar_t dtInitial, scalar_t AbsTol, scalar_t RelTol) = 0;
};

}  // namespace ocs2

#endif /* OCS2INTEGRATORBASE_H_ */
