/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

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

#include <ocs2_core/integration/IntegratorBase.h>

namespace ocs2 {

/*
 * 5th order Runge Kutta Dormand-Prince (ode45) Integrator class
 *
 * The implementation is based on the boost odeint integrator with the controlled
 * boost::numeric::odeint::runge_kutta_dopri5 stepper.
 */
class RungeKuttaDormandPrince5 : public IntegratorBase {
 public:
  explicit RungeKuttaDormandPrince5(std::shared_ptr<SystemEventHandler> eventHandlerPtr = nullptr)
      : IntegratorBase(std::move(eventHandlerPtr)){};

  ~RungeKuttaDormandPrince5() override = default;

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
   * @param [in] absTol: The absolute tolerance error for ode solver.
   * @param [in] relTol: The relative tolerance error for ode solver.
   */
  void runIntegrateAdaptive(system_func_t system, observer_func_t observer, const vector_t& initialState, scalar_t startTime,
                            scalar_t finalTime, scalar_t dtInitial, scalar_t absTol, scalar_t relTol) override;
  /**
   * Output integration based on a given time trajectory.
   *
   * @param [in] system: System function
   * @param [in] observer: Observer callback
   * @param [in] initialState: Initial state.
   * @param [in] beginTimeItr: The iterator to the beginning of the time stamp trajectory.
   * @param [in] endTimeItr: The iterator to the end of the time stamp trajectory.
   * @param [in] dtInitial: Initial time step.
   * @param [in] absTol: The absolute tolerance error for ode solver.
   * @param [in] relTol: The relative tolerance error for ode solver.
   */
  void runIntegrateTimes(system_func_t system, observer_func_t observer, const vector_t& initialState,
                         typename scalar_array_t::const_iterator beginTimeItr, typename scalar_array_t::const_iterator endTimeItr,
                         scalar_t dtInitial, scalar_t absTol, scalar_t relTol) override;

 private:
  /**
   * Try to perform one step. If the step is accepted, then state (x), derivative (dxdt), time (t) and step size (dt) are updated.
   * Otherwise only the step size (dt) is updated and false is returned.
   *
   * @param [in] system: System function.
   * @param [in,out] x: current state.
   * @param [in,out] dxdt: current derivative wrt. time.
   * @param [in,out] t: current time.
   * @param [in,out] dt: step size.
   * @param [in] absTol: The absolute tolerance error for ode solver.
   * @param [in] relTol: The relative tolerance error for ode solver.
   * @return true if the step is taken, false otherwise..
   */
  bool tryStep(system_func_t& system, vector_t& x, vector_t& dxdt, scalar_t& t, scalar_t& dt, scalar_t absTol, scalar_t relTol);

  /**
   * Perform one Dormand-Prince step.
   *
   * @param [in] system: System function.
   * @param [in] x0: current state.
   * @param [in] dxdt: current derivative wrt. time.
   * @param [in] t: current time.
   * @param [in] dt: step size.
   * @param [out] x_out: next state (can be same reference as x0).
   * @param [out] dxdt_out: derivative at next state (can be same reference as dxdt).
   */
  void doStep(system_func_t& system, const vector_t& x0, const vector_t& dxdt, scalar_t t, scalar_t dt, vector_t& x_out,
              vector_t& dxdt_out);

  /**
   * Perform one Dormand-Prince step and the error estimate.
   *
   * @param [in] system: System function.
   * @param [in] x0: current state.
   * @param [in] dxdt: current derivative wrt. time.
   * @param [in] t: current time.
   * @param [in] dt: step size.
   * @param [out] x_out: next state (can be same reference as x0).
   * @param [out] dxdt_out: derivative at next state (can be same reference as dxdt).
   * @param [out] error: step error estimate.
   */
  void doStep(system_func_t& system, const vector_t& x0, const vector_t& dxdt, scalar_t t, scalar_t dt, vector_t& x_out, vector_t& dxdt_out,
              vector_t& error);

  /**
   * Estimate the maximal error value.
   *
   * @param [in] x_old: prevoius state.
   * @param [in] dxdt_old: prevoius derivative.
   * @param [in] error: step error estimate.
   * @param [in] dt: step size.
   * @param [in] absTol: The absolute error tolerance.
   * @param [in] relTol: The relative error tolerance.
   * @return maximal error value.
   */
  scalar_t error(const vector_t& x_old, const vector_t& dxdt_old, const vector_t& x_err, scalar_t dt, scalar_t eps_abs,
                 scalar_t eps_rel) const;

  /**
   * Decrease the step size
   *
   * @param [in] dt: step size.
   * @param [in] error: maximal error.
   * @return new step size dt.
   */
  scalar_t decreaseStep(scalar_t dt, scalar_t error) const;

  /**
   * Increase the step size
   *
   * @param [in] dt: step size.
   * @param [in] error: maximal error.
   * @return new step size dt.
   */
  scalar_t increaseStep(scalar_t dt, scalar_t error) const;

 private:
  const size_t MAX_STEP_RETRIES = 100;

  /** intermediate state during Runge-Kutta step. */
  vector_t x_;
  /** intermediate derivatives during Runge-Kutta step. */
  vector_t k1_, k2_, k3_, k4_, k5_, k6_;
};

}  // namespace ocs2
