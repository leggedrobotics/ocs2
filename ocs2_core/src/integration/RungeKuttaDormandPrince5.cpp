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

#include <algorithm>
#include <limits>

#include <ocs2_core/integration/RungeKuttaDormandPrince5.h>

namespace ocs2 {

namespace {

/** Helper less comparison for both positive and negative dt case. */
bool lessWithSign(scalar_t t1, scalar_t t2, scalar_t dt) {
  if (dt > 0) {
    return t2 - t1 > std::numeric_limits<scalar_t>::epsilon();
  } else {
    return t1 - t2 > std::numeric_limits<scalar_t>::epsilon();
  }
}

/** Helper to get the min absolute value, t1 and t2 have same sign. */
scalar_t minAbs(scalar_t t1, scalar_t t2) {
  if (t1 > 0) {
    return std::min(t1, t2);
  } else {
    return std::max(t1, t2);
  }
}

/** Helper to get max absolute value, t1 and t2 have same sign. */
scalar_t maxAbs(scalar_t t1, scalar_t t2) {
  if (t1 > 0) {
    return std::max(t1, t2);
  } else {
    return std::min(t1, t2);
  }
}

/** Runge Kutta Dormand-Prince stepper */
class Stepper {
 public:
  using system_func_t = IntegratorBase::system_func_t;

  /**
   * Try to perform one step. If the step is accepted, then state (x), derivative (dxdt), time (t) and step size (dt) are updated.
   * Otherwise only the step size (dt) is updated and false is returned.
   *
   * @param [in] system: System function.
   * @param [in,out] x: current state, updated if step is taken.
   * @param [in,out] dxdt: current derivative wrt. time, updated if step is taken.
   * @param [in,out] t: current time, updated if step is taken.
   * @param [in,out] dt: step size, updated if step is taken.
   * @param [in] absTol: The absolute tolerance error for ode solver.
   * @param [in] relTol: The relative tolerance error for ode solver.
   * @return true if the step is taken, false otherwise..
   */
  bool tryStep(system_func_t& system, vector_t& x, vector_t& dxdt, scalar_t& t, scalar_t& dt, scalar_t absTol, scalar_t relTol) {
    constexpr scalar_t c1 = 35.0 / 384;
    // c2 = 0
    constexpr scalar_t c3 = 500.0 / 1113;
    constexpr scalar_t c4 = 125.0 / 192;
    constexpr scalar_t c5 = -2187.0 / 6784;
    constexpr scalar_t c6 = 11.0 / 84;

    constexpr scalar_t dc1 = c1 - 5179.0 / 57600;
    constexpr scalar_t dc3 = c3 - 7571.0 / 16695;
    constexpr scalar_t dc4 = c4 - 393.0 / 640;
    constexpr scalar_t dc5 = c5 - -92097.0 / 339200;
    constexpr scalar_t dc6 = c6 - 187.0 / 2100;
    constexpr scalar_t dc7 = -1.0 / 40;

    vector_t x_out, dxdt_out;
    doStep(system, x, dxdt, t, dt, x_out, dxdt_out);

    // error estimate
    const vector_t x_err = dt * (dc1 * k1_ + dc3 * k3_ + dc4 * k4_ + dc5 * k5_ + dc6 * k6_ + dc7 * dxdt_out);

    const scalar_t error = maxError(x, dxdt, x_err, dt, absTol, relTol);
    if (error > 1.0) {
      dt = decreaseStep(dt, error);
      return false;
    } else {
      // accept the step
      t += dt;
      x = x_out;
      dxdt = dxdt_out;
      dt = increaseStep(dt, error);
      return true;
    }
  }

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
              vector_t& dxdt_out) {
    /* Runge Kutta Dormand-Prince Butcher tableau constants.
     * https://en.wikipedia.org/wiki/Dormand%E2%80%93Prince_method */
    constexpr scalar_t a2 = 1.0 / 5;
    constexpr scalar_t a3 = 3.0 / 10;
    constexpr scalar_t a4 = 4.0 / 5;
    constexpr scalar_t a5 = 8.0 / 9;

    constexpr scalar_t b21 = 1.0 / 5;

    constexpr scalar_t b31 = 3.0 / 40;
    constexpr scalar_t b32 = 9.0 / 40;

    constexpr scalar_t b41 = 44.0 / 45;
    constexpr scalar_t b42 = -56.0 / 15;
    constexpr scalar_t b43 = 32.0 / 9;

    constexpr scalar_t b51 = 19372.0 / 6561;
    constexpr scalar_t b52 = -25360.0 / 2187;
    constexpr scalar_t b53 = 64448.0 / 6561;
    constexpr scalar_t b54 = -212.0 / 729;

    constexpr scalar_t b61 = 9017.0 / 3168;
    constexpr scalar_t b62 = -355.0 / 33;
    constexpr scalar_t b63 = 46732.0 / 5247;
    constexpr scalar_t b64 = 49.0 / 176;
    constexpr scalar_t b65 = -5103.0 / 18656;

    constexpr scalar_t c1 = 35.0 / 384;
    // c2 = 0
    constexpr scalar_t c3 = 500.0 / 1113;
    constexpr scalar_t c4 = 125.0 / 192;
    constexpr scalar_t c5 = -2187.0 / 6784;
    constexpr scalar_t c6 = 11.0 / 84;

    k1_ = dxdt;  // k1 = system(x, t) from previous iteration
    vector_t x = x0 + dt * b21 * k1_;
    system(x, k2_, t + dt * a2);
    x.noalias() = x0 + dt * b31 * k1_ + dt * b32 * k2_;
    system(x, k3_, t + dt * a3);
    x.noalias() = x0 + dt * (b41 * k1_ + b42 * k2_ + b43 * k3_);
    system(x, k4_, t + dt * a4);
    x.noalias() = x0 + dt * (b51 * k1_ + b52 * k2_ + b53 * k3_ + b54 * k4_);
    system(x, k5_, t + dt * a5);
    x.noalias() = x0 + dt * (b61 * k1_ + b62 * k2_ + b63 * k3_ + b64 * k4_ + b65 * k5_);
    system(x, k6_, t + dt);
    // update x_out and dxdt_out (x_out can be x0 and dxdt_out can be dxdt)
    x_out = x0 + dt * (c1 * k1_ + c3 * k3_ + c4 * k4_ + c5 * k5_ + c6 * k6_);
    system(x_out, dxdt_out, t + dt);
  }

 private:
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
  static scalar_t maxError(const vector_t& x_old, const vector_t& dxdt_old, const vector_t& x_err, scalar_t dt, scalar_t absTol,
                           scalar_t relTol) {
    const vector_t err = x_err.array() / (absTol + relTol * (x_old.array().abs() + std::abs(dt) * dxdt_old.array().abs()));
    return err.lpNorm<Eigen::Infinity>();
  }

  /**
   * Decrease the step size
   *
   * @param [in] dt: step size.
   * @param [in] error: maximal error.
   * @return new step size dt.
   */
  static scalar_t decreaseStep(scalar_t dt, scalar_t error) {
    constexpr int ERROR_ORDER = 4;
    dt *= std::max(0.9 * std::pow(error, -1.0 / (ERROR_ORDER - 1)), 0.2);
    return dt;
  }

  /**
   * Increase the step size
   *
   * @param [in] dt: step size.
   * @param [in] error: maximal error.
   * @return new step size dt.
   */
  static scalar_t increaseStep(scalar_t dt, scalar_t error) {
    constexpr int STEPPER_ORDER = 5;
    if (error < 0.5) {
      error = std::max(std::pow(scalar_t(5.0), -STEPPER_ORDER), error);
      dt *= 0.9 * std::pow(error, -1.0 / STEPPER_ORDER);
    }
    return dt;
  }

  /** intermediate derivatives during Runge-Kutta step. */
  vector_t k1_, k2_, k3_, k4_, k5_, k6_;
};

}  // namespace

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void RungeKuttaDormandPrince5::runIntegrateConst(system_func_t system, observer_func_t observer, const vector_t& initialState,
                                                 scalar_t startTime, scalar_t finalTime, scalar_t dt) {
  // TODO(mspieler): This does one redundant system() evaluation at the end.

  // Ensure that finalTime is included by adding a fraction of dt such that: N * dt <= finalTime < (N + 1) * dt.
  finalTime += 0.1 * dt;

  Stepper stepper;
  scalar_t t = startTime;
  vector_t x = initialState;
  vector_t dxdt;
  system(x, dxdt, t);
  size_t step = 0;
  while (lessWithSign(t + dt, finalTime, dt)) {
    observer(x, t);
    stepper.doStep(system, x, dxdt, t, dt, x, dxdt);
    step++;
    t = startTime + step * dt;
  }
  observer(x, t);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void RungeKuttaDormandPrince5::runIntegrateAdaptive(system_func_t system, observer_func_t observer, const vector_t& initialState,
                                                    scalar_t startTime, scalar_t finalTime, scalar_t dtInitial, scalar_t absTol,
                                                    scalar_t relTol) {
  Stepper stepper;
  scalar_t t = startTime;
  scalar_t dt = dtInitial;
  vector_t x = initialState;
  vector_t dxdt;
  system(x, dxdt, t);

  while (lessWithSign(t, finalTime, dt)) {
    observer(x, t);

    if (lessWithSign(finalTime, t + dt, dt)) {
      dt = finalTime - t;
    }

    size_t tries = 0;
    while (!stepper.tryStep(system, x, dxdt, t, dt, absTol, relTol)) {
      tries++;
      if (tries > maxNumStepsRetries_) {
        throw std::runtime_error("[RungeKuttaDormandPrince5] Max number of iterations exceeded");
      }
    }  // end of while loop
  }    // end of while loop
  observer(x, t);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void RungeKuttaDormandPrince5::runIntegrateTimes(system_func_t system, observer_func_t observer, const vector_t& initialState,
                                                 typename scalar_array_t::const_iterator beginTimeItr,
                                                 typename scalar_array_t::const_iterator endTimeItr, scalar_t dtInitial, scalar_t absTol,
                                                 scalar_t relTol) {
  Stepper stepper;
  scalar_t dt = dtInitial;
  vector_t x = initialState;
  vector_t dxdt;
  system(x, dxdt, *beginTimeItr);

  while (true) {
    scalar_t t = *beginTimeItr++;
    observer(x, t);

    if (beginTimeItr == endTimeItr) {
      break;
    }

    size_t tries = 0;
    while (lessWithSign(t, *beginTimeItr, dt)) {
      // adjust stepsize to end up exactly at the observation point
      scalar_t dtCurrent = minAbs(dt, *beginTimeItr - t);
      if (stepper.tryStep(system, x, dxdt, t, dtCurrent, absTol, relTol)) {
        tries = 0;
        // continue with the original step size if dt was reduced due to observation
        dt = maxAbs(dt, dtCurrent);
      } else {
        tries++;
        dt = dtCurrent;
        if (tries > maxNumStepsRetries_) {
          throw std::runtime_error("[RungeKuttaDormandPrince5] Max number of iterations exceeded");
        }
      }
    }  // end of while loop
  }    // end of while loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
constexpr size_t RungeKuttaDormandPrince5::maxNumStepsRetries_;

}  // namespace ocs2
