/*
 * Author: Michael Spieler
 * Date:   2020-11-23
 */

#include <ocs2_core/integration/RungeKuttaDormandPrince5.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
static inline bool lessWithSign(scalar_t t1, scalar_t t2, scalar_t dt) {
  if (dt > 0) {
    return t2 - t1 > std::numeric_limits<scalar_t>::epsilon();
  } else {
    return t1 - t2 > std::numeric_limits<scalar_t>::epsilon();
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
static inline scalar_t minAbs(scalar_t t1, scalar_t t2) {
  if (t1 > 0) {
    return std::min(t1, t2);
  } else {
    return std::max(t1, t2);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
static inline scalar_t maxAbs(scalar_t t1, scalar_t t2) {
  if (t1 > 0) {
    return std::max(t1, t2);
  } else {
    return std::min(t1, t2);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void RungeKuttaDormandPrince5::runIntegrateConst(system_func_t system, observer_func_t observer, const vector_t& initialState,
                                                 scalar_t startTime, scalar_t finalTime, scalar_t dt) {
  // TODO(mspieler): This does one redundant system() evaluation at the end.

  // Ensure that finalTime is included by adding a fraction of dt such that: N * dt <= finalTime < (N + 1) * dt.
  finalTime += 0.1 * dt;

  scalar_t t = startTime;
  vector_t x = initialState;
  vector_t dxdt;
  system(x, dxdt, t);
  size_t step = 0;
  while (lessWithSign(t + dt, finalTime, dt)) {
    observer(x, t);
    doStep(system, x, dxdt, t, dt, x, dxdt);
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
    while (!tryStep(system, x, dxdt, t, dt, absTol, relTol)) {
      tries++;
      if (tries > MAX_STEP_RETRIES) {
        throw std::runtime_error("[RungeKuttaDormandPrince5] Max number of iterations exceeded");
      }
    }
  }
  observer(x, t);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void RungeKuttaDormandPrince5::runIntegrateTimes(system_func_t system, observer_func_t observer, const vector_t& initialState,
                                                 typename scalar_array_t::const_iterator beginTimeItr,
                                                 typename scalar_array_t::const_iterator endTimeItr, scalar_t dtInitial, scalar_t absTol,
                                                 scalar_t relTol) {
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
      if (tryStep(system, x, dxdt, t, dtCurrent, absTol, relTol)) {
        tries = 0;
        // continue with the original step size if dt was reduced due to observation
        dt = maxAbs(dt, dtCurrent);
      } else {
        tries++;
        dt = dtCurrent;
        if (tries > MAX_STEP_RETRIES) {
          throw std::runtime_error("[RungeKuttaDormandPrince5] Max number of iterations exceeded");
        }
      }
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool RungeKuttaDormandPrince5::tryStep(system_func_t& system, vector_t& x, vector_t& dxdt, scalar_t& t, scalar_t& dt, scalar_t absTol,
                                       scalar_t relTol) {
  vector_t x_out, dxdt_out, x_err;
  doStep(system, x, dxdt, t, dt, x_out, dxdt_out, x_err);

  const scalar_t maxRelErr = error(x, dxdt, x_err, dt, absTol, relTol);
  if (maxRelErr > 1.0) {
    dt = decreaseStep(dt, maxRelErr);
    return false;
  } else {
    // accept the step
    t += dt;
    x = x_out;
    dxdt = dxdt_out;
    dt = increaseStep(dt, maxRelErr);
    return true;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void RungeKuttaDormandPrince5::doStep(system_func_t& system, const vector_t& x0, const vector_t& dxdt, scalar_t t, scalar_t dt,
                                      vector_t& x_out, vector_t& dxdt_out) {
  /* Runge Kutta Dormand-Prince Butcher tableau constanst: */
  const scalar_t a2 = 1.0 / 5;
  const scalar_t a3 = 3.0 / 10;
  const scalar_t a4 = 4.0 / 5;
  const scalar_t a5 = 8.0 / 9;

  const scalar_t b21 = 1.0 / 5;

  const scalar_t b31 = 3.0 / 40;
  const scalar_t b32 = 9.0 / 40;

  const scalar_t b41 = 44.0 / 45;
  const scalar_t b42 = -56.0 / 15;
  const scalar_t b43 = 32.0 / 9;

  const scalar_t b51 = 19372.0 / 6561;
  const scalar_t b52 = -25360.0 / 2187;
  const scalar_t b53 = 64448.0 / 6561;
  const scalar_t b54 = -212.0 / 729;

  const scalar_t b61 = 9017.0 / 3168;
  const scalar_t b62 = -355.0 / 33;
  const scalar_t b63 = 46732.0 / 5247;
  const scalar_t b64 = 49.0 / 176;
  const scalar_t b65 = -5103.0 / 18656;

  const scalar_t c1 = 35.0 / 384;
  // c2 = 0
  const scalar_t c3 = 500.0 / 1113;
  const scalar_t c4 = 125.0 / 192;
  const scalar_t c5 = -2187.0 / 6784;
  const scalar_t c6 = 11.0 / 84;

  k1_ = dxdt;  // k1 = system(x, t) from previous iteration
  x_.noalias() = x0 + dt * b21 * k1_;
  system(x_, k2_, t + dt * a2);
  x_.noalias() = x0 + dt * b31 * k1_ + dt * b32 * k2_;
  system(x_, k3_, t + dt * a3);
  x_.noalias() = x0 + dt * (b41 * k1_ + b42 * k2_ + b43 * k3_);
  system(x_, k4_, t + dt * a4);
  x_.noalias() = x0 + dt * (b51 * k1_ + b52 * k2_ + b53 * k3_ + b54 * k4_);
  system(x_, k5_, t + dt * a5);
  x_.noalias() = x0 + dt * (b61 * k1_ + b62 * k2_ + b63 * k3_ + b64 * k4_ + b65 * k5_);
  system(x_, k6_, t + dt);
  // update x_out and dxdt_out (x_out can be x0 and dxdt_out can be dxdt)
  x_out = x0 + dt * (c1 * k1_ + c3 * k3_ + c4 * k4_ + c5 * k5_ + c6 * k6_);
  system(x_out, dxdt_out, t + dt);
}
/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void RungeKuttaDormandPrince5::doStep(system_func_t& system, const vector_t& x0, const vector_t& dxdt, scalar_t t, scalar_t dt,
                                      vector_t& x_out, vector_t& dxdt_out, vector_t& error) {
  const scalar_t c1 = 35.0 / 384;
  // c2 = 0
  const scalar_t c3 = 500.0 / 1113;
  const scalar_t c4 = 125.0 / 192;
  const scalar_t c5 = -2187.0 / 6784;
  const scalar_t c6 = 11.0 / 84;

  const scalar_t dc1 = c1 - 5179.0 / 57600;
  const scalar_t dc3 = c3 - 7571.0 / 16695;
  const scalar_t dc4 = c4 - 393.0 / 640;
  const scalar_t dc5 = c5 - -92097.0 / 339200;
  const scalar_t dc6 = c6 - 187.0 / 2100;
  const scalar_t dc7 = -1.0 / 40;

  doStep(system, x0, dxdt, t, dt, x_out, dxdt_out);

  // error estimate
  error.noalias() = dt * (dc1 * k1_ + dc3 * k3_ + dc4 * k4_ + dc5 * k5_ + dc6 * k6_ + dc7 * dxdt_out);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t RungeKuttaDormandPrince5::error(const vector_t& x_old, const vector_t& dxdt_old, vector_t& x_err, scalar_t dt, scalar_t absTol,
                                         scalar_t relTol) const {
  x_err = x_err.array() / (absTol + relTol * (x_old.array().abs() + std::abs(dt) * dxdt_old.array().abs()));
  return x_err.lpNorm<Eigen::Infinity>();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t RungeKuttaDormandPrince5::decreaseStep(scalar_t dt, scalar_t error) const {
  const int ERROR_ORDER = 4;
  dt *= std::max(0.9 * std::pow(error, -1.0 / (ERROR_ORDER - 1)), 0.2);
  return dt;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t RungeKuttaDormandPrince5::increaseStep(scalar_t dt, scalar_t error) const {
  const int STEPPER_ORDER = 5;
  if (error < 0.5) {
    error = std::max(std::pow(scalar_t(5.0), -STEPPER_ORDER), error);
    dt *= 0.9 * std::pow(error, -1.0 / STEPPER_ORDER);
  }
  return dt;
}

}  // namespace ocs2
