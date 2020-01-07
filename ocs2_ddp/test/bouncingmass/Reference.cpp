#include <algorithm>

#include "ocs2_ddp/test/bouncingmass/Reference.h"

using DIMENSIONS = ocs2::Dimensions<3, 1>;
using scalar_t = typename DIMENSIONS::scalar_t;
using scalar_array_t = typename DIMENSIONS::scalar_array_t;
using state_vector_t = typename DIMENSIONS::state_vector_t;
using input_vector_t = typename DIMENSIONS::input_vector_t;
using state_vector_array_t = typename DIMENSIONS::state_vector_array_t;
using state_matrix_t = typename DIMENSIONS::state_matrix_t;
using state_input_matrix_t = typename DIMENSIONS::state_input_matrix_t;

Reference::Reference(scalar_t t0, scalar_t t1, state_vector_t p0, state_vector_t p1) {
  Create5thOrdPol(t0, t1, p0, p1);
  polV_ = polyder(polX_);
  polU_ = polyder(polV_);

  t0_ = t0;
  t1_ = t1;
}

void Reference::getInput(scalar_t time, input_vector_t& input) {
  input[0] = 0;
  for (int i = 0; i < polU_.size(); i++) {
    input[0] += polU_[i] * std::pow(time, i);
  }
}

input_vector_t Reference::getInput(scalar_t time) {
  input_vector_t input;
  getInput(time, input);
  return input;
}

void Reference::getState(scalar_t time, state_vector_t& x) {
  if (time <= t1_ && time >= t0_) {
    x[0] = 0;
    x[1] = 0;
    x[2] = 0;

    for (int i = 0; i < polU_.size(); i++) {
      x[0] += polX_[i] * std::pow(time, i);
      x[1] += polV_[i] * std::pow(time, i);
    }
  } else {
    interpolate_ext(time, x);
  }
}

void Reference::extendref(scalar_t delta, Reference* refPre, Reference* refPost) {
  delta_ = delta;
  boost::numeric::odeint::runge_kutta_dopri5<Eigen::Vector3d, scalar_t, Eigen::Vector3d, scalar_t,
                                             boost::numeric::odeint::vector_space_algebra>
      stepper;
  // Lambda for general system dynamics, assuming that the reference input is available
  auto model = [](const state_vector_t& x, state_vector_t& dxdt, const double t, input_vector_t uref) {
    state_matrix_t A;
    A << 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    state_input_matrix_t B;
    B << 0.0, 1.0, 0.0;

    dxdt = A * x + B * uref;
  };
  // pre-part of extension
  if (refPre != nullptr) {
    // Construct Lambda to represent System Dynamics with correct reference input
    auto preModel = [&refPre, &model](const state_vector_t& x, state_vector_t& dxdt, const double t) {
      input_vector_t uref = refPre->getInput(t);
      model(x, dxdt, t, uref);
    };
    // Construct lambda to act as observer, which will store the time and state trajectories
    scalar_array_t* timeStorePtr = &tPre_;
    state_vector_array_t* stateStorePtr = &xPre_;
    auto preObserver = [&timeStorePtr, &stateStorePtr](state_vector_t& x, scalar_t& t) {
      timeStorePtr->push_back(t);
      stateStorePtr->push_back(x);
    };

    state_vector_t x0;
    getState(t0_, x0);
    scalar_t t0 = t0_;
    scalar_t t1 = t0 - delta;
    scalar_t dt = -1e-3;

    boost::numeric::odeint::integrate_adaptive(stepper, preModel, x0, t0, t1, dt, preObserver);
    std::reverse(std::begin(tPre_), std::end(tPre_));
    std::reverse(std::begin(xPre_), std::end(xPre_));
  }

  // post-part of extension
  if (refPost != nullptr) {
    // Construct Lambda to represent System Dynamics with correct reference input
    auto postModel = [&refPost, &model](const state_vector_t& x, state_vector_t& dxdt, const double t) {
      input_vector_t uref = refPost->getInput(t);
      model(x, dxdt, t, uref);
    };
    // Construct lambda to act as observer, which will store the time and state trajectories
    scalar_array_t* timeStorePtr = &tPost_;
    state_vector_array_t* stateStorePtr = &xPost_;
    auto postObserver = [&timeStorePtr, &stateStorePtr](state_vector_t& x, scalar_t& t) {
      timeStorePtr->push_back(t);
      stateStorePtr->push_back(x);
    };

    state_vector_t x0;
    getState(t1_, x0);
    scalar_t t0 = t1_;
    scalar_t t1 = t0 + delta;
    scalar_t dt = 1e-3;
    boost::numeric::odeint::integrate_adaptive(stepper, postModel, x0, t0, t1, dt, postObserver);
  }
}

void Reference::Create5thOrdPol(scalar_t t0, scalar_t t1, state_vector_t p0, state_vector_t p1) {
  Eigen::Matrix<scalar_t, 6, 6> A;
  Eigen::Matrix<scalar_t, 6, 6> Ainv;

  A << 1, t0, std::pow(t0, 2), std::pow(t0, 3), std::pow(t0, 4), std::pow(t0, 5), 0, 1, 2 * t0, 3 * std::pow(t0, 2), 4 * std::pow(t0, 3),
      5 * std::pow(t0, 4), 0, 0, 2, 6 * t0, 12 * std::pow(t0, 2), 20 * std::pow(t0, 3), 1, t1, std::pow(t1, 2), std::pow(t1, 3),
      std::pow(t1, 4), std::pow(t1, 5), 0, 1, 2 * t1, 3 * std::pow(t1, 2), 4 * std::pow(t1, 3), 5 * std::pow(t1, 4), 0, 0, 2, 6 * t1,
      12 * std::pow(t1, 2), 20 * std::pow(t1, 3);

  Ainv = A.inverse();

  Eigen::Matrix<scalar_t, 6, 1> x;
  x << p0, p1;
  polX_ = Ainv * x;
}

void Reference::interpolate_ext(scalar_t time, state_vector_t& x) {
  scalar_array_t* tVec;
  state_vector_array_t* xVec;
  if (time < t0_) {
    tVec = &tPre_;
    xVec = &xPre_;
    x = xPre_.front();
  } else {
    tVec = &tPost_;
    xVec = &xPost_;
    x = xPost_.back();
  }

  int idx;
  for (int i = 0; i < tVec->size() - 1; i++) {
    if (time > tVec->at(i) && time < tVec->at(i + 1)) {
      idx = i;
      scalar_t fac = (time - tVec->at(idx)) / (tVec->at(idx + 1) - tVec->at(idx));
      x = fac * xVec->at(idx) + (1 - fac) * xVec->at(idx + 1);
      return;
    }
  }
}

void Reference::display() {
  std::cerr << "#########################" << std::endl;
  std::cerr << "#Pre-Extended-Trajectory#" << std::endl;
  std::cerr << "#########################" << std::endl;
  for (int i = 0; i < tPre_.size(); i++) {
    std::cerr << tPre_[i] << ";" << xPre_[i][0] << ";" << xPre_[i][1] << std::endl;
  }

  std::cerr << "#########################" << std::endl;
  std::cerr << "####Normal-Trajectory####" << std::endl;
  std::cerr << "#########################" << std::endl;

  scalar_t dt = 0.01;
  for (int i = 0; i < (t1_ - t0_) / dt; i++) {
    scalar_t t = t0_ + dt * i;
    state_vector_t x;
    getState(t, x);

    std::cerr << t << ";" << x[0] << ";" << x[1] << std::endl;
  }

  std::cerr << "##########################" << std::endl;
  std::cerr << "#Post-Extended-Trajectory#" << std::endl;
  std::cerr << "##########################" << std::endl;

  for (int i = 0; i < tPost_.size(); i++) {
    std::cerr << tPost_[i] << ";" << xPost_[i][0] << ";" << xPost_[i][1] << std::endl;
  }
}

Eigen::Matrix<scalar_t, 6, 1> Reference::polyder(Eigen::Matrix<scalar_t, 6, 1> pol) {
  Eigen::Matrix<scalar_t, 6, 1> polOld = pol;

  for (int i = 0; i < pol.size(); i++) {
    if (i < pol.size() - 1) {
      pol[i] = (i + 1) * polOld[i + 1];
    } else {
      pol[i] = 0;
    }
  }

  return pol;
}
