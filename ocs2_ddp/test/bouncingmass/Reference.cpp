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

#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

#include <boost/numeric/odeint.hpp>

#include <ocs2_core/Types.h>
#include <ocs2_core/integration/eigenIntegration.h>

#include "ocs2_ddp/test/bouncingmass/Reference.h"

Reference::Reference(scalar_t t0, scalar_t t1, const vector_t& p0, const vector_t& p1) {
  Create5thOrdPol(t0, t1, p0, p1);
  polV_ = polyder(polX_);
  polU_ = polyder(polV_);

  t0_ = t0;
  t1_ = t1;
}

void Reference::getInput(scalar_t time, vector_t& input) const {
  input.setZero(1);
  for (int i = 0; i < polU_.size(); i++) {
    input[0] += polU_[i] * std::pow(time, i);
  }
}

vector_t Reference::getInput(scalar_t time) const {
  vector_t input;
  getInput(time, input);
  return input;
}

void Reference::getState(scalar_t time, vector_t& x) const {
  if (time <= t1_ && time >= t0_) {
    x.setZero(3);
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
  boost::numeric::odeint::runge_kutta_dopri5<vector_t, scalar_t, vector_t, scalar_t, boost::numeric::odeint::vector_space_algebra> stepper;
  // Lambda for general system dynamics, assuming that the reference input is available
  auto model = [](const vector_t& x, vector_t& dxdt, const double t, vector_t uref) {
    matrix_t A(3, 3);
    A << 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    matrix_t B(3, 1);
    B << 0.0, 1.0, 0.0;

    dxdt = A * x + B * uref;
  };
  // pre-part of extension
  if (refPre != nullptr) {
    // Construct Lambda to represent System Dynamics with correct reference input
    auto preModel = [&refPre, &model](const vector_t& x, vector_t& dxdt, const double t) {
      vector_t uref = refPre->getInput(t);
      model(x, dxdt, t, uref);
    };
    // Construct lambda to act as observer, which will store the time and state trajectories
    scalar_array_t* timeStorePtr = &tPre_;
    vector_array_t* stateStorePtr = &xPre_;
    auto preObserver = [&timeStorePtr, &stateStorePtr](vector_t& x, scalar_t& t) {
      timeStorePtr->push_back(t);
      stateStorePtr->push_back(x);
    };

    vector_t x0;
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
    auto postModel = [&refPost, &model](const vector_t& x, vector_t& dxdt, const double t) {
      vector_t uref = refPost->getInput(t);
      model(x, dxdt, t, uref);
    };
    // Construct lambda to act as observer, which will store the time and state trajectories
    scalar_array_t* timeStorePtr = &tPost_;
    vector_array_t* stateStorePtr = &xPost_;
    auto postObserver = [&timeStorePtr, &stateStorePtr](vector_t& x, scalar_t& t) {
      timeStorePtr->push_back(t);
      stateStorePtr->push_back(x);
    };

    vector_t x0;
    getState(t1_, x0);
    scalar_t t0 = t1_;
    scalar_t t1 = t0 + delta;
    scalar_t dt = 1e-3;
    boost::numeric::odeint::integrate_adaptive(stepper, postModel, x0, t0, t1, dt, postObserver);
  }
}

void Reference::Create5thOrdPol(scalar_t t0, scalar_t t1, const vector_t& p0, const vector_t& p1) {
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

void Reference::interpolate_ext(scalar_t time, vector_t& x) const {
  const scalar_array_t* tVec;
  const vector_array_t* xVec;
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
    vector_t x;
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
