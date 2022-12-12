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

#include <ocs2_core/Types.h>

namespace ocs2 {
namespace pipg {

/**
 * Defines the lower and upper bounds of the total cost hessian, H, and the upper bound of \f$ G^TG \f$.
 * Refer to "Proportional-Integral Projected Gradient Method for Model Predictive Control"
 * Link: https://arxiv.org/abs/2009.06980
 *
 * z := [u_{0}; x_{1}; ...; u_{n}; x_{n+1}].
 *
 * min  0.5 z' H z + z' h
 * s.t. G z = g
 *
 * H = [ R0
 *       *   Q1  P1'
 *       *   P1  R1
 *       *   *   *   Qn  Pn'
 *       *   *   *   Pn  Rn
 *       *   *   *   *   *   Q{n+1}]
 *
 * G = [-B0  I
 *       *  -A1 -B1   I
 *
 *       *   *   *   -An -Bn  I
 *       D0  0
 *       *   C1  D1   0
 *
 *       *   *   *    Cn  Dn  0]
 */
struct PipgBounds {
  PipgBounds(scalar_t muArg, scalar_t lambdaArg, scalar_t sigmaArg) : mu(muArg), lambda(lambdaArg), sigma(sigmaArg) {}

  scalar_t dualStepSize(size_t iteration) const { return (static_cast<scalar_t>(iteration) + 1.0) * mu / (2.0 * sigma); }

  scalar_t primalStepSize(size_t iteration) const { return 2.0 / ((static_cast<scalar_t>(iteration) + 1.0) * mu + 2.0 * lambda); }

  scalar_t mu = 1e-5;     // mu I <= H
  scalar_t lambda = 1.0;  // H <= lambda I
  scalar_t sigma = 1.0;   // G' G <= sigma I
};

}  // namespace pipg
}  // namespace ocs2
