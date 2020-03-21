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

#pragma once

#include "PenaltyBase.h"

namespace ocs2 {

/**
 *  Implements the relaxed barrier function for a single inequality constraint \f$ h \geq 0 \f$
 *
 *   \f[
 *   p(h)=\left\lbrace
 *               \begin{array}{ll}
 *                 -\mu \ln(h) & if \quad  h > \delta, \\
 *                 -\mu \ln(\delta) + \mu \frac{1}{2} \left( \left( \frac{h-2\delta}{\delta} \right)^2 - 1 \right) & otherwise,
 *               \end{array}
 *             \right.
 * \f]
 *
 *  where \f$ \mu \geq 0 \f$, and \f$ \delta \geq 0 \f$ are user defined parameters.
 *
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class RelaxedBarrierPenalty final : public PenaltyBase<STATE_DIM, INPUT_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using scalar_t = typename PenaltyBase<STATE_DIM, INPUT_DIM>::scalar_t;

  RelaxedBarrierPenalty(scalar_t mu, scalar_t delta) : mu_(mu), delta_(delta){};
  virtual ~RelaxedBarrierPenalty() = default;

 private:
  scalar_t mu_;
  scalar_t delta_;

  scalar_t getPenaltyFunctionValue(scalar_t h) const override {
    if (h > delta_) {
      return -mu_ * log(h);
    } else {
      return mu_ * (-log(delta_) + scalar_t(0.5) * pow((h - 2.0 * delta_) / delta_, 2.0) - scalar_t(0.5));
    };
  };

  scalar_t getPenaltyFunctionDerivative(scalar_t h) const override {
    if (h > delta_) {
      return -mu_ / h;
    } else {
      return mu_ * ((h - 2.0 * delta_) / (delta_ * delta_));
    };
  };

  scalar_t getPenaltyFunctionSecondDerivative(scalar_t h) const override {
    if (h > delta_) {
      return mu_ / (h * h);
    } else {
      return mu_ / (delta_ * delta_);
    };
  };
};
}  // namespace ocs2
