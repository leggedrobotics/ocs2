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

#include <ocs2_core/penalties/penalties/PenaltyBase.h>

namespace ocs2 {

/**
 *  Implements the penalty for a single equality constraint \f$ h = 0 \f$. This leads to the following:
 *
 *  \f[
 *      L_{A} = \frac{\mu}{2} h^2.
 *  \f]
 *
 *  where \f$ \mu \f$ is the scale.
 */
class QuadraticPenalty final : public PenaltyBase {
 public:
  /**
   * Thos constructor sets both the scale and stepLength the same. This is a common practice in Augmented Lagrangian.
   * @param [in] scale: Scaling of the cost.
   */
  explicit QuadraticPenalty(scalar_t scale) : scale_(scale) {}

  ~QuadraticPenalty() override = default;
  QuadraticPenalty* clone() const override { return new QuadraticPenalty(*this); }
  std::string name() const override { return "QuadraticPenalty"; }

  scalar_t getValue(scalar_t t, scalar_t h) const override { return 0.5 * scale_ * h * h; }
  scalar_t getDerivative(scalar_t t, scalar_t h) const override { return scale_ * h; }
  scalar_t getSecondDerivative(scalar_t t, scalar_t h) const override { return scale_; }

 private:
  QuadraticPenalty(const QuadraticPenalty& other) = default;

  const scalar_t scale_;
};

}  // namespace ocs2
