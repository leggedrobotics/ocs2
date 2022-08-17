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

#include <memory>

#include "ocs2_core/penalties/augmented/AugmentedPenaltyBase.h"

namespace ocs2 {
namespace augmented {

/**
 *  Implements the augmented Lagrangian for a single equality constraint \f$ h = 0 \f$. This leads to the following
 *  augmented Lagrangian:
 *
 *  \f[
 *      L_{A} = L * \lambda h + \frac{\mu}{2} h^2.
 *  \f]
 *
 *  where \f$ L \f$ is the Lagrangian and \f$ \mu \f$ is the scale, while the remaining terms form the penalty
 *  function \f$ p(h, \lambda) \f$.
 *
 *  This is then minimized with the solver, while the Lagrange multipliers are updated as:
 * \f[
 *      \lambda^*_{k+1} = \lambda^*_k - \alpha h^*_{k+1}.
 * \f]
 *
 * with \f$ alpha \f$ is the step-length. In Augmented Lagrangian method this is often set to \f$ \mu \f$.
 */
class QuadraticPenalty final : public AugmentedPenaltyBase {
 public:
  /**
   * Configuration object for the quadratic penalty.
   * scale: scaling factor, see class description
   * stepSize: step-length parameter, see class description
   */
  struct Config {
    Config() : Config(100.0, 0.0) {}
    Config(scalar_t scaleParam, scalar_t stepSizeParam) : scale(scaleParam), stepSize(stepSizeParam) {}
    scalar_t scale;
    scalar_t stepSize;
  };

  /** Constructor */
  explicit QuadraticPenalty(Config config) : config_(std::move(config)) {}

  /** Factory function */
  static std::unique_ptr<QuadraticPenalty> create(Config config) { return std::make_unique<QuadraticPenalty>(std::move(config)); }

  ~QuadraticPenalty() override = default;
  QuadraticPenalty* clone() const override { return new QuadraticPenalty(*this); }
  std::string name() const override { return "QuadraticPenalty"; }

  scalar_t getValue(scalar_t t, scalar_t l, scalar_t h) const override { return -l * h + 0.5 * config_.scale * h * h; }
  scalar_t getDerivative(scalar_t t, scalar_t l, scalar_t h) const override { return -l + config_.scale * h; }
  scalar_t getSecondDerivative(scalar_t t, scalar_t l, scalar_t h) const override { return config_.scale; }

  scalar_t updateMultiplier(scalar_t t, scalar_t l, scalar_t h) const override { return l - config_.stepSize * config_.scale * h; }
  scalar_t initializeMultiplier() const override { return 0.0; }

 private:
  QuadraticPenalty(const QuadraticPenalty& other) = default;

  const Config config_;
};

}  // namespace augmented
}  // namespace ocs2
