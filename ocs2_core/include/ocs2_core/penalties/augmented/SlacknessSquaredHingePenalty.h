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

#include "ocs2_core/penalties/augmented/AugmentedPenaltyBase.h"

namespace ocs2 {
namespace augmented {

/**
 *  Implements the augmented Lagrangian for a single inequality constraint \f$ h \geq 0 \f$ by transforming it to the following form:
 *
 *  \f[ \left\lbrace
 *              \begin{array}{ll}
 *                  h - s = 0, \\
 *                  s \geq 0.
 *              \end{array}
 *            \right.
 *  \f]
 *
 *  This leads to the following augmented-Lagrangian penalty function (referred to as PHR penalty in the corresponding paper):
 *  \f[
 *      p(h, \lambda) = \frac{1}{2 \rho} (\max\{ 0, \lambda - \rho h \}^2 - \lambda^2).
 *  \f]
 *
 *  where \rho is the scale factor of the penalty. This is then minimized with the DDP algorithm, while the maximization of the
 *  approximate dual function is done by updating the Lagrange multipliers with a gradient ascent step as such:
 * \f[
 *      \lambda^*_{k+1} = \max\{ \lambda^*_k - \alpha h^*_{k+1}, (1 - \frac{\alpha}{\rho}) \lambda^*_k \}.
 * \f]
 */
class SlacknessSquaredHingePenalty final : public AugmentedPenaltyBase {
 public:
  /**
   * Configuration object for the squared hinge penalty.
   * @param [in] scale : scaling factor. In the class description, it is referred to as \pho.
   * @param [in] stepSize: step-size for updating Lagrange multiplier. In the class description, it is referred to as \alpha.
   */
  struct Config {
    Config() : Config(10.0, 1.0) {}
    Config(scalar_t scaleParam, scalar_t stepSizeParam) : scale(scaleParam), stepSize(stepSizeParam) {}
    scalar_t scale;
    scalar_t stepSize;
  };

  /** Constructor */
  SlacknessSquaredHingePenalty(Config config) : config_(std::move(config)) {}

  /** Factory function */
  static std::unique_ptr<SlacknessSquaredHingePenalty> create(Config config) {
    return std::make_unique<SlacknessSquaredHingePenalty>(std::move(config));
  }

  ~SlacknessSquaredHingePenalty() override = default;
  SlacknessSquaredHingePenalty* clone() const override { return new SlacknessSquaredHingePenalty(*this); }
  std::string name() const override { return "SlacknessSquaredHingePenalty"; }

  scalar_t getValue(scalar_t t, scalar_t l, scalar_t h) const override {
    return (h < l / config_.scale) ? (-l * h + 0.5 * config_.scale * h * h) : (-0.5 * l * l / config_.scale);
  }
  scalar_t getDerivative(scalar_t t, scalar_t l, scalar_t h) const override {
    return (h < l / config_.scale) ? (-l + config_.scale * h) : 0.0;
  }
  scalar_t getSecondDerivative(scalar_t t, scalar_t l, scalar_t h) const override { return (h < l / config_.scale) ? config_.scale : 0.0; }

  scalar_t updateMultiplier(scalar_t t, scalar_t l, scalar_t h) const override {
    return std::max(0.0, std::max(l - config_.stepSize * config_.scale * h, (1.0 - config_.stepSize) * l));
  }
  scalar_t initializeMultiplier() const override { return 0.0; }

 private:
  SlacknessSquaredHingePenalty(const SlacknessSquaredHingePenalty& other) = default;

  const Config config_;
};

}  // namespace augmented
}  // namespace ocs2
