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
 * Implements the smooth-absolute function for a single equality constraint \f$ h = 0 \f$
 *
 * \f[
 *   p(h) = \mu sqrt(h^2 + \delta^2).
 * \f]
 *
 * where \f$ \mu > 0 \f$, and \f$ \delta > 0 \f$ are scale and relaxation parameters respectively. Note that
 * \f$ \delta \f$ defines the error bound between the absolute function and its approximation:
 *
 * \f[
 *   | x - sqrt(h^2 + \delta^2) | \leq \delta, \quad \forall x \in R
 * \f]
 *
 *  This is then minimized with the solver, while the Lagrange multipliers are updated as:
 * \f[
 *      \lambda^*_{k+1} = \lambda^*_k - \alpha h^*_{k+1}.
 * \f]
 *
 * with \f$ alpha \f$ is the step-length. In Augmented Lagrangian method this is often set to \f$ \mu \f$.
 */
class SmoothAbsolutePenalty final : public AugmentedPenaltyBase {
 public:
  /**
   * Configuration object for the smooth absolute penalty.
   * scale: scaling factor, see class description
   * relaxation: relaxation parameter, see class description
   * stepLenght: step-length parameter, see class description
   */
  struct Config {
    Config() : Config(100.0, 1e-2, 0.0) {}
    Config(scalar_t scaleParam, scalar_t relaxationParam, scalar_t stepSizeParam)
        : scale(scaleParam), relaxation(relaxationParam), stepSize(stepSizeParam) {}
    scalar_t scale;
    scalar_t relaxation;
    scalar_t stepSize;
  };

  /** Constructor */
  explicit SmoothAbsolutePenalty(Config config) : config_(std::move(config)) {}

  /** Factory function */
  static std::unique_ptr<SmoothAbsolutePenalty> create(Config config) { return std::make_unique<SmoothAbsolutePenalty>(std::move(config)); }

  ~SmoothAbsolutePenalty() override = default;
  SmoothAbsolutePenalty* clone() const override { return new SmoothAbsolutePenalty(*this); }
  std::string name() const override { return "SmoothAbsolutePenalty"; }

  scalar_t getValue(scalar_t t, scalar_t l, scalar_t h) const override {
    return -l * h + config_.scale * sqrt(h * h + config_.relaxation * config_.relaxation);
  }
  scalar_t getDerivative(scalar_t t, scalar_t l, scalar_t h) const override {
    return -l + config_.scale * h / sqrt(h * h + config_.relaxation * config_.relaxation);
  }
  scalar_t getSecondDerivative(scalar_t t, scalar_t l, scalar_t h) const override {
    const scalar_t deltaSquare = config_.relaxation * config_.relaxation;
    return config_.scale * deltaSquare / pow(h * h + deltaSquare, 1.5);
  }

  scalar_t updateMultiplier(scalar_t t, scalar_t l, scalar_t h) const override { return l - config_.stepSize * config_.scale * h; }
  scalar_t initializeMultiplier() const override { return 0.0; }

 private:
  SmoothAbsolutePenalty(const SmoothAbsolutePenalty& other) = default;

  const Config config_;
};

}  // namespace augmented
}  // namespace ocs2
