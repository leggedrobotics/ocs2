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

#include <ocs2_core/soft_constraint/penalties/PenaltyBase.h>

namespace ocs2 {

/**
 * Implements the smooth-absolute function for a single equality constraint \f$ h = 0 \f$
 *
 * \f[
 *   p(h) = \mu sqrt(x^2 + \delta^2).
 * \f]
 *
 * where \f$ \mu > 0 \f$, and \f$ \delta > 0 \f$ are user defined parameters. Note that
 * \f$ \delta \f$ defines the error bound between the absolute function and its approximation:
 *
 * \f[
 *   | x - sqrt(x^2 + \delta^2) | \leq \delta, \quad \forall x \in R
 * \f]
 */
class SmoothAbsolutePenalty final : public PenaltyBase {
 public:
  /**
   * Configuration object for the smooth absolute penalty.
   * mu : scaling factor
   * delta: relaxation parameter, see class description
   */
  struct Config {
    Config() : Config(1.0, 1e-2) {}
    Config(scalar_t muParam, scalar_t deltaParam) : mu(muParam), delta(deltaParam) {}
    scalar_t mu;
    scalar_t delta;
  };

  /**
   * Constructor
   * @param [in] config: Configuration object containing mu and delta.
   */
  explicit SmoothAbsolutePenalty(Config config) : config_(std::move(config)) {}

  /** Default destructor */
  ~SmoothAbsolutePenalty() override = default;

  SmoothAbsolutePenalty* clone() const override { return new SmoothAbsolutePenalty(*this); }

  scalar_t getValue(scalar_t t, scalar_t h) const override;
  scalar_t getDerivative(scalar_t t, scalar_t h) const override;
  scalar_t getSecondDerivative(scalar_t t, scalar_t h) const override;

 private:
  SmoothAbsolutePenalty(const SmoothAbsolutePenalty& other) = default;

  Config config_;
};

}  // namespace ocs2
