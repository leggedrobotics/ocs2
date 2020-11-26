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

#include <ocs2_core/soft_constraint/SoftConstraintPenaltyBase.h>

namespace ocs2 {

/**
 * Configuration object for the relaxed barrier penalty.
 * mu : scaling factor
 * delta: relaxation parameter, see class description
 */
struct RelaxedBarrierPenaltyConfig {
  scalar_t mu;
  scalar_t delta;
};

/**
 * Implements the relaxed barrier function for a single inequality constraint \f$ h \geq 0 \f$
 *
 * \f[
 *   p(h)=\left\lbrace
 *               \begin{array}{ll}
 *                 -\mu \ln(h) & if \quad  h > \delta, \\
 *                 -\mu \ln(\delta) + \mu \frac{1}{2} \left( \left( \frac{h-2\delta}{\delta} \right)^2 - 1 \right) & otherwise,
 *               \end{array}
 *             \right.
 * \f]
 *
 * where \f$ \mu \geq 0 \f$, and \f$ \delta \geq 0 \f$ are user defined parameters.
 */
class RelaxedBarrierSoftConstraint final : public SoftConstraintPenaltyBase {
 public:
  /**
   * Constructior
   * @param [in] config: Configuration object containing mu and delta.
   */
  explicit RelaxedBarrierSoftConstraint(const RelaxedBarrierPenaltyConfig& config) : configArray_({config}) {}

  /**
   * Constructior
   * @param [in] configArray: Array with configuration objects containing mu and delta.
   */
  explicit RelaxedBarrierSoftConstraint(std::vector<RelaxedBarrierPenaltyConfig> configArray) : configArray_(std::move(configArray)) {}

  /** Default destructor */
  virtual ~RelaxedBarrierSoftConstraint() = default;

 private:
  std::vector<RelaxedBarrierPenaltyConfig> configArray_;

  const RelaxedBarrierPenaltyConfig& getConfg(size_t i) const;

  scalar_t getPenaltyValue(size_t i, scalar_t h) const override;
  scalar_t getPenaltyDerivative(size_t i, scalar_t h) const override;
  scalar_t getPenaltySecondDerivative(size_t i, scalar_t h) const override;
};

}  // namespace ocs2
