/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

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

#include <ocs2_core/constraint/StateConstraint.h>

namespace ocs2 {

/**
 * Linear state-only constraint
 */
class LinearStateConstraint : public StateConstraint {
 public:
  /**
   * Constructor
   *
   * @param [in] h: Constant term in F * x + h = 0
   * @param [in] F: x factor in F * x + h = 0
   */
  LinearStateConstraint(vector_t h, matrix_t F);

  ~LinearStateConstraint() override = default;

  LinearStateConstraint* clone() const override;

  size_t getNumConstraints(scalar_t time) const final;

  vector_t getValue(scalar_t t, const vector_t& x, const PreComputation& /* preComputation */) const final;

  VectorFunctionLinearApproximation getLinearApproximation(scalar_t t, const vector_t& x,
                                                           const PreComputation& /* preComputation */) const final;

 public:
  vector_t h_; /**< State only constraint */
  matrix_t F_; /**< State only constraint derivative wrt. state */
};

}  // namespace ocs2
