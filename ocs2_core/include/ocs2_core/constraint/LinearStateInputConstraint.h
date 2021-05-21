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

#include <ocs2_core/constraint/StateInputConstraint.h>

namespace ocs2 {

/**
 * Linear state-input constraint
 */
class LinearStateInputConstraint : public StateInputConstraint {
 public:
  /**
   * Constructor
   *
   * @param[in] e: Constant term in C * x + D * u + e = 0
   * @param[in] C: x factor in C * x + D * u + e = 0
   * @param[in] D: u factor in C * x + D * u + e = 0
   */
  LinearStateInputConstraint(vector_t e, matrix_t C, matrix_t D);

  ~LinearStateInputConstraint() override = default;

  LinearStateInputConstraint* clone() const override;

  size_t getNumConstraints(scalar_t time) const final;

  vector_t getValue(scalar_t t, const vector_t& x, const vector_t& u, const PreComputation& /* preComputation */) const final;

  VectorFunctionLinearApproximation getLinearApproximation(scalar_t t, const vector_t& x, const vector_t& u,
                                                           const PreComputation& /* preComputation */) const final;

 public:
  vector_t e_; /**< State input constraint */
  matrix_t C_; /**< State input constraint derivative wrt. state */
  matrix_t D_; /**< State input constraint derivative wrt. input */
};

}  // namespace ocs2
