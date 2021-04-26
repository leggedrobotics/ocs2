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
#include <utility>

namespace ocs2 {

/**
 * Soft constraint base class
 */
class SoftConstraintBase {
 public:
  /** Default constructor */
  SoftConstraintBase() = default;

  /** Default destructor */
  virtual ~SoftConstraintBase() = default;

  /** Clones the class. */
  virtual SoftConstraintBase* clone() const;

  /** Get the penalty value and constraint violation */
  virtual std::pair<scalar_t, vector_t> intermediateStateInputPenalty(scalar_t t, const vector_t& x, const vector_t& u);
  virtual std::pair<scalar_t, vector_t> intermediateStatePenalty(scalar_t t, const vector_t& x);

  virtual std::pair<scalar_t, vector_t> preJumpStateInputPenalty(scalar_t t, const vector_t& x, const vector_t& u);
  virtual std::pair<scalar_t, vector_t> preJumpStatePenalty(scalar_t t, const vector_t& x);

  virtual std::pair<scalar_t, vector_t> finalPenalty(scalar_t t, const vector_t& x);

  virtual std::pair<ScalarFunctionQuadraticApproximation, vector_t> stateInputPenaltyQuadraticApproximation(scalar_t t, const vector_t& x,
                                                                                                            const vector_t& u);
  virtual std::pair<ScalarFunctionQuadraticApproximation, vector_t> statePenaltyQuadraticApproximation(scalar_t t, const vector_t& x);

  virtual std::pair<ScalarFunctionQuadraticApproximation, vector_t> preJumpPenaltyQuadraticApproximation(scalar_t t, const vector_t& x,
                                                                                                         const vector_t& u);
  virtual std::pair<ScalarFunctionQuadraticApproximation, vector_t> preJumpPenaltyQuadraticApproximation(scalar_t t, const vector_t& x);

  virtual std::pair<ScalarFunctionQuadraticApproximation, vector_t> finalPenaltyQuadraticApproximation(scalar_t t, const vector_t& x);

 protected:
  /** Copy constructor */
  SoftConstraintBase(const SoftConstraintBase& rhs) = default;
};

}  // end of namespace ocs2
