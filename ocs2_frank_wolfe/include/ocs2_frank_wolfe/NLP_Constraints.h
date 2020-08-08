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

#include <ocs2_core/Types.h>

namespace ocs2 {

/**
 * This class is an interface to a NLP constraints.
 */
class NLP_Constraints {
 public:
  /**
   * Default constructor.
   */
  NLP_Constraints() = default;

  /**
   * Default destructor.
   */
  virtual ~NLP_Constraints() = default;

  /**
   * Sets the current parameter vector.
   *
   * @param [in] x: The value of parameter vector.
   */
  virtual void setCurrentParameter(const vector_t& x) {}

  /**
   * Gets the linear equality constraints. \n
   * \f$ g_v = A_m x_v + b_v = 0\f$
   *
   * @param [out] g: The evaluation of the equality constraints, \f$ g_v \f$ vector.
   */
  virtual void getLinearEqualityConstraint(vector_t& g) { g.resize(0); }

  /**
   * Gets the derivative of the linear equality constraints. \n
   * \f$ g_v = A_m x_v + b_v = 0\f$
   *
   * @param [out] dgdx: The Jacobian of the equality constraints, \f$ A_m \f$ vector.
   */
  virtual void getLinearEqualityConstraintDerivative(matrix_t& dgdx) { dgdx.resize(0, 0); }

  /**
   * Gets the linear inequality constraints. \n
   * \f$ h_v = C_m x_v + d_v \geq 0\f$
   *
   * @param [out] h: The evaluation of the inequality constraints, \f$ h_v \f$ vector.
   */
  virtual void getLinearInequalityConstraint(vector_t& h) { h.resize(0); }

  /**
   * Gets the derivative of the linear inequality constraints. \n
   * \f$ h_v = C_m x_v + d_v \geq 0\f$
   *
   * @param [out] dhdx: The Jacobian of the inequality constraints, \f$ C_m \f$ vector.
   */
  virtual void getLinearInequalityConstraintDerivative(matrix_t& dhdx) { dhdx.resize(0, 0); }
};

}  // namespace ocs2
