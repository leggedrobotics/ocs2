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

#include <utility>

#include <ocs2_core/Types.h>
#include <ocs2_core/cost/CostFunctionBase.h>

namespace ocs2 {

/**
 * Quadratic Cost Function.
 */
class QuadraticCostFunction : public CostFunctionBase {
 public:
  /**
   * Constructor for the running and final cost function defined as the following:
   * - \f$ L = 0.5(x-x_{n})' Q (x-x_{n}) + 0.5(u-u_{n})' R (u-u_{n}) + (u-u_{n})' P (x-x_{n}) \f$
   * - \f$ \Phi = 0.5(x-x_{f})' Q_{f} (x-x_{f}) \f$.
   * @param [in] Q: \f$ Q \f$
   * @param [in] R: \f$ R \f$
   * @param [in] xNominal: \f$ x_{n}\f$
   * @param [in] uNominal: \f$ u_{n}\f$
   * @param [in] xNominalFinal: \f$ x_{f}\f$
   * @param [in] QFinal: \f$ Q_{f}\f$
   */
  QuadraticCostFunction(const matrix_t& Q, const matrix_t& R, const vector_t& xNominal, const vector_t& uNominal, const matrix_t& QFinal,
                        const vector_t& xNominalFinal, const matrix_t& P = matrix_t());

  /** Destructor */
  ~QuadraticCostFunction() override = default;

  /** Clone */
  QuadraticCostFunction* clone() const override;

  /**
   * Get the desired state and input for cost evaluation.
   *
   * Default implementation uses costDesiredTrajectoriesPtr_ if available, otherwise xNominal and uNominal are used.
   * Override this method if another behavior is desired.
   *
   * @param [in] t: current time.
   * @param [in] x: current state.
   * @param [in] u: current input.
   * @return pair of nominal state and input.
   */
  virtual std::pair<vector_t, vector_t> getNominalStateInput(scalar_t t, const vector_t& x, const vector_t& u);

  /**
   * Get the desired final state for cost evaluation.
   *
   * Default implementation uses costDesiredTrajectoriesPtr_ if available, otherwise xNominalFinal is used.
   * Override this method if another behavior is desired.
   *
   * @param [in] t: current time.
   * @param [in] x: current state.
   * @return nominal final state.
   */
  virtual vector_t getNominalFinalState(scalar_t t, const vector_t& x);

  scalar_t cost(scalar_t t, const vector_t& x, const vector_t& u) override;
  scalar_t finalCost(scalar_t t, const vector_t& x) override;
  ScalarFunctionQuadraticApproximation costQuadraticApproximation(scalar_t t, const vector_t& x, const vector_t& u) override;
  ScalarFunctionQuadraticApproximation finalCostQuadraticApproximation(scalar_t t, const vector_t& x) override;

 protected:
  matrix_t Q_;
  matrix_t R_;
  matrix_t P_;
  matrix_t QFinal_;

  vector_t xNominal_;
  vector_t uNominal_;
  vector_t xNominalFinal_;
};

}  // namespace ocs2
