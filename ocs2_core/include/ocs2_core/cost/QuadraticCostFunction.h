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

#include "ocs2_core/cost/CostFunctionBase.h"

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
   * @param [in] xNominalIntermediate: \f$ x_{n}\f$
   * @param [in] uNominalIntermediate: \f$ u_{n}\f$
   * @param [in] xNominalFinal: \f$ x_{f}\f$
   * @param [in] QFinal: \f$ Q_{f}\f$
   */
  QuadraticCostFunction(const matrix_t& Q, const matrix_t& R, const vector_t& xNominalIntermediate, const vector_t& uNominalIntermediate,
                        const matrix_t& QFinal, const vector_t& xNominalFinal, const matrix_t& P = matrix_t());

  /**
   * Destructor
   */
  virtual ~QuadraticCostFunction() = default;

  /**
   * Returns pointer to the class.
   *
   * @return A raw pointer to the class.
   */
  QuadraticCostFunction* clone() const override;

  /**
   * Sets the current time, state, and control input.
   *
   * @param [in] t: Current time.
   * @param [in] x: Current state vector.
   * @param [in] u: Current input vector.
   */
  void setCurrentStateAndControl(scalar_t t, const vector_t& x, const vector_t& u) override;

  /**
   * Sets the current time, state, control input, and desired state and input.
   *
   * @param [in] t: Current time.
   * @param [in] x: Current state vector.
   * @param [in] u: Current input vector.
   * @param [in] xNominalIntermediate: Intermediate desired state vector.
   * @param [in] uNominalIntermediate: Intermediate desired input vector.
   * @param [in] xNominalFinal: Final desired state vector.
   */
  virtual void setCurrentStateAndControl(scalar_t t, const vector_t& x, const vector_t& u, const vector_t& xNominalIntermediate,
                                         const vector_t& uNominalIntermediate, const vector_t& xNominalFinal);

  /**
   * Get the intermediate cost.
   *
   * @return The intermediate cost value.
   */
  scalar_t getCost() override;

  /**
   * Get the state derivative of the intermediate cost.
   *
   * @return First order derivative of the intermediate cost with respect to state vector.
   */
  vector_t getCostDerivativeState() override;

  /**
   * Get state second order derivative of the intermediate cost.
   *
   * @return Second order derivative of the intermediate cost with respect to state vector.
   */
  matrix_t getCostSecondDerivativeState() override;

  /**
   * Get control input derivative of the intermediate cost.
   *
   * @return First order derivative of the intermediate cost with respect to input vector.
   */
  vector_t getCostDerivativeInput() override;

  /**
   * Get control input second derivative of the intermediate cost.
   *
   * @return Second order derivative of the intermediate cost with respect to input vector.
   */
  matrix_t getCostSecondDerivativeInput() override;

  /**
   * Get the input-state derivative of the intermediate cost.
   *
   * @return Second order derivative of the intermediate cost with respect to input vector and state.
   */
  matrix_t getCostDerivativeInputState() override;

  /**
   * Get the terminal cost.
   *
   * @return The final cost value.
   */
  scalar_t getTerminalCost() override;

  /**
   * Get the terminal cost state derivative.
   *
   * @return First order final cost derivative with respect to state vector.
   */
  vector_t getTerminalCostDerivativeState() override;

  /**
   * Get the terminal cost state second derivative
   *
   * @return Second order final cost derivative with respect to state vector.
   */
  matrix_t getTerminalCostSecondDerivativeState() override;

 protected:
  matrix_t Q_;
  matrix_t R_;
  matrix_t P_;
  matrix_t QFinal_;

  vector_t xNominalIntermediate_;
  vector_t uNominalIntermediate_;

  vector_t xIntermediateDeviation_;
  vector_t uIntermediateDeviation_;

  vector_t xNominalFinal_;
};

}  // namespace ocs2
