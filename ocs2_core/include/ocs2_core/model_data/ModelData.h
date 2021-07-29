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

#include <ostream>
#include <string>
#include <vector>

#include "ocs2_core/Types.h"

namespace ocs2 {

/**
 * The optimal control problem model data.
 */
struct ModelData {
  /**
   * Displays all variables
   */
  void display() const;

  /**
   * Checks whether the size of the member variables matches the state and input dimension.
   *
   * @param [in] stateDim: The dimension of the state vector.
   * @param [in] inputDim: The dimension of the input vecotr.
   */
  void checkSizes(int stateDim, int inputDim) const;

  /**
   * Checks the numerical properties of the cost function and its derivatives.
   * @return The description of the error. If there was no error it would be empty;
   */
  std::string checkCostProperties() const;

  /**
   * Checks the numerical properties of the dynamics derivatives.
   * @return The description of the error. If there was no error it would be empty;
   */
  std::string checkDynamicsDerivativsProperties() const;

  /**
   * Checks the numerical properties of the constraint functions and derivatives.
   * @return The description of the error. If there was no error it would be empty;
   */
  std::string checkConstraintProperties() const;

  /**
   * Variables
   */
  scalar_t time_ = 0;
  int stateDim_ = 0;
  int inputDim_ = 0;

  // dynamics
  VectorFunctionLinearApproximation dynamics_;
  vector_t dynamicsBias_;
  matrix_t dynamicsCovariance_;

  // cost
  ScalarFunctionQuadraticApproximation cost_;

  // state equality constraints
  VectorFunctionLinearApproximation stateEqConstr_;

  // state-input equality constraints
  VectorFunctionLinearApproximation stateInputEqConstr_;

  // inequality constraints
  VectorFunctionQuadraticApproximation ineqConstr_;
};

std::ostream& operator<<(std::ostream& out, const ModelData& data);

}  // namespace ocs2
