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

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "ocs2_core/Dimensions.h"
#include "ocs2_core/misc/LinearAlgebra.h"

namespace ocs2 {

/**
 * The base class for model data.
 */
struct ModelDataBase {
  using scalar_t = typename Dimensions<0, 0>::scalar_t;
  using dynamic_vector_t = Eigen::Matrix<scalar_t, Eigen::Dynamic, 1>;
  using dynamic_matrix_t = Eigen::Matrix<scalar_t, Eigen::Dynamic, Eigen::Dynamic>;

  using scalar_array_t = typename Dimensions<0, 0>::scalar_array_t;
  using eigen_scalar_t = typename Dimensions<0, 0>::eigen_scalar_t;
  using dynamic_vector_array_t = typename Dimensions<0, 0>::dynamic_vector_array_t;
  using dynamic_matrix_array_t = typename Dimensions<0, 0>::dynamic_matrix_array_t;

  using array_t = std::vector<ModelDataBase>;
  using array2_t = std::vector<array_t>;

  /**
   * Creates a deep copy of the object.
   * @warning Cloning implies that the caller takes ownership and deletes the created object.
   * @return Pointer to a new instance.
   */
  virtual ModelDataBase* clone() const;

  /**
   * Displays all variables
   */
  virtual void display() const;

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
  scalar_t time_;
  int stateDim_;
  int inputDim_;

  // dynamics
  dynamic_vector_t dynamics_;
  dynamic_vector_t dynamicsBias_;
  dynamic_matrix_t dynamicsStateDerivative_;
  dynamic_matrix_t dynamicsInputDerivative_;

  dynamic_matrix_t dynamicsCovariance_;

  // cost
  scalar_t cost_;
  dynamic_vector_t costStateDerivative_;
  dynamic_vector_t costInputDerivative_;
  dynamic_matrix_t costStateSecondDerivative_;
  dynamic_matrix_t costInputSecondDerivative_;
  dynamic_matrix_t costInputStateDerivative_;

  // state equality constraints
  int numStateEqConstr_;
  dynamic_vector_t stateEqConstr_;
  dynamic_matrix_t stateEqConstrStateDerivative_;

  // state-input equality constraints
  int numStateInputEqConstr_;
  dynamic_vector_t stateInputEqConstr_;
  dynamic_matrix_t stateInputEqConstrStateDerivative_;
  dynamic_matrix_t stateInputEqConstrInputDerivative_;

  // inequality constraints
  int numIneqConstr_;
  scalar_array_t ineqConstr_;
  dynamic_vector_array_t ineqConstrStateDerivative_;
  dynamic_vector_array_t ineqConstrInputDerivative_;
  dynamic_matrix_array_t ineqConstrStateSecondDerivative_;
  dynamic_matrix_array_t ineqConstrInputSecondDerivative_;
  dynamic_matrix_array_t ineqConstrInputStateDerivative_;
};

}  // namespace ocs2
