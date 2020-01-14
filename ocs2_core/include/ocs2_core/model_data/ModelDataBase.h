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
#include <string>
#include <vector>

#include "ocs2_core/Dimensions.h"
#include "ocs2_core/misc/LinearAlgebra.h"

namespace ocs2 {

/**
 * The base class for model data.
 */
struct ModelDataBase {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using scalar_t = typename Dimensions<0, 0>::scalar_t;
  using dynamic_vector_t = Eigen::Matrix<scalar_t, Eigen::Dynamic, 1>;
  using dynamic_matrix_t = Eigen::Matrix<scalar_t, Eigen::Dynamic, Eigen::Dynamic>;

  using scalar_array_t = typename Dimensions<0, 0>::scalar_array_t;
  using eigen_scalar_t = typename Dimensions<0, 0>::eigen_scalar_t;
  using dynamic_vector_array_t = typename Dimensions<0, 0>::dynamic_vector_array_t;
  using dynamic_matrix_array_t = typename Dimensions<0, 0>::dynamic_matrix_array_t;

  using self_t = ModelDataBase;
  using array_t = std::vector<self_t, Eigen::aligned_allocator<self_t>>;
  using array2_t = std::vector<array_t, Eigen::aligned_allocator<array_t>>;

  /**
   * Default constructor.
   */
  ModelDataBase() = default;

  /**
   * Default destructor.
   */
  virtual ~ModelDataBase() = default;

  /**
   * Copy constructor.
   */
  ModelDataBase(const ModelDataBase& rhs) = default;

  /**
   * Creates a deep copy of the object.
   * @warning Cloning implies that the caller takes ownership and deletes the created object.
   * @return Pointer to a new instance.
   */
  virtual ModelDataBase* clone() const { return new self_t(*this); }

  /**
   * Displays all variables
   */
  virtual void display() {
    std::cerr << std::endl;
    std::cerr << "time: " << time_ << "\n";
    std::cerr << "Dynamics: " << dynamics_.transpose() << "\n";
    std::cerr << "Dynamics State Derivative: " << dynamicsStateDerivative_ << "\n";
    std::cerr << "Dynamics Input Derivative: " << dynamicsInputDerivative_ << "\n";

    std::cerr << "Cost: " << cost_ << "\n";
    std::cerr << "Cost State Derivative: " << costStateDerivative_ << "\n";
    std::cerr << "Cost Input Derivative: " << costInputDerivative_ << "\n";
    std::cerr << "Cost State Second Derivative: " << costStateSecondDerivative_ << "\n";
    std::cerr << "Cost Input Second Derivative: " << costInputSecondDerivative_ << "\n";
    std::cerr << "Cost Input State Derivative:  " << costInputStateDerivative_ << "\n";
    std::cerr << "Cost Input State Derivative:  " << costInputStateDerivative_ << "\n";
    std::cerr << std::endl;
  }

  /**
   * Checks whether the size of the member variables matches the state and input dimension.
   *
   * @param [in] stateDim: The dimension of the state vector.
   * @param [in] inputDim: The dimension of the input vecotr.
   */
  void checkSizes(int stateDim, int inputDim) const {
    assert(stateDim_ == stateDim);
    assert(inputDim_ == inputDim);

    // dynamics flow map
    assert(dynamics_.size() == stateDim);
    assert(dynamicsStateDerivative_.rows() == stateDim);
    assert(dynamicsStateDerivative_.cols() == stateDim);
    assert(dynamicsInputDerivative_.rows() == stateDim);
    assert(dynamicsInputDerivative_.cols() == inputDim);

    // cost
    assert(costStateDerivative_.size() == stateDim);
    assert(costStateSecondDerivative_.rows() == stateDim);
    assert(costStateSecondDerivative_.cols() == stateDim);
    assert(costInputDerivative_.size() == inputDim);
    assert(costInputSecondDerivative_.rows() == inputDim);
    assert(costInputSecondDerivative_.cols() == inputDim);
    assert(costInputStateDerivative_.rows() == inputDim);
    assert(costInputStateDerivative_.cols() == stateDim);

    // state equality constraints
    assert(stateEqConstr_.size() == numStateEqConstr_);
    assert(stateEqConstrStateDerivative_.rows() == stateEqConstr_.size());
    assert(stateEqConstrStateDerivative_.cols() == stateDim);

    // state-input equality constraints
    assert(stateInputEqConstr_.size() == numStateInputEqConstr_);
    assert(stateInputEqConstrStateDerivative_.rows() == stateInputEqConstr_.size());
    assert(stateInputEqConstrStateDerivative_.cols() == stateDim);
    assert(stateInputEqConstrInputDerivative_.rows() == stateInputEqConstr_.size());
    assert(stateInputEqConstrInputDerivative_.cols() == inputDim);

    // inequality constraints
    assert(ineqConstr_.size() == numIneqConstr_);
    assert(ineqConstrStateDerivative_.size() == ineqConstr_.size());
    assert(ineqConstrInputDerivative_.size() == ineqConstr_.size());
    assert(ineqConstrStateSecondDerivative_.size() == ineqConstr_.size());
    assert(ineqConstrInputSecondDerivative_.size() == ineqConstr_.size());
    assert(ineqConstrInputStateDerivative_.size() == ineqConstr_.size());
    for (int i = 0; i < ineqConstr_.size(); i++) {
      assert(ineqConstrStateDerivative_[i].size() == stateDim);
      assert(ineqConstrInputDerivative_[i].size() == inputDim);
      assert(ineqConstrStateSecondDerivative_[i].rows() == stateDim);
      assert(ineqConstrStateSecondDerivative_[i].cols() == stateDim);
      assert(ineqConstrInputSecondDerivative_[i].rows() == inputDim);
      assert(ineqConstrInputSecondDerivative_[i].cols() == inputDim);
      assert(ineqConstrInputStateDerivative_[i].rows() == inputDim);
      assert(ineqConstrInputStateDerivative_[i].cols() == stateDim);
    }  // end of i loop
  }

  /**
   * Checks the numerical properties of the cost function and its derivatives.
   * @return The description of the error. If there was no error it would be empty;
   */
  std::string checkCostProperties() const {
    std::string errorDescription;

    if (cost_ != cost_) {
      errorDescription += "Intermediate cost is is not finite.";
    }
    if (!costStateDerivative_.allFinite()) {
      errorDescription += "Intermediate cost first derivative w.r.t. state is is not finite.";
    }
    if (!costStateSecondDerivative_.allFinite()) {
      errorDescription += "Intermediate cost second derivative w.r.t. state is is not finite.";
    }
    if (!costStateSecondDerivative_.isApprox(costStateSecondDerivative_.transpose())) {
      errorDescription += "Intermediate cost second derivative w.r.t. state is is not self-adjoint.";
    }
    if (LinearAlgebra::eigenvalues(costStateSecondDerivative_).real().minCoeff() < -Eigen::NumTraits<scalar_t>::epsilon()) {
      errorDescription += "Q matrix is not positive semi-definite. It's smallest eigenvalue is " +
                          std::to_string(LinearAlgebra::eigenvalues(costStateSecondDerivative_).real().minCoeff()) + ".";
    }
    if (!costInputDerivative_.allFinite()) {
      errorDescription += "Intermediate cost first derivative w.r.t. input is is not finite.";
    }
    if (!costInputSecondDerivative_.allFinite()) {
      errorDescription += "Intermediate cost second derivative w.r.t. input is is not finite.";
    }
    if (!costInputSecondDerivative_.isApprox(costInputSecondDerivative_.transpose())) {
      errorDescription += "Intermediate cost second derivative w.r.t. input is is not self-adjoint.";
    }
    if (!costInputStateDerivative_.allFinite()) {
      errorDescription += "Intermediate cost second derivative w.r.t. input-state is is not finite.";
    }
    if (costInputSecondDerivative_.ldlt().rcond() < Eigen::NumTraits<scalar_t>::epsilon()) {
      errorDescription += "R matrix is not invertible. It's reciprocal condition number is " +
                          std::to_string(costInputSecondDerivative_.ldlt().rcond()) + ".";
    }
    if (LinearAlgebra::eigenvalues(costInputSecondDerivative_).real().minCoeff() < Eigen::NumTraits<scalar_t>::epsilon()) {
      errorDescription += "R matrix is not positive definite. It's smallest eigenvalue is " +
                          std::to_string(LinearAlgebra::eigenvalues(costInputSecondDerivative_).real().minCoeff()) + ".";
    }

    return errorDescription;
  }

  /**
   * Variables
   */
  scalar_t time_;
  int stateDim_;
  int inputDim_;

  // dynamics
  dynamic_vector_t dynamics_;
  dynamic_matrix_t dynamicsStateDerivative_;
  dynamic_matrix_t dynamicsInputDerivative_;

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
