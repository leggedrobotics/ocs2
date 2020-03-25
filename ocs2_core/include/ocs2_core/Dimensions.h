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
#include <vector>

#include "ocs2_core/Types.h"

namespace ocs2 {

/**
 * This class defines the types which are used throughout this package.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 */
template <int STATE_DIM, int INPUT_DIM>
class Dimensions {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * Enum for Dimensions
   */
  enum DIMS {
    /** The State space dimension. */
    STATE_DIM_ = STATE_DIM,
    /** The control input space dimension. */
    INPUT_DIM_ = INPUT_DIM,
    /** The maximum permitted number of state-input constraints. */
    MAX_CONSTRAINT1_DIM_ = INPUT_DIM,
    /** The maximum permitted number of state-only constraints. */
    MAX_CONSTRAINT2_DIM_ = INPUT_DIM
  };

  /* Forward types for backwards compatibility, use types in ocs2_core/Types.h instead */
  using size_array_t = ocs2::size_array_t;
  using size_array2_t = ocs2::size_array2_t;
  using scalar_t = ocs2::scalar_t;
  using scalar_array_t = ocs2::scalar_array_t;
  using scalar_array2_t = ocs2::scalar_array2_t;
  using scalar_array3_t = ocs2::scalar_array3_t;
  using dynamic_vector_t = ocs2::dynamic_vector_t;
  using dynamic_vector_array_t = ocs2::dynamic_vector_array_t;
  using dynamic_vector_array2_t = ocs2::dynamic_vector_array2_t;
  using dynamic_matrix_t = ocs2::dynamic_matrix_t;
  using dynamic_matrix_array_t = ocs2::dynamic_matrix_array_t;
  using dynamic_matrix_array2_t = ocs2::dynamic_matrix_array2_t;
  using eigen_scalar_t = ocs2::eigen_scalar_t;
  using eigen_scalar_array_t = ocs2::eigen_scalar_array_t;
  using eigen_scalar_array2_t = ocs2::eigen_scalar_array2_t;
  using eigen_scalar_array3_t = ocs2::eigen_scalar_array3_t;

  /** Fixed-size state vector type with size \f$ n_x \f$ . */
  using state_vector_t = Eigen::Matrix<scalar_t, STATE_DIM, 1>;
  /** State vector trajectory type. */
  using state_vector_array_t = std::vector<state_vector_t, Eigen::aligned_allocator<state_vector_t>>;
  /** Array of state vector trajectory type. */
  using state_vector_array2_t = std::vector<state_vector_array_t, Eigen::aligned_allocator<state_vector_array_t>>;
  /** Array of arrays of state vector trajectory type. */
  using state_vector_array3_t = std::vector<state_vector_array2_t, Eigen::aligned_allocator<state_vector_array2_t>>;

  /** Fixed-size state matrix type with size \f$ n_x * n_x \f$ . */
  using state_matrix_t = Eigen::Matrix<scalar_t, STATE_DIM, STATE_DIM>;
  /** State matrix trajectory type. */
  using state_matrix_array_t = std::vector<state_matrix_t, Eigen::aligned_allocator<state_matrix_t>>;
  /** Array of state matrix trajectory type. */
  using state_matrix_array2_t = std::vector<state_matrix_array_t, Eigen::aligned_allocator<state_matrix_array_t>>;
  /** Array of arrays of state matrix trajectory type. */
  using state_matrix_array3_t = std::vector<state_matrix_array2_t, Eigen::aligned_allocator<state_matrix_array2_t>>;

  /** Fixed-size state-input matrix type with size \f$ n_x * n_u \f$ . */
  using state_input_matrix_t = Eigen::Matrix<scalar_t, STATE_DIM, INPUT_DIM>;
  /** State-input matrix trajectory type. */
  using state_input_matrix_array_t = std::vector<state_input_matrix_t, Eigen::aligned_allocator<state_input_matrix_t>>;
  /** Array of state-input matrix trajectory type. */
  using state_input_matrix_array2_t = std::vector<state_input_matrix_array_t, Eigen::aligned_allocator<state_input_matrix_array_t>>;
  /** Array of arrays of state-input matrix trajectory type. */
  using state_input_matrix_array3_t = std::vector<state_input_matrix_array2_t, Eigen::aligned_allocator<state_input_matrix_array2_t>>;

  /** Fixed-size input_state matrix type with size \f$ n_u * n_x \f$ . */
  using input_state_matrix_t = Eigen::Matrix<scalar_t, INPUT_DIM, STATE_DIM>;
  /** Input_state matrix trajectory type. */
  using input_state_matrix_array_t = std::vector<input_state_matrix_t, Eigen::aligned_allocator<input_state_matrix_t>>;
  /** Array of input_state matrix trajectory type. */
  using input_state_matrix_array2_t = std::vector<input_state_matrix_array_t, Eigen::aligned_allocator<input_state_matrix_array_t>>;
  /** Array of arrays of input_state matrix trajectory type. */
  using input_state_matrix_array3_t = std::vector<input_state_matrix_array2_t, Eigen::aligned_allocator<input_state_matrix_array2_t>>;

  /** Fixed-size control input vector type with size \f$ n_u \f$ . */
  using input_vector_t = Eigen::Matrix<scalar_t, INPUT_DIM, 1>;
  /** Control input vector trajectory type. */
  using input_vector_array_t = std::vector<input_vector_t, Eigen::aligned_allocator<input_vector_t>>;
  /** Array of control input vector trajectory type. */
  using input_vector_array2_t = std::vector<input_vector_array_t, Eigen::aligned_allocator<input_vector_array_t>>;
  /** Array of arrays of control input vector trajectory type. */
  using input_vector_array3_t = std::vector<input_vector_array2_t, Eigen::aligned_allocator<input_vector_array2_t>>;

  /** Fixed-size control input matrix type with size \f$ n_u \f$ . */
  using input_matrix_t = Eigen::Matrix<scalar_t, INPUT_DIM, INPUT_DIM>;
  /** Control input matrix trajectory type. */
  using input_matrix_array_t = std::vector<input_matrix_t, Eigen::aligned_allocator<input_matrix_t>>;
  /** Array of control input matrix trajectory type. */
  using input_matrix_array2_t = std::vector<input_matrix_array_t, Eigen::aligned_allocator<input_matrix_array_t>>;
  /** Array of arrays of control input matrix trajectory type. */
  using input_matrix_array3_t = std::vector<input_matrix_array2_t, Eigen::aligned_allocator<input_matrix_array2_t>>;

  /** Dynamic-size by n_x matrix type. */
  using dynamic_state_matrix_t = Eigen::Matrix<scalar_t, Eigen::Dynamic, STATE_DIM>;
  /** Dynamic-state matrix trajectory type. */
  using dynamic_state_matrix_array_t = std::vector<dynamic_state_matrix_t, Eigen::aligned_allocator<dynamic_state_matrix_t>>;
  /** Array of dynamic-state matrix trajectory type. */
  using dynamic_state_matrix_array2_t = std::vector<dynamic_state_matrix_array_t, Eigen::aligned_allocator<dynamic_state_matrix_array_t>>;

  /** n_x by dynamic-size matrix type. */
  using state_dynamic_matrix_t = Eigen::Matrix<scalar_t, STATE_DIM, Eigen::Dynamic>;
  /** state_dynamic matrix trajectory type. */
  using state_dynamic_matrix_array_t = std::vector<state_dynamic_matrix_t, Eigen::aligned_allocator<state_dynamic_matrix_t>>;
  /** Array of state_dynamic matrix trajectory type. */
  using state_dynamic_matrix_array2_t = std::vector<state_dynamic_matrix_array_t, Eigen::aligned_allocator<state_dynamic_matrix_array_t>>;

  /** Dynamic-size by n_u matrix type. */
  using dynamic_input_matrix_t = Eigen::Matrix<scalar_t, Eigen::Dynamic, INPUT_DIM>;
  /** Dynamic-input matrix trajectory type. */
  using dynamic_input_matrix_array_t = std::vector<dynamic_input_matrix_t, Eigen::aligned_allocator<dynamic_input_matrix_t>>;
  /** Array of dynamic-input matrix trajectory type. */
  using dynamic_input_matrix_array2_t = std::vector<dynamic_input_matrix_array_t, Eigen::aligned_allocator<dynamic_input_matrix_array_t>>;

  /** n_u by dynamic-size matrix type. */
  using input_dynamic_matrix_t = Eigen::Matrix<scalar_t, Eigen::Dynamic, INPUT_DIM>;
  /** input_dynamic matrix trajectory type. */
  using input_dynamic_matrix_array_t = std::vector<input_dynamic_matrix_t, Eigen::aligned_allocator<input_dynamic_matrix_t>>;
  /** Array of input_dynamic matrix trajectory type. */
  using input_dynamic_matrix_array2_t = std::vector<input_dynamic_matrix_array_t, Eigen::aligned_allocator<input_dynamic_matrix_array_t>>;

  /** Fixed-size vector type with size \f$ n_x^2 \f$ . */
  using state_matrix_vectorized_t = Eigen::Matrix<scalar_t, STATE_DIM * STATE_DIM, 1>;

  /** Fixed-size vector of state-input constraints (i.e. type-1 constraint) with size \f$ {n_c}_1 \f$ . */
  using constraint1_vector_t = Eigen::Matrix<scalar_t, MAX_CONSTRAINT1_DIM_, 1>;
  /** Type-1 constraint (state-input constraint) vector trajectory type. */
  using constraint1_vector_array_t = std::vector<constraint1_vector_t, Eigen::aligned_allocator<constraint1_vector_t>>;
  /** Array of type-1 constraint (state-input constraint) vector trajectory type. */
  using constraint1_vector_array2_t = std::vector<constraint1_vector_array_t, Eigen::aligned_allocator<constraint1_vector_array_t>>;

  /** Fixed-size Matrix of state-input constraints (i.e. type-1 constraint) with size \f$ {n_c}_1 * {n_c}_1 \f$ . */
  using constraint1_matrix_t = Eigen::Matrix<scalar_t, MAX_CONSTRAINT1_DIM_, 1>;
  /** Type-1 constraint (state-input constraint) matrix trajectory type. */
  using constraint1_matrix_array_t = std::vector<constraint1_matrix_t, Eigen::aligned_allocator<constraint1_matrix_t>>;
  /** Array of type-1 constraint (state-input constraint) matrix trajectory type. */
  using constraint1_matrix_array2_t = std::vector<constraint1_matrix_array_t, Eigen::aligned_allocator<constraint1_matrix_array_t>>;

  /** Fixed-size type-1 constraint by state matrix type with size \f$ {n_c}_1 * n_x \f$ . */
  using constraint1_state_matrix_t = Eigen::Matrix<scalar_t, MAX_CONSTRAINT1_DIM_, STATE_DIM>;
  /** Constraint1_state matrix trajectory type. */
  using constraint1_state_matrix_array_t = std::vector<constraint1_state_matrix_t, Eigen::aligned_allocator<constraint1_state_matrix_t>>;
  /** Array of constraint1_state matrix trajectory type. */
  using constraint1_state_matrix_array2_t =
      std::vector<constraint1_state_matrix_array_t, Eigen::aligned_allocator<constraint1_state_matrix_array_t>>;

  /** Fixed-size type-1 constraint by control input matrix type with size \f$ {n_c}_1 * n_u \f$ . */
  using constraint1_input_matrix_t = Eigen::Matrix<scalar_t, MAX_CONSTRAINT1_DIM_, INPUT_DIM>;
  /** constraint1_input matrix trajectory type. */
  using constraint1_input_matrix_array_t = std::vector<constraint1_input_matrix_t, Eigen::aligned_allocator<constraint1_input_matrix_t>>;
  /** Array of constraint1_input matrix trajectory type. */
  using constraint1_input_matrix_array2_t =
      std::vector<constraint1_input_matrix_array_t, Eigen::aligned_allocator<constraint1_input_matrix_array_t>>;

  /** Fixed-size control input by type-1 constraint type with size \f$ n_u * {n_c}_1 \f$ . */
  using input_constraint1_matrix_t = Eigen::Matrix<scalar_t, INPUT_DIM, MAX_CONSTRAINT1_DIM_>;
  /** Input-constraint1 matrix trajectory type. */
  using input_constraint1_matrix_array_t = std::vector<input_constraint1_matrix_t, Eigen::aligned_allocator<input_constraint1_matrix_t>>;
  /** Array of Input-constraint1 matrix trajectory type. */
  using input_constraint1_matrix_array2_t =
      std::vector<input_constraint1_matrix_array_t, Eigen::aligned_allocator<input_constraint1_matrix_array_t>>;

  /** Fixed-size vector of state-only constraints (i.e. type-2 constraint) with size \f$ {n_c}_2 \f$ . */
  using constraint2_vector_t = Eigen::Matrix<scalar_t, MAX_CONSTRAINT2_DIM_, 1>;
  /** Type-2 constraint (state-only constraint) vector trajectory type. */
  using constraint2_vector_array_t = std::vector<constraint2_vector_t, Eigen::aligned_allocator<constraint2_vector_t>>;
  /** Array of type-2 constraint (state-only constraint) vector trajectory type. */
  using constraint2_vector_array2_t = std::vector<constraint2_vector_array_t, Eigen::aligned_allocator<constraint2_vector_array_t>>;

  /** Fixed-size type-2 constraint by state matrix type with size \f$ {n_c}_2 * n_x \f$ . */
  using constraint2_state_matrix_t = Eigen::Matrix<scalar_t, MAX_CONSTRAINT2_DIM_, STATE_DIM>;
  /** Constraint2_state matrix trajectory type. */
  using constraint2_state_matrix_array_t = std::vector<constraint2_state_matrix_t, Eigen::aligned_allocator<constraint2_state_matrix_t>>;
  /** Array of constraint2_state matrix trajectory type. */
  using constraint2_state_matrix_array2_t =
      std::vector<constraint2_state_matrix_array_t, Eigen::aligned_allocator<constraint2_state_matrix_array_t>>;
};

}  // namespace ocs2
