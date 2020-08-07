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

#include <cstddef>

#include <ocs2_core/Types.h>

namespace mobile_manipulator {

constexpr size_t STATE_DIM = 2;  // TODO: define dimensions
constexpr size_t INPUT_DIM = 1;

/* Import ocs2 types into the mobile_manipulator namespace */
using ocs2::matrix_array_t;
using ocs2::matrix_t;
using ocs2::scalar_array_t;
using ocs2::scalar_t;
using ocs2::size_array_t;
using ocs2::vector_array_t;
using ocs2::vector_t;

using ocs2::ScalarFunctionQuadraticApproximation;
using ocs2::VectorFunctionLinearApproximation;
using ocs2::VectorFunctionQuadraticApproximation;

/* Define fixed-size types */
using state_vector_t = Eigen::Matrix<scalar_t, STATE_DIM, 1>;
using input_vector_t = Eigen::Matrix<scalar_t, INPUT_DIM, 1>;
using state_matrix_t = Eigen::Matrix<scalar_t, STATE_DIM, STATE_DIM>;
using input_matrix_t = Eigen::Matrix<scalar_t, INPUT_DIM, INPUT_DIM>;
using input_state_matrix_t = Eigen::Matrix<scalar_t, INPUT_DIM, STATE_DIM>;

}  // namespace mobile_manipulator
