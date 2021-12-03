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

#include <ocs2_core/Types.h>
#include <ocs2_core/dynamics/TransferFunctionBase.h>
#include <Eigen/Dense>

namespace ocs2 {

class Filter {
 public:
  using diag_matrix_t = Eigen::DiagonalMatrix<ocs2::scalar_t, Eigen::Dynamic>;

  Filter();

  Filter(matrix_t A, matrix_t B, matrix_t C, matrix_t D);

  size_t getNumStates() const { return numStates_; };
  size_t getNumInputs() const { return numInputs_; };
  size_t getNumOutputs() const { return numOutputs_; };

  const matrix_t& getA() const { return A_; }
  const matrix_t& getB() const { return B_; }
  const matrix_t& getC() const { return C_; }
  const matrix_t& getD() const { return D_; }

  /// Get the diagonal of the filter matrices
  const diag_matrix_t& getAdiag() const { return a_; }
  const diag_matrix_t& getBdiag() const { return b_; }
  const diag_matrix_t& getCdiag() const { return c_; }
  const diag_matrix_t& getDdiag() const { return d_; }

  /// Get the equivalent element-wise scaling for pre- and post multiplying with diagonal matrices.
  const matrix_t& getScalingCdiagCdiag() const { return diagCC_; }
  const matrix_t& getScalingDdiagCdiag() const { return diagDC_; }
  const matrix_t& getScalingDdiagDdiag() const { return diagDD_; }

  void print() const;

  void findEquilibriumForOutput(const vector_t& y, vector_t& x, vector_t& u) const;
  void findEquilibriumForOutputGivenState(const vector_t& y, const vector_t& x, vector_t& u) const;
  void findEquilibriumForInput(const vector_t& u, vector_t& x, vector_t& y) const;

 private:
  void checkSize() const;

  matrix_t A_, B_, C_, D_;
  diag_matrix_t a_, b_, c_, d_;
  matrix_t diagCC_, diagDC_, diagDD_;
  size_t numStates_ = 0;
  size_t numInputs_ = 0;
  size_t numOutputs_ = 0;
  Eigen::ColPivHouseholderQR<matrix_t> Aqr_;
  Eigen::ColPivHouseholderQR<matrix_t> Dqr_;
  Eigen::ColPivHouseholderQR<matrix_t> ABCDqr_;
};

}  // namespace ocs2
