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

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <vector>

#include <ocs2_core/integration/OdeBase.h>

namespace ocs2 {

/**
 * LTI system class for linear defined as \f$ dx/dt = G_m*x(t) + G_v \f$.
 * with \f$ x \in R^{DIM1 \times DIM2} \f$.
 *
 * @tparam DIM1: First dimension of the state space.
 * @tparam DIM2: Second dimension of the state space.
 * @tparam SCALAR: data type
 */
template <int DIM1, int DIM2 = 1, typename SCALAR = double>
class LTI_Equations : public OdeBase<DIM1 * DIM2> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  enum { LTI_DIM_ = DIM1 * DIM2 };

  using state_t = Eigen::Matrix<SCALAR, DIM1, DIM2>;
  using vectorized_state_t = Eigen::Matrix<SCALAR, LTI_DIM_, 1>;
  using state_array_t = std::vector<state_t, Eigen::aligned_allocator<state_t>>;
  using vectorized_state_array_t = std::vector<vectorized_state_t, Eigen::aligned_allocator<vectorized_state_t>>;

  LTI_Equations() = default;

  ~LTI_Equations() = default;

  /**
   * Transcribes the stacked vector into its matrix form.
   *
   * @param [in] vector: a single vector constructed by concatenating columns of matrix.
   * @param [out] matrix: the original matrix
   */
  void static convert2Matrix(const Eigen::Matrix<SCALAR, LTI_DIM_, 1>& vector, Eigen::Matrix<SCALAR, DIM1, DIM2>& matrix) {
    matrix = Eigen::Map<const Eigen::Matrix<SCALAR, DIM1, DIM2>>(vector.data());
  }

  /**
   * Transcribes the state matrix type to a stacked vector.
   *
   * @param [in] matrix: the original matrix
   * @param [out] vector: a single vector constructed by concatenating columns of matrix.
   */
  void static convert2Vector(const Eigen::Matrix<SCALAR, DIM1, DIM2>& matrix, Eigen::Matrix<SCALAR, LTI_DIM_, 1>& vector) {
    vector = Eigen::Map<const Eigen::Matrix<SCALAR, LTI_DIM_, 1>>(matrix.data());
  }

  /**
   * Set the LTI system coefficients, \f$ dx/dt = G_m*x(t) + G_v \f$.
   * @param [in] Gm: G_m matrix.
   * @param [in] Gv: G_v vector.
   */
  void setData(const Eigen::Matrix<SCALAR, DIM1, DIM1>* GmPtr, const Eigen::Matrix<SCALAR, DIM1, DIM2>* GvPtr) {
    GmPtr_ = GmPtr;
    GvPtr_ = GvPtr;
  }

  /**
   * Computes the LTI system dynamics.
   *
   * @param [in] t: Current time.
   * @param [in] x: Current state.
   * @param [out] dxdt: Current state time derivative
   */
  void computeFlowMap(const SCALAR& t, const vectorized_state_t& x, vectorized_state_t& dxdt) override {
    Eigen::Map<Eigen::Matrix<SCALAR, DIM1, DIM2>> dxdt_Matrix(dxdt.data(), DIM1, DIM2);

    dxdt_Matrix = (*GmPtr_) * Eigen::Map<const Eigen::Matrix<SCALAR, DIM1, DIM2>>(x.data(), DIM1, DIM2) + (*GvPtr_);
  }

 private:
  // members required in computeFlowMap
  const Eigen::Matrix<SCALAR, DIM1, DIM1>* GmPtr_;
  const Eigen::Matrix<SCALAR, DIM1, DIM2>* GvPtr_;
};

}  // namespace ocs2
