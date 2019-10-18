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

#include <ocs2_core/dynamics/DerivativesBase.h>

#include "ocs2_double_integrator_example/definitions.h"

namespace ocs2 {
namespace double_integrator {

class DoubleIntegratorDynamicsDerivatives final : public DerivativesBase<double_integrator::STATE_DIM_, double_integrator::INPUT_DIM_> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<DoubleIntegratorDynamicsDerivatives>;
  using ConstPtr = std::shared_ptr<const DoubleIntegratorDynamicsDerivatives>;

  using BASE = DerivativesBase<double_integrator::STATE_DIM_, double_integrator::INPUT_DIM_>;
  using scalar_t = typename BASE::scalar_t;
  using state_vector_t = typename BASE::state_vector_t;
  using state_matrix_t = typename BASE::state_matrix_t;
  using input_vector_t = typename BASE::input_vector_t;
  using state_input_matrix_t = typename BASE::state_input_matrix_t;

  /**
   * Constructor
   *
   * @param [in] A: \f$ A(t) \f$ matrix.
   * @param [in] B: \f$ B(t) \f$ matrix.
   */
  DoubleIntegratorDynamicsDerivatives(state_matrix_t A, state_input_matrix_t B) : A_(std::move(A)), B_(std::move(B)) {}

  /**
   * Destructor
   */
  ~DoubleIntegratorDynamicsDerivatives() override = default;

  DoubleIntegratorDynamicsDerivatives* clone() const override { return new DoubleIntegratorDynamicsDerivatives(*this); }

  void getFlowMapDerivativeState(state_matrix_t& A) override { A = A_; }

  void getFlowMapDerivativeInput(state_input_matrix_t& B) override { B = B_; }

 private:
  state_matrix_t A_;
  state_input_matrix_t B_;
};

}  // namespace double_integrator
}  // namespace ocs2
