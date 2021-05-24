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

#include <ocs2_core/dynamics/SystemDynamicsBase.h>

#include <ocs2_double_integrator_example/definitions.h>

namespace ocs2 {
namespace double_integrator {

class DoubleIntegratorDynamics final : public SystemDynamicsBase {
 public:
  /**
   * Constructor
   *
   * @param [in] A: \f$ A(t) \f$ matrix.
   * @param [in] B: \f$ B(t) \f$ matrix.
   */
  DoubleIntegratorDynamics(matrix_t A, matrix_t B) : A_(std::move(A)), B_(std::move(B)) {}

  /** Destructor */
  ~DoubleIntegratorDynamics() override = default;

  DoubleIntegratorDynamics* clone() const override { return new DoubleIntegratorDynamics(*this); }

  vector_t computeFlowMap(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation&) override {
    return A_ * state + B_ * input;
  }

  VectorFunctionLinearApproximation linearApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                        const PreComputation& preComp) override {
    VectorFunctionLinearApproximation dynamics;
    dynamics.f = computeFlowMap(time, state, input, preComp);
    dynamics.dfdx = A_;
    dynamics.dfdu = B_;
    return dynamics;
  }

 private:
  matrix_t A_;
  matrix_t B_;
};

}  // namespace double_integrator
}  // namespace ocs2
