/******************************************************************************
Copyright (c) 2020, Ruben Grandia. All rights reserved.

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

#include <ocs2_core/loopshaping/constraint/LoopshapingConstraintOutputPattern.h>

namespace ocs2 {

VectorFunctionQuadraticApproximation LoopshapingConstraintOutputPattern::inequalityConstraintQuadraticApproximation(scalar_t t,
                                                                                                                    const vector_t& x,
                                                                                                                    const vector_t& u) {
  const auto& s_filter = loopshapingDefinition_->getInputFilter();
  const vector_t x_system = loopshapingDefinition_->getSystemState(x);
  const vector_t u_system = loopshapingDefinition_->getSystemInput(x, u);
  const size_t FILTER_STATE_DIM = s_filter.getNumStates();
  const auto h_system = systemConstraint_->inequalityConstraintQuadraticApproximation(t, x_system, u_system);

  VectorFunctionQuadraticApproximation h;
  h.f = h_system.f;

  h.dfdx.resize(h.f.rows(), x.rows());
  h.dfdx.leftCols(x_system.rows()) = h_system.dfdx;
  h.dfdx.rightCols(FILTER_STATE_DIM).setZero();

  h.dfdu = h_system.dfdu;

  h.dfdxx.resize(h.f.rows());
  h.dfduu.resize(h.f.rows());
  h.dfdux.resize(h.f.rows());
  for (size_t i = 0; i < h.f.rows(); i++) {
    h.dfdxx[i].setZero(x.rows(), x.rows());
    h.dfdxx[i].topLeftCorner(x_system.rows(), x_system.rows()) = h_system.dfdxx[i];

    h.dfduu[i] = h_system.dfduu[i];

    h.dfdux[i].resize(u.rows(), x.rows());
    h.dfdux[i].leftCols(x_system.rows()) = h_system.dfdux[i];
    h.dfdux[i].rightCols(FILTER_STATE_DIM).setZero();
  }

  return h;
}

vector_t LoopshapingConstraintOutputPattern::stateInputEqualityConstraint(scalar_t t, const vector_t& x, const vector_t& u) {
  const vector_t x_system = loopshapingDefinition_->getSystemState(x);
  const vector_t u_system = loopshapingDefinition_->getSystemInput(x, u);
  return systemConstraint_->stateInputEqualityConstraint(t, x_system, u_system);
}

VectorFunctionLinearApproximation LoopshapingConstraintOutputPattern::stateInputEqualityConstraintLinearApproximation(scalar_t t,
                                                                                                                      const vector_t& x,
                                                                                                                      const vector_t& u) {
  const vector_t x_system = loopshapingDefinition_->getSystemState(x);
  const vector_t u_system = loopshapingDefinition_->getSystemInput(x, u);
  const auto g_system = systemConstraint_->stateInputEqualityConstraintLinearApproximation(t, x_system, u_system);

  VectorFunctionLinearApproximation g;
  g.f = g_system.f;

  g.dfdx.resize(g.f.rows(), x.rows());
  g.dfdx.leftCols(x_system.rows()) = g_system.dfdx;
  g.dfdx.rightCols(x.rows() - x_system.rows()).setZero();

  g.dfdu.resize(g.f.rows(), u.rows());
  g.dfdu.leftCols(u_system.rows()).noalias() = g_system.dfdu;
  g.dfdu.rightCols(u.rows() - u_system.rows()).setZero();

  return g;
}

}  // namespace ocs2
