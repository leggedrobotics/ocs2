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

#include "ocs2_oc/multiple_shooting/LagrangianEvaluation.h"

namespace ocs2 {
namespace multiple_shooting {

ScalarFunctionQuadraticApproximation evaluateLagrangianIntermediateNode(const vector_t& lmd, const vector_t& lmd_next, const vector_t& nu,
                                                                        ScalarFunctionQuadraticApproximation&& cost,
                                                                        const VectorFunctionLinearApproximation& dynamics,
                                                                        const VectorFunctionLinearApproximation& stateInputEqConstraints) {
  ScalarFunctionQuadraticApproximation lagrangian = std::move(cost);
  lagrangian.dfdx.noalias() += dynamics.dfdx.transpose() * lmd_next;
  lagrangian.dfdx.noalias() -= lmd;
  lagrangian.dfdu.noalias() += dynamics.dfdu.transpose() * lmd_next;
  if (stateInputEqConstraints.f.size() > 0) {
    lagrangian.dfdx.noalias() += stateInputEqConstraints.dfdx.transpose() * nu;
    lagrangian.dfdu.noalias() += stateInputEqConstraints.dfdu.transpose() * nu;
  }
  return lagrangian;
}

ScalarFunctionQuadraticApproximation evaluateLagrangianTerminalNode(const vector_t& lmd, ScalarFunctionQuadraticApproximation&& cost) {
  ScalarFunctionQuadraticApproximation lagrangian = std::move(cost);
  lagrangian.dfdx.noalias() -= lmd;
  return lagrangian;
}

ScalarFunctionQuadraticApproximation evaluateLagrangianEventNode(const vector_t& lmd, const vector_t& lmd_next,
                                                                 ScalarFunctionQuadraticApproximation&& cost,
                                                                 const VectorFunctionLinearApproximation& dynamics) {
  ScalarFunctionQuadraticApproximation lagrangian = std::move(cost);
  lagrangian.dfdx.noalias() += dynamics.dfdx.transpose() * lmd_next;
  lagrangian.dfdx.noalias() -= lmd;
  return lagrangian;
}

}  // namespace multiple_shooting
}  // namespace ocs2
