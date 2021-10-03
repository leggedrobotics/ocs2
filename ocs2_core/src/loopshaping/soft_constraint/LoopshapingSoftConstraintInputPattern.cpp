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

#include <ocs2_core/loopshaping/LoopshapingPreComputation.h>
#include <ocs2_core/loopshaping/soft_constraint/LoopshapingSoftConstraintInputPattern.h>

namespace ocs2 {

ScalarFunctionQuadraticApproximation LoopshapingSoftConstraintInputPattern::getQuadraticApproximation(
    scalar_t t, const vector_t& x, const vector_t& u, const TargetTrajectories& targetTrajectories, const PreComputation& preComp) const {
  if (this->empty()) {
    return ScalarFunctionQuadraticApproximation::Zero(x.rows(), u.rows());
  }

  const auto& s_filter = loopshapingDefinition_->getInputFilter();
  const auto& preCompLS = cast<LoopshapingPreComputation>(preComp);
  const auto& x_system = preCompLS.getSystemState();
  const auto& u_system = preCompLS.getSystemInput();

  const auto L_system =
      StateInputCostCollection::getQuadraticApproximation(t, x_system, u_system, targetTrajectories, preCompLS.getSystemPreComputation());

  ScalarFunctionQuadraticApproximation L = ScalarFunctionQuadraticApproximation::Zero(x.rows(), u.rows());
  L.f = L_system.f;
  L.dfdx.head(x_system.rows()) = L_system.dfdx;
  L.dfdxx.topLeftCorner(x_system.rows(), x_system.rows()) = L_system.dfdxx;
  L.dfdu.head(u_system.rows()) = L_system.dfdu;
  L.dfduu.topLeftCorner(u_system.rows(), u_system.rows()) = L_system.dfduu;
  L.dfdux.topLeftCorner(u_system.rows(), x_system.rows()) = L_system.dfdux;

  return L;
}

}  // namespace ocs2
