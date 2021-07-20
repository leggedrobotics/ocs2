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
#include <ocs2_core/loopshaping/constraint/LoopshapingFilterConstraint.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t LoopshapingFilterConstraint::getValue(scalar_t t, const vector_t& x, const vector_t& u, const PreComputation& preComp) const {
  if (loopshapingDefinition_->getType() != LoopshapingType::inputpattern) {
    return vector_t(0);
  }

  const bool isDiagonal = loopshapingDefinition_->isDiagonal();
  const auto& s_filter = loopshapingDefinition_->getInputFilter();
  const auto& preCompLS = cast<LoopshapingPreComputation>(preComp);
  const auto& u_system = preCompLS.getSystemInput();
  const auto& x_filter = preCompLS.getFilterState();
  const auto& u_filter = preCompLS.getFilteredInput();

  if (isDiagonal) {
    return s_filter.getCdiag() * x_filter + s_filter.getDdiag() * u_filter - u_system;
  } else {
    return s_filter.getC() * x_filter + s_filter.getD() * u_filter - u_system;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation LoopshapingFilterConstraint::getLinearApproximation(scalar_t t, const vector_t& x, const vector_t& u,
                                                                                      const PreComputation& preComp) const {
  if (loopshapingDefinition_->getType() != LoopshapingType::inputpattern) {
    return VectorFunctionLinearApproximation(0, x.rows(), u.rows());
  }

  const bool isDiagonal = loopshapingDefinition_->isDiagonal();
  const auto& s_filter = loopshapingDefinition_->getInputFilter();
  const auto& preCompLS = cast<LoopshapingPreComputation>(preComp);
  const auto& x_system = preCompLS.getSystemState();
  const auto& u_system = preCompLS.getSystemInput();
  const auto& x_filter = preCompLS.getFilterState();
  const auto& u_filter = preCompLS.getFilteredInput();

  VectorFunctionLinearApproximation g(u_system.rows(), x.rows(), u.rows());
  if (isDiagonal) {
    g.f = s_filter.getCdiag() * x_filter + s_filter.getDdiag() * u_filter - u_system;
  } else {
    g.f = s_filter.getC() * x_filter + s_filter.getD() * u_filter - u_system;
  }

  g.dfdx.leftCols(x_system.rows()).setZero();
  g.dfdx.rightCols(x_filter.rows()) = s_filter.getC();

  g.dfdu.leftCols(u_system.rows()) = -matrix_t::Identity(u_system.rows(), u_system.rows());
  g.dfdu.rightCols(u_filter.rows()) = s_filter.getD();

  return g;
}

}  // namespace ocs2
