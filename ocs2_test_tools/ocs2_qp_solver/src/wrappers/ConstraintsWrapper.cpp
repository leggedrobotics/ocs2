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

#include "ocs2_qp_solver/wrappers/ConstraintsWrapper.h"

namespace ocs2 {
namespace qp_solver {

dynamic_vector_t ConstraintsWrapper::getConstraint(scalar_t t, const dynamic_vector_t& x, const dynamic_vector_t& u) {
  p_->setCurrentStateAndControl(t, x, u);
  return p_->getConstraints();
}

VectorFunctionLinearApproximation ConstraintsWrapper::getLinearApproximation(scalar_t t, const dynamic_vector_t& x,
                                                                             const dynamic_vector_t& u) {
  VectorFunctionLinearApproximation linearConstraints;
  p_->setCurrentStateAndControl(t, x, u);
  linearConstraints.dfdx = p_->getConstraintsDerivativeState();
  linearConstraints.dfdu = p_->getConstraintsDerivativeInput();
  linearConstraints.f = p_->getConstraints();
  return linearConstraints;
}

dynamic_vector_t ConstraintsWrapper::getTerminalConstraint(scalar_t t, const dynamic_vector_t& x) {
  p_->setCurrentStateAndControl(t, x);
  return p_->getTerminalConstraints();
}

VectorFunctionLinearApproximation ConstraintsWrapper::getTerminalLinearApproximation(scalar_t t, const dynamic_vector_t& x) {
  VectorFunctionLinearApproximation linearConstraints;
  p_->setCurrentStateAndControl(t, x);
  linearConstraints.dfdx = p_->getTerminalConstraintsDerivativeState();
  linearConstraints.f = p_->getTerminalConstraints();
  return linearConstraints;
}

}  // namespace qp_solver
}  // namespace ocs2
