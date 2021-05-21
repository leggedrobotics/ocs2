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

#include <ocs2_core/constraint/LinearStateInputConstraint.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LinearStateInputConstraint::LinearStateInputConstraint(vector_t e, matrix_t C, matrix_t D)
    : StateInputConstraint(ConstraintOrder::Linear), e_(std::move(e)), C_(std::move(C)), D_(std::move(D)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LinearStateInputConstraint* LinearStateInputConstraint::clone() const {
  return new LinearStateInputConstraint(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
size_t LinearStateInputConstraint::getNumConstraints(scalar_t time) const {
  return e_.rows();
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t LinearStateInputConstraint::getValue(scalar_t t, const vector_t& x, const vector_t& u, const PreComputation&) const {
  vector_t g = e_;
  g.noalias() += C_ * x;
  g.noalias() += D_ * u;
  return g;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation LinearStateInputConstraint::getLinearApproximation(scalar_t t, const vector_t& x, const vector_t& u,
                                                                                     const PreComputation&) const {
  VectorFunctionLinearApproximation g;
  g.f = e_;
  g.f.noalias() += C_ * x;
  g.f.noalias() += D_ * u;
  g.dfdx = C_;
  g.dfdu = D_;
  return g;
}

}  // namespace ocs2
