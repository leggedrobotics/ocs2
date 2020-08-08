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

#include <ocs2_core/constraint/ConstraintBase.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ConstraintBase* ConstraintBase::clone() const {
  return new ConstraintBase(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t ConstraintBase::stateInputEqualityConstraint(scalar_t t, const vector_t& x, const vector_t& u) {
  return vector_t(0);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t ConstraintBase::stateEqualityConstraint(scalar_t t, const vector_t& x) {
  return vector_t(0);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t ConstraintBase::inequalityConstraint(scalar_t t, const vector_t& x, const vector_t& u) {
  return vector_t(0);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t ConstraintBase::finalStateEqualityConstraint(scalar_t t, const vector_t& x) {
  return vector_t(0);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation ConstraintBase::stateInputEqualityConstraintLinearApproximation(scalar_t t, const vector_t& x,
                                                                                                  const vector_t& u) {
  VectorFunctionLinearApproximation g;
  g.f.setZero(0);
  g.dfdx.setZero(0, x.rows());
  g.dfdu.setZero(0, u.rows());
  return g;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation ConstraintBase::stateEqualityConstraintLinearApproximation(scalar_t t, const vector_t& x) {
  VectorFunctionLinearApproximation g;
  g.f.setZero(0);
  g.dfdx.setZero(0, x.rows());
  return g;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionQuadraticApproximation ConstraintBase::inequalityConstraintQuadraticApproximation(scalar_t t, const vector_t& x,
                                                                                                const vector_t& u) {
  VectorFunctionQuadraticApproximation h;
  h.f.setZero(0);
  h.dfdx.setZero(0, x.rows());
  h.dfdu.setZero(0, u.rows());
  h.dfdxx.clear();
  h.dfdux.clear();
  h.dfduu.clear();
  return h;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation ConstraintBase::finalStateEqualityConstraintLinearApproximation(scalar_t t, const vector_t& x) {
  VectorFunctionLinearApproximation gf;
  gf.f.setZero(0);
  gf.dfdx.setZero(0, x.rows());
  return gf;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_array_t ConstraintBase::stateInputEqualityConstraintDerivativesEventTimes(scalar_t t, const vector_t& x, const vector_t& u) {
  return vector_array_t(0);
}

}  // namespace ocs2
