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

#include <ocs2_core/dynamics/SystemDynamicsBase.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SystemDynamicsBase::SystemDynamicsBase(const PreComputation& preComputation) : ControlledSystemBase(preComputation) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SystemDynamicsBase::SystemDynamicsBase(const SystemDynamicsBase& other) : ControlledSystemBase(other) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation SystemDynamicsBase::linearApproximation(scalar_t t, const vector_t& x, const vector_t& u) {
  assert(preCompPtr_ != nullptr);
  preCompPtr_->request(Request::Dynamics + Request::Approximation, t, x, u);
  return linearApproximation(t, x, u, *preCompPtr_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation SystemDynamicsBase::jumpMapLinearApproximation(scalar_t t, const vector_t& x) {
  assert(preCompPtr_ != nullptr);
  preCompPtr_->requestPreJump(Request::Dynamics + Request::Approximation, t, x);
  return jumpMapLinearApproximation(t, x, *preCompPtr_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation SystemDynamicsBase::jumpMapLinearApproximation(scalar_t t, const vector_t& x,
                                                                                 const PreComputation& preComp) {
  VectorFunctionLinearApproximation approximation;
  approximation.dfdx.setIdentity(x.rows(), x.rows());
  approximation.dfdu.setZero(x.rows(), 0);
  assert(preCompPtr_ != nullptr);
  approximation.f = ControlledSystemBase::computeJumpMap(t, x, preComp);
  return approximation;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation SystemDynamicsBase::guardSurfacesLinearApproximation(scalar_t t, const vector_t& x, const vector_t& u) {
  VectorFunctionLinearApproximation approximation;
  approximation.dfdx.setZero(1, x.rows());
  approximation.dfdu.setZero(1, u.rows());
  approximation.f = this->computeGuardSurfaces(t, x);
  return approximation;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t SystemDynamicsBase::flowMapDerivativeTime(scalar_t t, const vector_t& x, const vector_t& u) {
  return vector_t::Zero(x.rows());
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t SystemDynamicsBase::jumpMapDerivativeTime(scalar_t t, const vector_t& x, const vector_t& u) {
  return vector_t::Zero(x.rows());
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t SystemDynamicsBase::guardSurfacesDerivativeTime(scalar_t t, const vector_t& x, const vector_t& u) {
  return vector_t::Zero(1);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_t SystemDynamicsBase::dynamicsCovariance(scalar_t t, const vector_t& x, const vector_t& u) {
  return matrix_t::Zero(0, 0);
}

}  // namespace ocs2
