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

#include <ocs2_core/dynamics/ControlledSystemBase.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ControlledSystemBase::ControlledSystemBase(const PreComputation& preComputation) : preCompPtr_(preComputation.clone()) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ControlledSystemBase::ControlledSystemBase(const ControlledSystemBase& other) : OdeBase(other) {
  assert(other.preCompPtr_ != nullptr);
  preCompPtr_.reset(other.preCompPtr_->clone());
  setController(other.controllerPtr());
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t ControlledSystemBase::computeFlowMap(scalar_t t, const vector_t& x) {
  assert(controllerPtr_ != nullptr);
  const vector_t u = controllerPtr_->computeInput(t, x);
  return computeFlowMap(t, x, u);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t ControlledSystemBase::computeFlowMap(scalar_t t, const vector_t& x, const vector_t& u) {
  assert(preCompPtr_ != nullptr);
  preCompPtr_->request(Request::Dynamics, t, x, u);
  return computeFlowMap(t, x, u, *preCompPtr_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t ControlledSystemBase::computeJumpMap(scalar_t t, const vector_t& x) {
  assert(preCompPtr_ != nullptr);
  preCompPtr_->requestPreJump(Request::Dynamics, t, x);
  return computeJumpMap(t, x, *preCompPtr_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t ControlledSystemBase::computeJumpMap(scalar_t t, const vector_t& x, const PreComputation&) {
  // default implementation
  return OdeBase::computeJumpMap(t, x);
}

}  // namespace ocs2
