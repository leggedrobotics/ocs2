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
ControlledSystemBase::ControlledSystemBase(PreComputation* preCompPtr) : preCompPtr_(preCompPtr) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ControlledSystemBase::ControlledSystemBase(const ControlledSystemBase& other) : OdeBase(other) {
  setController(other.controllerPtr());
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void ControlledSystemBase::reset() {
  controllerPtr_ = nullptr;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void ControlledSystemBase::setController(ControllerBase* controllerPtr) {
  controllerPtr_ = controllerPtr;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ControllerBase* ControlledSystemBase::controllerPtr() const {
  return controllerPtr_;
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
  if (preCompPtr_ != nullptr) {
    preCompPtr_->request(PreComputation::Request::Dynamics, t, x, u);
  }
  return computeFlowMap(t, x, u, preCompPtr_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t ControlledSystemBase::computeJumpMap(scalar_t t, const vector_t& x) {
  if (preCompPtr_ != nullptr) {
    preCompPtr_->requestPreJump(PreComputation::Request::Dynamics, t, x);
  }
  return computeJumpMap(t, x, /* u, */ preCompPtr_);
}

}  // namespace ocs2
