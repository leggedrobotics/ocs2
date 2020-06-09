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
ControlledSystemBase::ControlledSystemBase() : controllerPtr_(nullptr) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ControlledSystemBase::ControlledSystemBase(const ControlledSystemBase& rhs) {
  setController(rhs.controllerPtr());
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
void ControlledSystemBase::computeFlowMap(const scalar_t& t, const vector_t& x, vector_t& dxdt) {
  vector_t u = controllerPtr_->computeInput(t, x);
  ModelDataBase& modelData = this->modelDataEmplaceBack();
  modelData.time_ = t;
  modelData.stateDim_ = x.rows();
  modelData.inputDim_ = u.rows();
  computeFlowMap(t, x, u, dxdt);
  modelData.dynamics_ = dxdt;
}

}  // namespace ocs2
