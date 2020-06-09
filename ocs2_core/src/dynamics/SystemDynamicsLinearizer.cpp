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

#include <ocs2_core/dynamics/SystemDynamicsLinearizer.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SystemDynamicsLinearizer::SystemDynamicsLinearizer(std::shared_ptr<ControlledSystemBase> nonlinearSystemPtr,
                                                   bool doubleSidedDerivative /*= true*/, bool isSecondOrderSystem /*= false*/)
    : controlledSystemPtr_(std::move(nonlinearSystemPtr)),
      doubleSidedDerivative_(doubleSidedDerivative),
      isSecondOrderSystem_(isSecondOrderSystem) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SystemDynamicsLinearizer::SystemDynamicsLinearizer(const SystemDynamicsLinearizer& other)
    : controlledSystemPtr_(other.controlledSystemPtr_->clone()),
      doubleSidedDerivative_(other.doubleSidedDerivative_),
      isSecondOrderSystem_(other.isSecondOrderSystem_) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SystemDynamicsLinearizer& SystemDynamicsLinearizer::operator=(const SystemDynamicsLinearizer& other) {
  controlledSystemPtr_.reset(other.controlledSystemPtr_->clone());
  doubleSidedDerivative_ = other.doubleSidedDerivative_;
  isSecondOrderSystem_ = other.isSecondOrderSystem_;
  return *this;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_t SystemDynamicsLinearizer::getFlowMapDerivativeState() {
  return finiteDifferenceDerivativeState(*controlledSystemPtr_, this->t_, this->x_, this->u_, Eigen::NumTraits<scalar_t>::epsilon(),
                                         doubleSidedDerivative_, isSecondOrderSystem_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_t SystemDynamicsLinearizer::getFlowMapDerivativeInput() {
  return finiteDifferenceDerivativeInput(*controlledSystemPtr_, this->t_, this->x_, this->u_, Eigen::NumTraits<scalar_t>::epsilon(),
                                         doubleSidedDerivative_, isSecondOrderSystem_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SystemDynamicsLinearizer* SystemDynamicsLinearizer::clone() const {
  return new SystemDynamicsLinearizer(*this);
}

}  // namespace ocs2
