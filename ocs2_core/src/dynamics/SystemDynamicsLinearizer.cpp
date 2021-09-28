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
SystemDynamicsLinearizer::SystemDynamicsLinearizer(std::unique_ptr<ControlledSystemBase> nonlinearSystemPtr,
                                                   bool doubleSidedDerivative /*= true*/, bool isSecondOrderSystem /*= false*/,
                                                   scalar_t eps /*= Eigen::NumTraits<scalar_t>::epsilon()*/)
    : SystemDynamicsBase(nonlinearSystemPtr->getPreComputation()),
      controlledSystemPtr_(std::move(nonlinearSystemPtr)),
      doubleSidedDerivative_(doubleSidedDerivative),
      isSecondOrderSystem_(isSecondOrderSystem),
      eps_(eps) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SystemDynamicsLinearizer::SystemDynamicsLinearizer(const SystemDynamicsLinearizer& other)
    : SystemDynamicsBase(other),
      controlledSystemPtr_(other.controlledSystemPtr_->clone()),
      doubleSidedDerivative_(other.doubleSidedDerivative_),
      isSecondOrderSystem_(other.isSecondOrderSystem_),
      eps_(other.eps_) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SystemDynamicsLinearizer* SystemDynamicsLinearizer::clone() const {
  return new SystemDynamicsLinearizer(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t SystemDynamicsLinearizer::computeFlowMap(scalar_t time, const vector_t& state, const vector_t& input,
                                                  const PreComputation& preComp) {
  return controlledSystemPtr_->computeFlowMap(time, state, input, preComp);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation SystemDynamicsLinearizer::linearApproximation(scalar_t t, const vector_t& x, const vector_t& u,
                                                                                const PreComputation& preComp) {
  VectorFunctionLinearApproximation linearDynamics;
  linearDynamics.f = controlledSystemPtr_->computeFlowMap(t, x, u, preComp);
  linearDynamics.dfdx = finiteDifferenceDerivativeState(*controlledSystemPtr_, t, x, u, eps_, doubleSidedDerivative_, isSecondOrderSystem_);
  linearDynamics.dfdu = finiteDifferenceDerivativeInput(*controlledSystemPtr_, t, x, u, eps_, doubleSidedDerivative_, isSecondOrderSystem_);
  return linearDynamics;
}

}  // namespace ocs2
