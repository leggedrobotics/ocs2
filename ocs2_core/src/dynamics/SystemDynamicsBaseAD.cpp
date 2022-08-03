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

#include <ocs2_core/dynamics/SystemDynamicsBaseAD.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SystemDynamicsBaseAD::SystemDynamicsBaseAD() : SystemDynamicsBase() {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SystemDynamicsBaseAD::SystemDynamicsBaseAD(const SystemDynamicsBaseAD& rhs)
    : SystemDynamicsBase(rhs),
      flowMapADInterfacePtr_(new CppAdInterface(*rhs.flowMapADInterfacePtr_)),
      jumpMapADInterfacePtr_(new CppAdInterface(*rhs.jumpMapADInterfacePtr_)),
      guardSurfacesADInterfacePtr_(new CppAdInterface(*rhs.guardSurfacesADInterfacePtr_)),
      tapedTimeStateInput_(rhs.tapedTimeStateInput_.size()),
      tapedTimeState_(rhs.tapedTimeState_.size()),
      flowJacobian_(rhs.flowJacobian_.rows(), rhs.flowJacobian_.cols()),
      jumpJacobian_(rhs.jumpJacobian_.rows(), rhs.jumpJacobian_.cols()),
      guardJacobian_(rhs.guardJacobian_.rows(), rhs.guardJacobian_.cols()) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SystemDynamicsBaseAD::initialize(size_t stateDim, size_t inputDim, const std::string& modelName, const std::string& modelFolder,
                                      bool recompileLibraries, bool verbose) {
  tapedTimeStateInput_.resize(1 + stateDim + inputDim);
  tapedTimeState_.resize(1 + stateDim);

  auto flowMap = [this, stateDim, inputDim](const ad_vector_t& x, const ad_vector_t& p, ad_vector_t& y) {
    const ad_scalar_t time = x(0);
    const ad_vector_t state = x.segment(1, stateDim);
    const ad_vector_t input = x.tail(inputDim);
    y = this->systemFlowMap(time, state, input, p);
  };
  flowMapADInterfacePtr_.reset(
      new CppAdInterface(flowMap, 1 + stateDim + inputDim, getNumFlowMapParameters(), modelName + "_flow_map", modelFolder));

  auto jumpMap = [this, stateDim](const ad_vector_t& x, const ad_vector_t& p, ad_vector_t& y) {
    const ad_scalar_t time = x(0);
    const ad_vector_t state = x.tail(stateDim);
    y = this->systemJumpMap(time, state, p);
  };
  jumpMapADInterfacePtr_.reset(new CppAdInterface(jumpMap, 1 + stateDim, getNumJumpMapParameters(), modelName + "_jump_map", modelFolder));

  auto guardSurfaces = [this, stateDim](const ad_vector_t& x, const ad_vector_t& p, ad_vector_t& y) {
    const ad_scalar_t time = x(0);
    const ad_vector_t state = x.tail(stateDim);
    y = this->systemGuardSurfaces(time, state, p);
  };
  guardSurfacesADInterfacePtr_.reset(
      new CppAdInterface(guardSurfaces, 1 + stateDim, getNumGuardSurfacesParameters(), modelName + "_guard_surfaces", modelFolder));

  if (recompileLibraries) {
    flowMapADInterfacePtr_->createModels(CppAdInterface::ApproximationOrder::First, verbose);
    jumpMapADInterfacePtr_->createModels(CppAdInterface::ApproximationOrder::First, verbose);
    guardSurfacesADInterfacePtr_->createModels(CppAdInterface::ApproximationOrder::First, verbose);
  } else {
    flowMapADInterfacePtr_->loadModelsIfAvailable(CppAdInterface::ApproximationOrder::First, verbose);
    jumpMapADInterfacePtr_->loadModelsIfAvailable(CppAdInterface::ApproximationOrder::First, verbose);
    guardSurfacesADInterfacePtr_->loadModelsIfAvailable(CppAdInterface::ApproximationOrder::First, verbose);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t SystemDynamicsBaseAD::computeFlowMap(scalar_t t, const vector_t& x, const vector_t& u, const PreComputation& preComputation) {
  tapedTimeStateInput_ << t, x, u;
  const vector_t parameters = getFlowMapParameters(t, preComputation);
  return flowMapADInterfacePtr_->getFunctionValue(tapedTimeStateInput_, parameters);
}

/*******************q**********************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t SystemDynamicsBaseAD::computeJumpMap(scalar_t t, const vector_t& x, const PreComputation& preComputation) {
  tapedTimeState_ << t, x;
  const vector_t parameters = getJumpMapParameters(t, preComputation);
  return jumpMapADInterfacePtr_->getFunctionValue(tapedTimeState_, parameters);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t SystemDynamicsBaseAD::computeGuardSurfaces(scalar_t t, const vector_t& x) {
  tapedTimeState_ << t, x;
  const vector_t parameters = getGuardSurfacesParameters(t);
  return guardSurfacesADInterfacePtr_->getFunctionValue(tapedTimeState_, parameters);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation SystemDynamicsBaseAD::linearApproximation(scalar_t t, const vector_t& x, const vector_t& u,
                                                                            const PreComputation& preComputation) {
  tapedTimeStateInput_ << t, x, u;
  const vector_t parameters = getFlowMapParameters(t, preComputation);
  flowJacobian_ = flowMapADInterfacePtr_->getJacobian(tapedTimeStateInput_, parameters);

  VectorFunctionLinearApproximation approximation;
  approximation.dfdx = flowJacobian_.middleCols(1, x.rows());
  approximation.dfdu = flowJacobian_.rightCols(u.rows());
  approximation.f = flowMapADInterfacePtr_->getFunctionValue(tapedTimeStateInput_, parameters);
  return approximation;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation SystemDynamicsBaseAD::jumpMapLinearApproximation(scalar_t t, const vector_t& x,
                                                                                   const PreComputation& preComputation) {
  tapedTimeState_ << t, x;
  const vector_t parameters = getJumpMapParameters(t, preComputation);
  jumpJacobian_ = jumpMapADInterfacePtr_->getJacobian(tapedTimeState_, parameters);

  VectorFunctionLinearApproximation approximation;
  approximation.dfdx = jumpJacobian_.rightCols(x.rows());
  approximation.dfdu.setZero(jumpJacobian_.rows(), 0);
  approximation.f = jumpMapADInterfacePtr_->getFunctionValue(tapedTimeState_, parameters);
  return approximation;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation SystemDynamicsBaseAD::guardSurfacesLinearApproximation(scalar_t t, const vector_t& x, const vector_t& u) {
  tapedTimeState_ << t, x;
  const vector_t parameters = getGuardSurfacesParameters(t);
  guardJacobian_ = guardSurfacesADInterfacePtr_->getJacobian(tapedTimeState_, parameters);

  VectorFunctionLinearApproximation approximation;
  approximation.dfdx = guardJacobian_.rightCols(x.rows());
  approximation.dfdu = matrix_t::Zero(guardJacobian_.rows(), u.rows());  // not provided
  approximation.f = guardSurfacesADInterfacePtr_->getFunctionValue(tapedTimeState_, parameters);
  return approximation;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t SystemDynamicsBaseAD::flowMapDerivativeTime(scalar_t t, const vector_t& x, const vector_t& u) {
  return flowJacobian_.leftCols(1);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t SystemDynamicsBaseAD::jumpMapDerivativeTime(scalar_t t, const vector_t& x, const vector_t& u) {
  return jumpJacobian_.leftCols(1);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t SystemDynamicsBaseAD::guardSurfacesDerivativeTime(scalar_t t, const vector_t& x, const vector_t& u) {
  return guardJacobian_.leftCols(1);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ad_vector_t SystemDynamicsBaseAD::systemJumpMap(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& parameters) const {
  return state;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ad_vector_t SystemDynamicsBaseAD::systemGuardSurfaces(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& parameters) const {
  return -ad_vector_t::Ones(1);
}

}  // namespace ocs2
