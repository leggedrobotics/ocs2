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
SystemDynamicsBaseAD::SystemDynamicsBaseAD(size_t stateDim, size_t inputDim)
    : SystemDynamicsBase(stateDim, inputDim),
      flowJacobian_(matrix_t::Zero(stateDim_, 1 + stateDim_ + inputDim_)),
      jumpJacobian_(matrix_t::Zero(stateDim_, 1 + stateDim_)),
      guardJacobian_(matrix_t::Zero(1, 1 + stateDim_)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SystemDynamicsBaseAD::SystemDynamicsBaseAD(const SystemDynamicsBaseAD& rhs)
    : SystemDynamicsBase(rhs),
      flowMapADInterfacePtr_(new CppAdInterface(*rhs.flowMapADInterfacePtr_)),
      jumpMapADInterfacePtr_(new CppAdInterface(*rhs.jumpMapADInterfacePtr_)),
      guardSurfacesADInterfacePtr_(new CppAdInterface(*rhs.guardSurfacesADInterfacePtr_)),
      flowJacobian_(matrix_t::Zero(stateDim_, 1 + stateDim_ + inputDim_)),
      jumpJacobian_(matrix_t::Zero(stateDim_, 1 + stateDim_)),
      guardJacobian_(matrix_t::Zero(1, 1 + stateDim_)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SystemDynamicsBaseAD::initialize(const std::string& modelName, const std::string& modelFolder, bool recompileLibraries, bool verbose) {
  setADInterfaces(modelName, modelFolder);
  if (recompileLibraries) {
    createModels(verbose);
  } else {
    loadModelsIfAvailable(verbose);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t SystemDynamicsBaseAD::computeFlowMap(scalar_t time, const vector_t& state, const vector_t& input) {
  vector_t tapedInput(1 + stateDim_ + inputDim_);
  tapedInput << time, state, input;

  return flowMapADInterfacePtr_->getFunctionValue(tapedInput);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t SystemDynamicsBaseAD::computeJumpMap(scalar_t time, const vector_t& state) {
  vector_t tapedInput(1 + stateDim_);
  tapedInput << time, state;

  return jumpMapADInterfacePtr_->getFunctionValue(tapedInput);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t SystemDynamicsBaseAD::computeGuardSurfaces(scalar_t time, const vector_t& state) {
  vector_t tapedInput(1 + stateDim_);
  tapedInput << time, state;

  return guardSurfacesADInterfacePtr_->getFunctionValue(tapedInput);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SystemDynamicsBaseAD::setCurrentStateAndControl(scalar_t time, const vector_t& state, const vector_t& input) {
  SystemDynamicsBase::setCurrentStateAndControl(time, state, input);

  vector_t tapedTimeStateInput(1 + stateDim_ + inputDim_);
  tapedTimeStateInput << time, state, input;

  vector_t tapedTimeState(1 + stateDim_);
  tapedTimeState << time, state;

  flowJacobian_ = flowMapADInterfacePtr_->getJacobian(tapedTimeStateInput);
  jumpJacobian_ = jumpMapADInterfacePtr_->getJacobian(tapedTimeState);
  guardJacobian_ = guardSurfacesADInterfacePtr_->getJacobian(tapedTimeState);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t SystemDynamicsBaseAD::getFlowMapDerivativeTime() {
  return flowJacobian_.leftCols(1);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_t SystemDynamicsBaseAD::getFlowMapDerivativeState() {
  return flowJacobian_.middleCols(1, stateDim_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_t SystemDynamicsBaseAD::getFlowMapDerivativeInput() {
  return flowJacobian_.rightCols(inputDim_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t SystemDynamicsBaseAD::getJumpMapDerivativeTime() {
  return jumpJacobian_.leftCols(1);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_t SystemDynamicsBaseAD::getJumpMapDerivativeState() {
  return jumpJacobian_.rightCols(stateDim_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t SystemDynamicsBaseAD::getGuardSurfacesDerivativeTime() {
  return guardJacobian_.leftCols(1);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_t SystemDynamicsBaseAD::getGuardSurfacesDerivativeState() {
  return guardJacobian_.rightCols(stateDim_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SystemDynamicsBaseAD::ad_vector_t SystemDynamicsBaseAD::systemJumpMap(ad_scalar_t time, const ad_vector_t& state) const {
  return state;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SystemDynamicsBaseAD::ad_vector_t SystemDynamicsBaseAD::systemGuardSurfaces(ad_scalar_t time, const ad_vector_t& state) const {
  return -ad_vector_t::Ones(1);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SystemDynamicsBaseAD::setADInterfaces(const std::string& modelName, const std::string& modelFolder) {
  auto tapedFlowMap = [this](const ad_vector_t& x, ad_vector_t& y) {
    auto time = x(0);
    auto state = x.segment(1, stateDim_);
    auto input = x.segment(1 + stateDim_, inputDim_);
    y = this->systemFlowMap(time, state, input);
  };
  flowMapADInterfacePtr_.reset(new CppAdInterface(tapedFlowMap, 1 + stateDim_ + inputDim_, modelName + "_flow_map", modelFolder));

  auto tapedJumpMap = [this](const ad_vector_t& x, ad_vector_t& y) {
    auto time = x(0);
    auto state = x.segment(1, stateDim_);
    y = this->systemJumpMap(time, state);
  };
  jumpMapADInterfacePtr_.reset(new CppAdInterface(tapedJumpMap, 1 + stateDim_, modelName + "_jump_map", modelFolder));

  auto tapedGuardSurfaces = [this](const ad_vector_t& x, ad_vector_t& y) {
    auto time = x(0);
    auto state = x.segment(1, stateDim_);
    y = this->systemGuardSurfaces(time, state);
  };
  guardSurfacesADInterfacePtr_.reset(new CppAdInterface(tapedGuardSurfaces, 1 + stateDim_, modelName + "_guard_surfaces", modelFolder));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SystemDynamicsBaseAD::createModels(bool verbose) {
  flowMapADInterfacePtr_->createModels(CppAdInterface::ApproximationOrder::First, verbose);
  jumpMapADInterfacePtr_->createModels(CppAdInterface::ApproximationOrder::First, verbose);
  guardSurfacesADInterfacePtr_->createModels(CppAdInterface::ApproximationOrder::First, verbose);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SystemDynamicsBaseAD::loadModelsIfAvailable(bool verbose) {
  flowMapADInterfacePtr_->loadModelsIfAvailable(CppAdInterface::ApproximationOrder::First, verbose);
  jumpMapADInterfacePtr_->loadModelsIfAvailable(CppAdInterface::ApproximationOrder::First, verbose);
  guardSurfacesADInterfacePtr_->loadModelsIfAvailable(CppAdInterface::ApproximationOrder::First, verbose);
}

}  // namespace ocs2
