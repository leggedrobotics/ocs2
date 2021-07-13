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
SystemDynamicsBaseAD::SystemDynamicsBaseAD(size_t stateDim, size_t inputDim) : stateDim_(stateDim), inputDim_(inputDim) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SystemDynamicsBaseAD::SystemDynamicsBaseAD(const SystemDynamicsBaseAD& rhs)
    : SystemDynamicsBase(rhs),
      stateDim_(rhs.stateDim_),
      inputDim_(rhs.inputDim_),
      flowMapADInterfacePtr_(new CppAdInterface(*rhs.flowMapADInterfacePtr_)),
      jumpMapADInterfacePtr_(new CppAdInterface(*rhs.jumpMapADInterfacePtr_)),
      guardSurfacesADInterfacePtr_(new CppAdInterface(*rhs.guardSurfacesADInterfacePtr_)) {}

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
  vector_t tapedTimeStateInput(1 + state.rows() + input.rows());
  tapedTimeStateInput << time, state, input;
  vector_t parameters = getFlowMapParameters(time);
  return flowMapADInterfacePtr_->getFunctionValue(tapedTimeStateInput, parameters);
}

/*******************q**********************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t SystemDynamicsBaseAD::computeJumpMap(scalar_t time, const vector_t& state) {
  vector_t tapedTimeState(1 + state.rows());
  tapedTimeState << time, state;
  vector_t parameters = getJumpMapParameters(time);
  return jumpMapADInterfacePtr_->getFunctionValue(tapedTimeState, parameters);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t SystemDynamicsBaseAD::computeGuardSurfaces(scalar_t time, const vector_t& state) {
  vector_t tapedTimeState(1 + state.rows());
  tapedTimeState << time, state;
  vector_t parameters = getGuardSurfacesParameters(time);
  return guardSurfacesADInterfacePtr_->getFunctionValue(tapedTimeState, parameters);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation SystemDynamicsBaseAD::linearApproximation(scalar_t t, const vector_t& x, const vector_t& u) {
  vector_t tapedTimeStateInput(1 + x.rows() + u.rows());
  tapedTimeStateInput << t, x, u;
  vector_t parameters = getFlowMapParameters(t);
  flowJacobian_ = flowMapADInterfacePtr_->getJacobian(tapedTimeStateInput, parameters);

  VectorFunctionLinearApproximation approximation;
  approximation.dfdx = flowJacobian_.middleCols(1, x.rows());
  approximation.dfdu = flowJacobian_.rightCols(u.rows());
  approximation.f = flowMapADInterfacePtr_->getFunctionValue(tapedTimeStateInput, parameters);
  return approximation;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation SystemDynamicsBaseAD::jumpMapLinearApproximation(scalar_t t, const vector_t& x) {
  vector_t tapedTimeState(1 + x.rows());
  tapedTimeState << t, x;
  vector_t parameters = getJumpMapParameters(t);
  jumpJacobian_ = jumpMapADInterfacePtr_->getJacobian(tapedTimeState, parameters);

  VectorFunctionLinearApproximation approximation;
  approximation.dfdx = jumpJacobian_.rightCols(x.rows());
  approximation.dfdu.setZero(jumpJacobian_.rows(), 0);
  approximation.f = jumpMapADInterfacePtr_->getFunctionValue(tapedTimeState, parameters);
  return approximation;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation SystemDynamicsBaseAD::guardSurfacesLinearApproximation(scalar_t t, const vector_t& x, const vector_t& u) {
  vector_t tapedTimeState(1 + x.rows());
  tapedTimeState << t, x;
  vector_t parameters = getGuardSurfacesParameters(t);
  guardJacobian_ = guardSurfacesADInterfacePtr_->getJacobian(tapedTimeState, parameters);

  VectorFunctionLinearApproximation approximation;
  approximation.dfdx = guardJacobian_.rightCols(x.rows());
  approximation.dfdu = matrix_t::Zero(guardJacobian_.rows(), u.rows());  // not provided
  approximation.f = guardSurfacesADInterfacePtr_->getFunctionValue(tapedTimeState, parameters);
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

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SystemDynamicsBaseAD::setADInterfaces(const std::string& modelName, const std::string& modelFolder) {
  auto tapedFlowMap = [this](const ad_vector_t& x, const ad_vector_t& p, ad_vector_t& y) {
    auto time = x(0);
    auto state = x.segment(1, stateDim_);
    auto input = x.tail(inputDim_);
    y = this->systemFlowMap(time, state, input, p);
  };
  flowMapADInterfacePtr_.reset(
      new CppAdInterface(tapedFlowMap, 1 + stateDim_ + inputDim_, getNumFlowMapParameters(), modelName + "_flow_map", modelFolder));

  auto tapedJumpMap = [this](const ad_vector_t& x, const ad_vector_t& p, ad_vector_t& y) {
    auto time = x(0);
    auto state = x.tail(stateDim_);
    y = this->systemJumpMap(time, state, p);
  };
  jumpMapADInterfacePtr_.reset(
      new CppAdInterface(tapedJumpMap, 1 + stateDim_, getNumJumpMapParameters(), modelName + "_jump_map", modelFolder));

  auto tapedGuardSurfaces = [this](const ad_vector_t& x, const ad_vector_t& p, ad_vector_t& y) {
    auto time = x(0);
    auto state = x.tail(stateDim_);
    y = this->systemGuardSurfaces(time, state, p);
  };
  guardSurfacesADInterfacePtr_.reset(
      new CppAdInterface(tapedGuardSurfaces, 1 + stateDim_, getNumGuardSurfacesParameters(), modelName + "_guard_surfaces", modelFolder));
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
