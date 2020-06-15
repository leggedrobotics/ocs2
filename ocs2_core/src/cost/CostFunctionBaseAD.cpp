/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

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

#include <ocs2_core/cost/CostFunctionBaseAD.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
CostFunctionBaseAD::CostFunctionBaseAD(size_t stateDim, size_t inputDim) : CostFunctionBase(), stateDim_(stateDim), inputDim_(inputDim) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
CostFunctionBaseAD::CostFunctionBaseAD(const CostFunctionBaseAD& rhs)
    : CostFunctionBase(rhs),
      stateDim_(rhs.stateDim_),
      inputDim_(rhs.inputDim_),
      intermediateADInterfacePtr_(new CppAdInterface(*rhs.intermediateADInterfacePtr_)),
      finalADInterfacePtr_(new CppAdInterface(*rhs.finalADInterfacePtr_)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void CostFunctionBaseAD::initialize(const std::string& modelName, const std::string& modelFolder, bool recompileLibraries, bool verbose) {
  setADInterfaces(modelName, modelFolder);
  if (recompileLibraries) {
    createModels(verbose);
  } else {
    loadModelsIfAvailable(verbose);
  }
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t CostFunctionBaseAD::cost(scalar_t t, const vector_t& x, const vector_t& u) {
  tapedTimeStateInput_.resize(1 + stateDim_ + inputDim_);
  tapedTimeStateInput_ << t, x, u;
  intermediateParameters_ = getIntermediateParameters(t);
  return intermediateADInterfacePtr_->getFunctionValue(tapedTimeStateInput_, intermediateParameters_)(0);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t CostFunctionBaseAD::finalCost(scalar_t t, const vector_t& x) {
  tapedTimeState_.resize(1 + stateDim_);
  tapedTimeState_ << t, x;
  finalParameters_ = getFinalParameters(t);
  return finalADInterfacePtr_->getFunctionValue(tapedTimeState_, finalParameters_)(0);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation CostFunctionBaseAD::costQuadraticApproximation(scalar_t t, const vector_t& x, const vector_t& u) {
  tapedTimeStateInput_.resize(1 + stateDim_ + inputDim_);
  tapedTimeStateInput_ << t, x, u;
  intermediateParameters_ = getIntermediateParameters(t);

  intermediateJacobian_ = intermediateADInterfacePtr_->getJacobian(tapedTimeStateInput_, intermediateParameters_);
  intermediateHessian_ = intermediateADInterfacePtr_->getHessian(0, tapedTimeStateInput_, intermediateParameters_);

  ScalarFunctionQuadraticApproximation L;
  L.dfdxx = intermediateHessian_.block(1, 1, stateDim_, stateDim_);
  L.dfdux = intermediateHessian_.block(1 + stateDim_, 1, inputDim_, stateDim_);
  L.dfduu = intermediateHessian_.block(1 + stateDim_, 1 + stateDim_, inputDim_, inputDim_);
  L.dfdx = intermediateJacobian_.segment(1, stateDim_).transpose();
  L.dfdu = intermediateJacobian_.segment(1 + stateDim_, inputDim_).transpose();
  L.f = intermediateADInterfacePtr_->getFunctionValue(tapedTimeStateInput_, intermediateParameters_)(0);
  return L;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation CostFunctionBaseAD::finalCostQuadraticApproximation(scalar_t t, const vector_t& x) {
  tapedTimeState_.resize(1 + stateDim_);
  tapedTimeState_ << t, x;
  finalParameters_ = getFinalParameters(t);

  finalJacobian_ = finalADInterfacePtr_->getJacobian(tapedTimeState_, finalParameters_);
  finalHessian_ = finalADInterfacePtr_->getHessian(0, tapedTimeState_, finalParameters_);

  ScalarFunctionQuadraticApproximation Phi;
  Phi.dfdxx = finalHessian_.block(1, 1, stateDim_, stateDim_);
  Phi.dfdx = finalJacobian_.segment(1, stateDim_).transpose();
  Phi.f = finalADInterfacePtr_->getFunctionValue(tapedTimeState_, finalParameters_)(0);
  return Phi;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t CostFunctionBaseAD::costDerivativeTime(scalar_t t, const vector_t& x, const vector_t& u) {
  return intermediateJacobian_(0);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t CostFunctionBaseAD::finalCostDerivativeTime(scalar_t t, const vector_t& x) {
  return finalJacobian_(0);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t CostFunctionBaseAD::getIntermediateParameters(scalar_t time) const {
  return vector_t(0);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
size_t CostFunctionBaseAD::getNumIntermediateParameters() const {
  return 0;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t CostFunctionBaseAD::getFinalParameters(scalar_t time) const {
  return vector_t(0);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
size_t CostFunctionBaseAD::getNumFinalParameters() const {
  return 0;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
CostFunctionBaseAD::ad_scalar_t CostFunctionBaseAD::finalCostFunction(ad_scalar_t time, const ad_vector_t& state,
                                                                      const ad_vector_t& parameters) const {
  return ad_scalar_t(0);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void CostFunctionBaseAD::setADInterfaces(const std::string& modelName, const std::string& modelFolder) {
  auto intermediateCostAd = [this](const ad_vector_t& x, const ad_vector_t& p, ad_vector_t& y) {
    auto time = x(0);
    auto state = x.segment(1, stateDim_);
    auto input = x.segment(1 + stateDim_, inputDim_);
    y = ad_vector_t(1);
    y(0) = this->intermediateCostFunction(time, state, input, p);
  };
  intermediateADInterfacePtr_.reset(new CppAdInterface(intermediateCostAd, 1 + stateDim_ + inputDim_, getNumIntermediateParameters(),
                                                       modelName + "_intermediate", modelFolder));

  auto finalCostAd = [this](const ad_vector_t& x, const ad_vector_t& p, ad_vector_t& y) {
    auto time = x(0);
    auto state = x.segment(1, stateDim_);
    y = ad_vector_t(1);
    y(0) = this->finalCostFunction(time, state, p);
  };
  finalADInterfacePtr_.reset(new CppAdInterface(finalCostAd, 1 + stateDim_, getNumFinalParameters(), modelName + "_final", modelFolder));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void CostFunctionBaseAD::createModels(bool verbose) {
  intermediateADInterfacePtr_->createModels(CppAdInterface::ApproximationOrder::Second, verbose);
  finalADInterfacePtr_->createModels(CppAdInterface::ApproximationOrder::Second, verbose);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void CostFunctionBaseAD::loadModelsIfAvailable(bool verbose) {
  intermediateADInterfacePtr_->loadModelsIfAvailable(CppAdInterface::ApproximationOrder::Second, verbose);
  finalADInterfacePtr_->loadModelsIfAvailable(CppAdInterface::ApproximationOrder::Second, verbose);
}

}  // namespace ocs2
