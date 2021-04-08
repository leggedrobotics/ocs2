/******************************************************************************
Copyright (c) 2020, Johannes Pankert. All rights reserved.

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

#include <ocs2_core/cost/QuadraticGaussNewtonCostBaseAD.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
QuadraticGaussNewtonCostBaseAD::QuadraticGaussNewtonCostBaseAD(size_t stateDim, size_t inputDim)
    : stateDim_(stateDim), inputDim_(inputDim) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
QuadraticGaussNewtonCostBaseAD::QuadraticGaussNewtonCostBaseAD(const QuadraticGaussNewtonCostBaseAD& rhs)
    : CostFunctionBase(rhs),
      stateDim_(rhs.stateDim_),
      inputDim_(rhs.inputDim_),
      intermediateADInterfacePtr_(new CppAdInterface(*rhs.intermediateADInterfacePtr_)),
      finalADInterfacePtr_(new CppAdInterface(*rhs.finalADInterfacePtr_)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void QuadraticGaussNewtonCostBaseAD::initialize(const std::string& modelName, const std::string& modelFolder, bool recompileLibraries,
                                                bool verbose) {
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
scalar_t QuadraticGaussNewtonCostBaseAD::cost(scalar_t t, const vector_t& x, const vector_t& u) {
  tapedTimeStateInput_.resize(1 + stateDim_ + inputDim_);
  tapedTimeStateInput_ << t, x, u;
  intermediateParameters_ = getIntermediateParameters(t);
  intermediateCostValues_ = intermediateADInterfacePtr_->getFunctionValue(tapedTimeStateInput_, intermediateParameters_);
  return 0.5 * intermediateCostValues_.dot(intermediateCostValues_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t QuadraticGaussNewtonCostBaseAD::finalCost(scalar_t t, const vector_t& x) {
  tapedTimeState_.resize(1 + stateDim_);
  tapedTimeState_ << t, x;
  finalParameters_ = getFinalParameters(t);
  finalCostValues_ = finalADInterfacePtr_->getFunctionValue(tapedTimeState_, finalParameters_);
  return 0.5 * finalCostValues_.dot(finalCostValues_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation QuadraticGaussNewtonCostBaseAD::costQuadraticApproximation(scalar_t t, const vector_t& x,
                                                                                                const vector_t& u) {
  tapedTimeStateInput_.resize(1 + stateDim_ + inputDim_);
  tapedTimeStateInput_ << t, x, u;
  intermediateParameters_ = getIntermediateParameters(t);
  intermediateJacobian_ = intermediateADInterfacePtr_->getJacobian(tapedTimeStateInput_, intermediateParameters_);
  intermediateCostValues_ = intermediateADInterfacePtr_->getFunctionValue(tapedTimeStateInput_, intermediateParameters_);
  const size_t costDim = intermediateCostValues_.rows();

  ScalarFunctionQuadraticApproximation L;
  L.f = 0.5 * intermediateCostValues_.dot(intermediateCostValues_);
  L.dfdx = intermediateJacobian_.block(0, 1, costDim, stateDim_).transpose() * intermediateCostValues_;
  L.dfdxx = intermediateJacobian_.block(0, 1, costDim, stateDim_).transpose() * intermediateJacobian_.block(0, 1, costDim, stateDim_);
  L.dfdu = intermediateJacobian_.rightCols(inputDim_).transpose() * intermediateCostValues_;
  L.dfduu = intermediateJacobian_.rightCols(inputDim_).transpose() * intermediateJacobian_.rightCols(inputDim_);
  L.dfdux = intermediateJacobian_.rightCols(inputDim_).transpose() * intermediateJacobian_.block(0, 1, costDim, stateDim_);
  return L;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation QuadraticGaussNewtonCostBaseAD::finalCostQuadraticApproximation(scalar_t t, const vector_t& x) {
  tapedTimeState_.resize(1 + stateDim_);
  tapedTimeState_ << t, x;
  finalParameters_ = getFinalParameters(t);
  finalJacobian_ = finalADInterfacePtr_->getJacobian(tapedTimeState_, finalParameters_);
  finalCostValues_ = finalADInterfacePtr_->getFunctionValue(tapedTimeState_, finalParameters_);

  ScalarFunctionQuadraticApproximation Phi;
  Phi.f = 0.5 * finalCostValues_.dot(finalCostValues_);
  Phi.dfdx = finalJacobian_.rightCols(stateDim_).transpose() * finalCostValues_;
  Phi.dfdxx = finalJacobian_.rightCols(stateDim_).transpose() * finalJacobian_.rightCols(stateDim_);
  return Phi;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t QuadraticGaussNewtonCostBaseAD::costDerivativeTime(scalar_t t, const vector_t& x, const vector_t& u) {
  return intermediateCostValues_.transpose() * intermediateJacobian_.col(0);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t QuadraticGaussNewtonCostBaseAD::finalCostDerivativeTime(scalar_t t, const vector_t& x) {
  return finalCostValues_.transpose() * finalJacobian_.col(0);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void QuadraticGaussNewtonCostBaseAD::setADInterfaces(const std::string& modelName, const std::string& modelFolder) {
  auto intermediateCostAd = [this](const ad_vector_t& x, const ad_vector_t& p, ad_vector_t& y) {
    auto time = x(0);
    auto state = x.segment(1, stateDim_);
    auto input = x.tail(inputDim_);
    y = this->intermediateCostFunction(time, state, input, p);
  };
  intermediateADInterfacePtr_.reset(new CppAdInterface(intermediateCostAd, 1 + stateDim_ + inputDim_, getNumIntermediateParameters(),
                                                       modelName + "_intermediate", modelFolder));

  auto finalCostAd = [this](const ad_vector_t& x, const ad_vector_t& p, ad_vector_t& y) {
    auto time = x(0);
    auto state = x.tail(stateDim_);
    y = this->finalCostFunction(time, state, p);
  };
  finalADInterfacePtr_.reset(new CppAdInterface(finalCostAd, 1 + stateDim_, getNumFinalParameters(), modelName + "_final", modelFolder));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void QuadraticGaussNewtonCostBaseAD::createModels(bool verbose) {
  intermediateADInterfacePtr_->createModels(CppAdInterface::ApproximationOrder::First, verbose);
  finalADInterfacePtr_->createModels(CppAdInterface::ApproximationOrder::First, verbose);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void QuadraticGaussNewtonCostBaseAD::loadModelsIfAvailable(bool verbose) {
  intermediateADInterfacePtr_->loadModelsIfAvailable(CppAdInterface::ApproximationOrder::First, verbose);
  finalADInterfacePtr_->loadModelsIfAvailable(CppAdInterface::ApproximationOrder::First, verbose);
}

}  // namespace ocs2
