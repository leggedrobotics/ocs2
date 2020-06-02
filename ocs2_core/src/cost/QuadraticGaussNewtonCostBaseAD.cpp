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
QuadraticGaussNewtonCostBaseAD::QuadraticGaussNewtonCostBaseAD(size_t stateDim, size_t inputDim, size_t intermediateCostDim,
                                                               size_t terminalCostDim)
    : CostFunctionBase(),
      stateDim_(stateDim),
      inputDim_(inputDim),
      intermediateCostDim_(intermediateCostDim),
      terminalCostDim_(terminalCostDim),
      intermediateCostValuesComputed_(false),
      intermediateCostValues_(intermediateCostDim),
      intermediateDerivativesComputed_(false),
      intermediateParameters_(0),
      tapedTimeStateInput_(1 + stateDim + inputDim),
      intermediateJacobian_(intermediateCostDim, 1 + stateDim + inputDim),
      terminalDerivativesComputed_(false),
      terminalCostValues_(terminalCostDim),
      terminalCostValuesComputed_(false),
      terminalParameters_(0),
      tapedTimeState_(1 + stateDim),
      terminalJacobian_(terminalCostDim, 1 + stateDim) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
QuadraticGaussNewtonCostBaseAD::QuadraticGaussNewtonCostBaseAD(const QuadraticGaussNewtonCostBaseAD& rhs)
    : QuadraticGaussNewtonCostBaseAD(rhs.stateDim_, rhs.inputDim_, rhs.intermediateCostDim_, rhs.terminalCostDim_),
      intermediateADInterfacePtr_(new CppAdInterface(*rhs.intermediateADInterfacePtr_)),
      terminalADInterfacePtr_(new CppAdInterface(*rhs.terminalADInterfacePtr_)) {}

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
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void QuadraticGaussNewtonCostBaseAD::setCurrentStateAndControl(scalar_t t, const vector_t& x, const vector_t& u) {
  CostFunctionBase::setCurrentStateAndControl(t, x, u);

  tapedTimeState_ << t, x;
  tapedTimeStateInput_ << t, x, u;

  intermediateParameters_ = getIntermediateParameters(t);
  terminalParameters_ = getTerminalParameters(t);

  intermediateCostValuesComputed_ = false;
  intermediateDerivativesComputed_ = false;
  terminalDerivativesComputed_ = false;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t QuadraticGaussNewtonCostBaseAD::getCost() {
  if (!intermediateCostValuesComputed_) {
    intermediateCostValues_ = intermediateADInterfacePtr_->getFunctionValue(tapedTimeStateInput_, intermediateParameters_);
    intermediateCostValuesComputed_ = true;
  }
  return 0.5 * intermediateCostValues_.dot(intermediateCostValues_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t QuadraticGaussNewtonCostBaseAD::getCostDerivativeTime() {
  if (!intermediateDerivativesComputed_) {
    intermediateJacobian_ = intermediateADInterfacePtr_->getJacobian(tapedTimeStateInput_, intermediateParameters_);
    intermediateDerivativesComputed_ = true;
  }
  if (!intermediateCostValuesComputed_) {
    intermediateCostValues_ = intermediateADInterfacePtr_->getFunctionValue(tapedTimeStateInput_, intermediateParameters_);
    intermediateCostValuesComputed_ = true;
  }
  return intermediateCostValues_.transpose() * intermediateJacobian_.col(0);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t QuadraticGaussNewtonCostBaseAD::getCostDerivativeState() {
  if (!intermediateDerivativesComputed_) {
    intermediateJacobian_ = intermediateADInterfacePtr_->getJacobian(tapedTimeStateInput_, intermediateParameters_);
    intermediateDerivativesComputed_ = true;
  }
  if (!intermediateCostValuesComputed_) {
    intermediateCostValues_ = intermediateADInterfacePtr_->getFunctionValue(tapedTimeStateInput_, intermediateParameters_);
    intermediateCostValuesComputed_ = true;
  }
  return intermediateJacobian_.block(0, 1, intermediateCostDim_, stateDim_).transpose() * intermediateCostValues_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_t QuadraticGaussNewtonCostBaseAD::getCostSecondDerivativeState() {
  if (!intermediateDerivativesComputed_) {
    intermediateJacobian_ = intermediateADInterfacePtr_->getJacobian(tapedTimeStateInput_, intermediateParameters_);
    intermediateDerivativesComputed_ = true;
  }
  if (!intermediateCostValuesComputed_) {
    intermediateCostValues_ = intermediateADInterfacePtr_->getFunctionValue(tapedTimeStateInput_, intermediateParameters_);
    intermediateCostValuesComputed_ = true;
  }
  return intermediateJacobian_.block(0, 1, intermediateCostDim_, stateDim_).transpose() *
         intermediateJacobian_.block(0, 1, intermediateCostDim_, stateDim_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t QuadraticGaussNewtonCostBaseAD::getCostDerivativeInput() {
  if (!intermediateDerivativesComputed_) {
    intermediateJacobian_ = intermediateADInterfacePtr_->getJacobian(tapedTimeStateInput_, intermediateParameters_);
    intermediateDerivativesComputed_ = true;
  }
  if (!intermediateCostValuesComputed_) {
    intermediateCostValues_ = intermediateADInterfacePtr_->getFunctionValue(tapedTimeStateInput_, intermediateParameters_);
    intermediateCostValuesComputed_ = true;
  }
  dLdu = intermediateJacobian_.block(0, 1 + stateDim_, intermediateCostDim_, inputDim_).transpose() * intermediateCostValues_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_t QuadraticGaussNewtonCostBaseAD::getCostSecondDerivativeInput() {
  if (!intermediateDerivativesComputed_) {
    intermediateJacobian_ = intermediateADInterfacePtr_->getJacobian(tapedTimeStateInput_, intermediateParameters_);
    intermediateDerivativesComputed_ = true;
  }
  if (!intermediateCostValuesComputed_) {
    intermediateCostValues_ = intermediateADInterfacePtr_->getFunctionValue(tapedTimeStateInput_, intermediateParameters_);
    intermediateCostValuesComputed_ = true;
  }
  return intermediateJacobian_.block(0, 1 + stateDim_, intermediateCostDim_, inputDim_).transpose() *
         intermediateJacobian_.block(0, 1 + stateDim_, intermediateCostDim_, inputDim_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_t QuadraticGaussNewtonCostBaseAD::getCostDerivativeInputState() {
  if (!intermediateDerivativesComputed_) {
    intermediateJacobian_ = intermediateADInterfacePtr_->getJacobian(tapedTimeStateInput_, intermediateParameters_);
    intermediateDerivativesComputed_ = true;
  }
  return intermediateJacobian_.block(0, 1 + stateDim_, intermediateCostDim_, inputDim_).transpose() *
         intermediateJacobian_.block(0, 1, intermediateCostDim_, stateDim_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t QuadraticGaussNewtonCostBaseAD::getTerminalCost() {
  if (terminalCostValuesComputed_) {
    terminalCostValues_ = terminalADInterfacePtr_->getFunctionValue(tapedTimeState_, terminalParameters_);
    terminalCostValuesComputed_ = true;
  }
  return 0.5 * terminalCostValues_.dot(terminalCostValues_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t QuadraticGaussNewtonCostBaseAD::getTerminalCostDerivativeTime() {
  if (!terminalDerivativesComputed_) {
    terminalJacobian_ = terminalADInterfacePtr_->getJacobian(tapedTimeState_, terminalParameters_);
    terminalDerivativesComputed_ = true;
  }
  if (terminalCostValuesComputed_) {
    terminalCostValues_ = terminalADInterfacePtr_->getFunctionValue(tapedTimeState_, terminalParameters_);
    terminalCostValuesComputed_ = true;
  }
  return terminalCostValues_.transpose() * terminalJacobian_.col(0);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t QuadraticGaussNewtonCostBaseAD::getTerminalCostDerivativeState() {
  if (!terminalDerivativesComputed_) {
    terminalJacobian_ = terminalADInterfacePtr_->getJacobian(tapedTimeState_, terminalParameters_);
    terminalDerivativesComputed_ = true;
  }
  if (terminalCostValuesComputed_) {
    terminalCostValues_ = terminalADInterfacePtr_->getFunctionValue(tapedTimeState_, terminalParameters_);
    terminalCostValuesComputed_ = true;
  }
  return terminalJacobian_.block(0, 1, terminalCostDim_, stateDim_).transpose() * terminalCostValues_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_t QuadraticGaussNewtonCostBaseAD::getTerminalCostSecondDerivativeState() {
  if (!terminalDerivativesComputed_) {
    terminalJacobian_ = terminalADInterfacePtr_->getJacobian(tapedTimeState_, terminalParameters_);
    terminalDerivativesComputed_ = true;
  }
  if (terminalCostValuesComputed_) {
    terminalCostValues_ = terminalADInterfacePtr_->getFunctionValue(tapedTimeState_, terminalParameters_);
    terminalCostValuesComputed_ = true;
  }
  return terminalJacobian_.block(0, 1, terminalCostDim_, stateDim_).transpose() *
         terminalJacobian_.block(0, 1, terminalCostDim_, stateDim_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t QuadraticGaussNewtonCostBaseAD::getIntermediateParameters(scalar_t time) const {
  return vector_t(0);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
size_t QuadraticGaussNewtonCostBaseAD::getNumIntermediateParameters() const {
  return 0;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t QuadraticGaussNewtonCostBaseAD::getTerminalParameters(scalar_t time) const {
  return vector_t(0);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
size_t QuadraticGaussNewtonCostBaseAD::getNumTerminalParameters() const {
  return 0;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t QuadraticGaussNewtonCostBaseAD::intermediateCostFunction(ad_scalar_t time, const vector_t& state, const vector_t& input,
                                                                  const vector_t& parameters) const = 0;

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t QuadraticGaussNewtonCostBaseAD::terminalCostFunction(ad_scalar_t time, const vector_t& state, const vector_t& parameters) const {
  return ad_vector_t::Zero(terminalCostDim_);
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
  intermediateADInterfacePtr_.reset(new CppAdInterface(intermediateCostAd, intermediateCostDim_, 1 + stateDim_ + inputDim_,
                                                       getNumIntermediateParameters(), modelName + "_intermediate", modelFolder));

  auto terminalCostAd = [this](const ad_vector_t& x, const ad_vector_t& p, ad_vector_t& y) {
    auto time = x(0);
    auto state = x.tail(stateDim_);
    y = this->terminalCostFunction(time, state, p);
  };
  terminalADInterfacePtr_.reset(new CppAdInterface(terminalCostAd, terminalCostDim_, 1 + stateDim_, getNumTerminalParameters(),
                                                   modelName + "_terminal", modelFolder));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void QuadraticGaussNewtonCostBaseAD::createModels(bool verbose) {
  intermediateADInterfacePtr_->createModels(CppAdInterface::ApproximationOrder::First, verbose);
  terminalADInterfacePtr_->createModels(CppAdInterface::ApproximationOrder::First, verbose);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void QuadraticGaussNewtonCostBaseAD::loadModelsIfAvailable(bool verbose) {
  intermediateADInterfacePtr_->loadModelsIfAvailable(CppAdInterface::ApproximationOrder::First, verbose);
  terminalADInterfacePtr_->loadModelsIfAvailable(CppAdInterface::ApproximationOrder::First, verbose);
}

}  // namespace ocs2
