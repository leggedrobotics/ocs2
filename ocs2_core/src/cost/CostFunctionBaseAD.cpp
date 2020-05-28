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
CostFunctionBaseAD::CostFunctionBaseAD(size_t stateDim, size_t inputDim)
    : CostFunctionBase(),
      stateDim_(stateDim),
      inputDim_(inputDim),
      intermediateDerivativesComputed_(false),
      terminalDerivativesComputed_(false) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
CostFunctionBaseAD::CostFunctionBaseAD(const CostFunctionBaseAD& rhs)
    : CostFunctionBase(rhs),
      stateDim_(rhs.stateDim_),
      inputDim_(rhs.inputDim_),
      intermediateADInterfacePtr_(new CppAdInterface(*rhs.intermediateADInterfacePtr_)),
      terminalADInterfacePtr_(new CppAdInterface(*rhs.terminalADInterfacePtr_)),
      intermediateDerivativesComputed_(false),
      terminalDerivativesComputed_(false) {}

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
void CostFunctionBaseAD::setCurrentStateAndControl(scalar_t t, const vector_t& x, const vector_t& u) {
  CostFunctionBase::setCurrentStateAndControl(t, x, u);

  tapedTimeState_.resize(1 + stateDim_);
  tapedTimeState_ << t, x;
  tapedTimeStateInput_.resize(1 + stateDim_ + inputDim_);
  tapedTimeStateInput_ << t, x, u;

  intermediateParameters_ = getIntermediateParameters(t);
  terminalParameters_ = getTerminalParameters(t);

  // TODO(mspieler): Remove caching and do all computation here, similar to ConstraintBaseAD
  intermediateDerivativesComputed_ = false;
  terminalDerivativesComputed_ = false;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t CostFunctionBaseAD::getIntermediateCost() {
  auto costValue = intermediateADInterfacePtr_->getFunctionValue(tapedTimeStateInput_, intermediateParameters_);
  return costValue(0);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t CostFunctionBaseAD::getIntermediateCostDerivativeTime() {
  if (!intermediateDerivativesComputed_) {
    intermediateJacobian_ = intermediateADInterfacePtr_->getJacobian(tapedTimeStateInput_, intermediateParameters_);
    intermediateHessian_ = intermediateADInterfacePtr_->getHessian(0, tapedTimeStateInput_, intermediateParameters_);
    intermediateDerivativesComputed_ = true;
  }
  return intermediateJacobian_(0);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t CostFunctionBaseAD::getIntermediateCostDerivativeState() {
  if (!intermediateDerivativesComputed_) {
    intermediateJacobian_ = intermediateADInterfacePtr_->getJacobian(tapedTimeStateInput_, intermediateParameters_);
    intermediateHessian_ = intermediateADInterfacePtr_->getHessian(0, tapedTimeStateInput_, intermediateParameters_);
    intermediateDerivativesComputed_ = true;
  }
  return intermediateJacobian_.segment(1, stateDim_).transpose();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_t CostFunctionBaseAD::getIntermediateCostSecondDerivativeState() {
  if (!intermediateDerivativesComputed_) {
    intermediateJacobian_ = intermediateADInterfacePtr_->getJacobian(tapedTimeStateInput_, intermediateParameters_);
    intermediateHessian_ = intermediateADInterfacePtr_->getHessian(0, tapedTimeStateInput_, intermediateParameters_);
    intermediateDerivativesComputed_ = true;
  }
  return intermediateHessian_.block(1, 1, stateDim_, stateDim_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t CostFunctionBaseAD::getIntermediateCostDerivativeInput() {
  if (!intermediateDerivativesComputed_) {
    intermediateJacobian_ = intermediateADInterfacePtr_->getJacobian(tapedTimeStateInput_, intermediateParameters_);
    intermediateHessian_ = intermediateADInterfacePtr_->getHessian(0, tapedTimeStateInput_, intermediateParameters_);
    intermediateDerivativesComputed_ = true;
  }
  return intermediateJacobian_.segment(1 + stateDim_, inputDim_).transpose();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_t CostFunctionBaseAD::getIntermediateCostSecondDerivativeInput() {
  if (!intermediateDerivativesComputed_) {
    intermediateJacobian_ = intermediateADInterfacePtr_->getJacobian(tapedTimeStateInput_, intermediateParameters_);
    intermediateHessian_ = intermediateADInterfacePtr_->getHessian(0, tapedTimeStateInput_, intermediateParameters_);
    intermediateDerivativesComputed_ = true;
  }
  return intermediateHessian_.block(1 + stateDim_, 1 + stateDim_, inputDim_, inputDim_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_t CostFunctionBaseAD::getIntermediateCostDerivativeInputState() {
  if (!intermediateDerivativesComputed_) {
    intermediateJacobian_ = intermediateADInterfacePtr_->getJacobian(tapedTimeStateInput_, intermediateParameters_);
    intermediateHessian_ = intermediateADInterfacePtr_->getHessian(0, tapedTimeStateInput_, intermediateParameters_);
    intermediateDerivativesComputed_ = true;
  }
  return intermediateHessian_.block(1 + stateDim_, 1, inputDim_, stateDim_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t CostFunctionBaseAD::getTerminalCost() {
  auto costValue = terminalADInterfacePtr_->getFunctionValue(tapedTimeState_, terminalParameters_);
  return costValue(0);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t CostFunctionBaseAD::getTerminalCostDerivativeTime() {
  if (!terminalDerivativesComputed_) {
    terminalJacobian_ = terminalADInterfacePtr_->getJacobian(tapedTimeState_, terminalParameters_);
    terminalHessian_ = terminalADInterfacePtr_->getHessian(0, tapedTimeState_, terminalParameters_);
    terminalDerivativesComputed_ = true;
  }
  return terminalJacobian_(0);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t CostFunctionBaseAD::getTerminalCostDerivativeState() {
  if (!terminalDerivativesComputed_) {
    terminalJacobian_ = terminalADInterfacePtr_->getJacobian(tapedTimeState_, terminalParameters_);
    terminalHessian_ = terminalADInterfacePtr_->getHessian(0, tapedTimeState_, terminalParameters_);
    terminalDerivativesComputed_ = true;
  }
  return terminalJacobian_.segment(1, stateDim_).transpose();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_t CostFunctionBaseAD::getTerminalCostSecondDerivativeState() {
  if (!terminalDerivativesComputed_) {
    terminalJacobian_ = terminalADInterfacePtr_->getJacobian(tapedTimeState_, terminalParameters_);
    terminalHessian_ = terminalADInterfacePtr_->getHessian(0, tapedTimeState_, terminalParameters_);
    terminalDerivativesComputed_ = true;
  }
  return terminalHessian_.block(1, 1, stateDim_, stateDim_);
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
vector_t CostFunctionBaseAD::getTerminalParameters(scalar_t time) const {
  return vector_t(0);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
size_t CostFunctionBaseAD::getNumTerminalParameters() const {
  return 0;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
CostFunctionBaseAD::ad_scalar_t CostFunctionBaseAD::terminalCostFunction(ad_scalar_t time, const ad_vector_t& state,
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

  auto terminalCostAd = [this](const ad_vector_t& x, const ad_vector_t& p, ad_vector_t& y) {
    auto time = x(0);
    auto state = x.segment(1, stateDim_);
    y = ad_vector_t(1);
    y(0) = this->terminalCostFunction(time, state, p);
  };
  terminalADInterfacePtr_.reset(
      new CppAdInterface(terminalCostAd, 1 + stateDim_, getNumTerminalParameters(), modelName + "_terminal", modelFolder));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void CostFunctionBaseAD::createModels(bool verbose) {
  intermediateADInterfacePtr_->createModels(CppAdInterface::ApproximationOrder::Second, verbose);
  terminalADInterfacePtr_->createModels(CppAdInterface::ApproximationOrder::Second, verbose);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void CostFunctionBaseAD::loadModelsIfAvailable(bool verbose) {
  intermediateADInterfacePtr_->loadModelsIfAvailable(CppAdInterface::ApproximationOrder::Second, verbose);
  terminalADInterfacePtr_->loadModelsIfAvailable(CppAdInterface::ApproximationOrder::Second, verbose);
}

}  // namespace ocs2
