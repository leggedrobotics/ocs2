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

#include <ocs2_core/cost/RelaxedBarrierCost.h>
#include <algorithm>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
RelaxedBarrierCost::RelaxedBarrierCost(Config config, size_t stateDim, size_t inputDim, size_t intermediateCostDim, size_t terminalCostDim)
    : RelaxedBarrierCost(std::vector<Config>(intermediateCostDim, config), std::vector<Config>(terminalCostDim, config), stateDim,
                         inputDim) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
RelaxedBarrierCost::RelaxedBarrierCost(std::vector<Config> intermediateConfig, std::vector<Config> terminalConfig, size_t stateDim,
                                       size_t inputDim)
    : CostFunctionBase(),
      stateDim_(stateDim),
      inputDim_(inputDim),
      intermediateDerivativesComputed_(false),
      intermediateCostValuesComputed_(false),
      terminalDerivativesComputed_(false),
      terminalCostValuesComputed_(false),
      intermediateConfig_(std::move(intermediateConfig)),
      terminalConfig_(std::move(terminalConfig)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
RelaxedBarrierCost::RelaxedBarrierCost(const RelaxedBarrierCost& rhs)
    : CostFunctionBase(rhs),
      stateDim_(rhs.stateDim_),
      inputDim_(rhs.inputDim_),
      intermediateADInterfacePtr_(new CppAdInterface(*rhs.intermediateADInterfacePtr_)),
      terminalADInterfacePtr_(new CppAdInterface(*rhs.terminalADInterfacePtr_)),
      intermediateDerivativesComputed_(false),
      intermediateCostValuesComputed_(false),
      terminalDerivativesComputed_(false),
      terminalCostValuesComputed_(false),
      intermediateConfig_(rhs.intermediateConfig_),
      terminalConfig_(rhs.terminalConfig_) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void RelaxedBarrierCost::initialize(const std::string& modelName, const std::string& modelFolder, bool recompileLibraries, bool verbose) {
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
void RelaxedBarrierCost::setCurrentStateAndControl(scalar_t t, const vector_t& x, const vector_t& u) {
  CostFunctionBase::setCurrentStateAndControl(t, x, u);

  tapedTimeState_.resize(1 + stateDim_);
  tapedTimeStateInput_.resize(1 + stateDim_ + inputDim_);

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
scalar_t RelaxedBarrierCost::getCost() {
  scalar_t L = 0;
  if (intermediateConfig_.size() == 0) {
    return L;
  }

  if (!intermediateCostValuesComputed_) {
    intermediateCostValues_ = intermediateADInterfacePtr_->getFunctionValue(tapedTimeStateInput_, intermediateParameters_);
    intermediateCostValuesComputed_ = true;
  }
  for (int i = 0; i < intermediateConfig_.size(); i++) {
    L += getPenaltyFunctionValue(intermediateCostValues_[i], intermediateConfig_[i]);
  }
  return L;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t RelaxedBarrierCost::getCostDerivativeTime() {
  if (intermediateConfig_.size() == 0) {
    return 0;
  }

  if (!intermediateDerivativesComputed_) {
    intermediateJacobian_ = intermediateADInterfacePtr_->getJacobian(tapedTimeStateInput_, intermediateParameters_);
    intermediateDerivativesComputed_ = true;
  }
  if (!intermediateCostValuesComputed_) {
    intermediateCostValues_ = intermediateADInterfacePtr_->getFunctionValue(tapedTimeStateInput_, intermediateParameters_);
    intermediateCostValuesComputed_ = true;
  }

  vector_t penalityFctDerivative(intermediateConfig_.size());
  for (int i = 0; i < intermediateConfig_.size(); i++) {
    penalityFctDerivative(i) = getPenaltyFunctionDerivative(intermediateCostValues_[i], intermediateConfig_[i]);
  }
  return penalityFctDerivative.dot(intermediateJacobian_.col(0));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t RelaxedBarrierCost::getCostDerivativeState() {
  if (intermediateConfig_.size() == 0) {
    return vector_t::Zero(stateDim_);
  }

  if (!intermediateDerivativesComputed_) {
    intermediateJacobian_ = intermediateADInterfacePtr_->getJacobian(tapedTimeStateInput_, intermediateParameters_);
    intermediateDerivativesComputed_ = true;
  }
  if (!intermediateCostValuesComputed_) {
    intermediateCostValues_ = intermediateADInterfacePtr_->getFunctionValue(tapedTimeStateInput_, intermediateParameters_);
    intermediateCostValuesComputed_ = true;
  }

  vector_t penalityFctDerivative(intermediateConfig_.size());
  for (int i = 0; i < intermediateConfig_.size(); i++) {
    penalityFctDerivative(i) = getPenaltyFunctionDerivative(intermediateCostValues_[i], intermediateConfig_[i]);
  }
  return intermediateJacobian_.block(0, 1, intermediateConfig_.size(), stateDim_).transpose() * penalityFctDerivative;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_t RelaxedBarrierCost::getCostSecondDerivativeState() {
  if (intermediateConfig_.size() == 0) {
    return matrix_t::Zero(stateDim_, stateDim_);
  }

  if (!intermediateDerivativesComputed_) {
    intermediateJacobian_ = intermediateADInterfacePtr_->getJacobian(tapedTimeStateInput_, intermediateParameters_);
    intermediateDerivativesComputed_ = true;
  }
  if (!intermediateCostValuesComputed_) {
    intermediateCostValues_ = intermediateADInterfacePtr_->getFunctionValue(tapedTimeStateInput_, intermediateParameters_);
    intermediateCostValuesComputed_ = true;
  }

  vector_t penalityFctSecondDerivative(intermediateConfig_.size());
  for (int i = 0; i < intermediateConfig_.size(); i++) {
    penalityFctSecondDerivative(i) = getPenaltyFunctionSecondDerivative(intermediateCostValues_[i], intermediateConfig_[i]);
  }
  return intermediateJacobian_.block(0, 1, intermediateConfig_.size(), stateDim_).transpose() * penalityFctSecondDerivative.asDiagonal() *
         intermediateJacobian_.block(0, 1, intermediateConfig_.size(), stateDim_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t RelaxedBarrierCost::getCostDerivativeInput() {
  if (intermediateConfig_.size() == 0) {
    return vector_t::Zero(inputDim_);
  }

  if (!intermediateDerivativesComputed_) {
    intermediateJacobian_ = intermediateADInterfacePtr_->getJacobian(tapedTimeStateInput_, intermediateParameters_);
    intermediateDerivativesComputed_ = true;
  }
  if (!intermediateCostValuesComputed_) {
    intermediateCostValues_ = intermediateADInterfacePtr_->getFunctionValue(tapedTimeStateInput_, intermediateParameters_);
    intermediateCostValuesComputed_ = true;
  }

  vector_t penalityFctDerivative(intermediateConfig_.size());
  for (int i = 0; i < intermediateConfig_.size(); i++) {
    penalityFctDerivative(i) = getPenaltyFunctionDerivative(intermediateCostValues_[i], intermediateConfig_[i]);
  }
  return intermediateJacobian_.block(0, 1 + stateDim_, intermediateConfig_.size(), inputDim_).transpose() * penalityFctDerivative;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_t RelaxedBarrierCost::getCostSecondDerivativeInput() {
  if (intermediateConfig_.size() == 0) {
    return matrix_t::Zero(inputDim_, inputDim_);
  }

  if (!intermediateDerivativesComputed_) {
    intermediateJacobian_ = intermediateADInterfacePtr_->getJacobian(tapedTimeStateInput_, intermediateParameters_);
    intermediateDerivativesComputed_ = true;
  }
  if (!intermediateCostValuesComputed_) {
    intermediateCostValues_ = intermediateADInterfacePtr_->getFunctionValue(tapedTimeStateInput_, intermediateParameters_);
    intermediateCostValuesComputed_ = true;
  }

  vector_t penalityFctSecondDerivative(intermediateConfig_.size());
  for (int i = 0; i < intermediateConfig_.size(); i++) {
    penalityFctSecondDerivative(i) = getPenaltyFunctionSecondDerivative(intermediateCostValues_[i], intermediateConfig_[i]);
  }
  return intermediateJacobian_.block(0, 1 + stateDim_, intermediateConfig_.size(), inputDim_).transpose() *
         penalityFctSecondDerivative.asDiagonal() * intermediateJacobian_.block(0, 1 + stateDim_, intermediateConfig_.size(), inputDim_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_t RelaxedBarrierCost::getCostDerivativeInputState() {
  if (intermediateConfig_.size() == 0) {
    return matrix_t::Zero(inputDim_, stateDim_);
  }

  if (!intermediateDerivativesComputed_) {
    intermediateJacobian_ = intermediateADInterfacePtr_->getJacobian(tapedTimeStateInput_, intermediateParameters_);
    intermediateDerivativesComputed_ = true;
  }

  vector_t penalityFctSecondDerivative(intermediateConfig_.size());
  for (int i = 0; i < intermediateConfig_.size(); i++) {
    penalityFctSecondDerivative(i) = getPenaltyFunctionSecondDerivative(intermediateCostValues_[i], intermediateConfig_[i]);
  }
  return intermediateJacobian_.block(0, 1 + stateDim_, intermediateConfig_.size(), inputDim_).transpose() *
         penalityFctSecondDerivative.asDiagonal() * intermediateJacobian_.block(0, 1, intermediateConfig_.size(), stateDim_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t RelaxedBarrierCost::getTerminalCost() {
  if (terminalConfig_.size() == 0) {
    return 0;
  }

  if (!terminalCostValuesComputed_) {
    terminalCostValues_ = terminalADInterfacePtr_->getFunctionValue(tapedTimeState_, terminalParameters_);
    terminalCostValuesComputed_ = true;
  }

  scalar_t Phi = 0;
  for (int i = 0; i < terminalConfig_.size(); i++) {
    Phi += getPenaltyFunctionValue(terminalCostValues_[i], terminalConfig_[i]);
  }
  return Phi;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t RelaxedBarrierCost::getTerminalCostDerivativeTime() {
  if (terminalConfig_.size() == 0) {
    return 0;
  }

  if (!terminalDerivativesComputed_) {
    terminalJacobian_ = terminalADInterfacePtr_->getJacobian(tapedTimeState_, terminalParameters_);
    terminalDerivativesComputed_ = true;
  }
  if (!terminalCostValuesComputed_) {
    terminalCostValues_ = terminalADInterfacePtr_->getFunctionValue(tapedTimeState_, terminalParameters_);
    terminalCostValuesComputed_ = true;
  }

  vector_t penalityFctDerivative(terminalConfig_.size());
  for (int i = 0; i < terminalConfig_.size(); i++) {
    penalityFctDerivative(i) = getPenaltyFunctionDerivative(terminalCostValues_[i], terminalConfig_[i]);
  }
  return penalityFctDerivative.transpose() * terminalJacobian_.col(0);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t RelaxedBarrierCost::getTerminalCostDerivativeState() {
  if (terminalConfig_.size() == 0) {
    return vector_t::Zero(stateDim_);
  }

  if (!terminalDerivativesComputed_) {
    terminalJacobian_ = terminalADInterfacePtr_->getJacobian(tapedTimeState_, terminalParameters_);
    terminalDerivativesComputed_ = true;
  }
  if (!terminalCostValuesComputed_) {
    terminalCostValues_ = terminalADInterfacePtr_->getFunctionValue(tapedTimeState_, terminalParameters_);
    terminalCostValuesComputed_ = true;
  }

  vector_t penalityFctDerivative(terminalConfig_.size());
  for (int i = 0; i < terminalConfig_.size(); i++) {
    penalityFctDerivative(i) = getPenaltyFunctionDerivative(terminalCostValues_[i], terminalConfig_[i]);
  }
  return terminalJacobian_.block(0, 1, terminalConfig_.size(), stateDim_).transpose() * penalityFctDerivative;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_t RelaxedBarrierCost::getTerminalCostSecondDerivativeState() {
  if (terminalConfig_.size() == 0) {
    return matrix_t::Zero(stateDim_, stateDim_);
  }

  if (!terminalDerivativesComputed_) {
    terminalJacobian_ = terminalADInterfacePtr_->getJacobian(tapedTimeState_, terminalParameters_);
    terminalDerivativesComputed_ = true;
  }
  if (!terminalCostValuesComputed_) {
    terminalCostValues_ = terminalADInterfacePtr_->getFunctionValue(tapedTimeState_, terminalParameters_);
    terminalCostValuesComputed_ = true;
  }

  vector_t penalityFctSecondDerivative(terminalConfig_.size());
  for (int i = 0; i < terminalConfig_.size(); i++) {
    penalityFctSecondDerivative(i) = getPenaltyFunctionSecondDerivative(terminalCostValues_[i], terminalConfig_[i]);
  }
  return terminalJacobian_.block(0, 1, terminalConfig_.size(), stateDim_).transpose() * penalityFctSecondDerivative.asDiagonal() *
         terminalJacobian_.block(0, 1, terminalConfig_.size(), stateDim_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t RelaxedBarrierCost::getIntermediateParameters(scalar_t time) const {
  return vector_t(0);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
size_t RelaxedBarrierCost::getNumIntermediateParameters() const {
  return 0;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t RelaxedBarrierCost::getTerminalParameters(scalar_t time) const {
  return vector_t(0);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
size_t RelaxedBarrierCost::getNumTerminalParameters() const {
  return 0;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
RelaxedBarrierCost::ad_vector_t RelaxedBarrierCost::terminalCostFunction(ad_scalar_t time, const ad_vector_t& state,
                                                                         const ad_vector_t& parameters) const {
  return ad_vector_t::Zero(terminalConfig_.size());
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void RelaxedBarrierCost::setADInterfaces(const std::string& modelName, const std::string& modelFolder) {
  auto intermediateCostAd = [this](const ad_vector_t& x, const ad_vector_t& p, ad_vector_t& y) {
    auto time = x(0);
    auto state = x.segment(1, stateDim_);
    auto input = x.tail(inputDim_);
    y = this->intermediateCostFunction(time, state, input, p);
  };
  intermediateADInterfacePtr_.reset(new CppAdInterface(intermediateCostAd, 1 + stateDim_ + inputDim_, getNumIntermediateParameters(),
                                                       modelName + "_intermediate", modelFolder));

  auto terminalCostAd = [this](const ad_vector_t& x, const ad_vector_t& p, ad_vector_t& y) {
    auto time = x(0);
    auto state = x.tail(stateDim_);
    y = this->terminalCostFunction(time, state, p);
  };
  terminalADInterfacePtr_.reset(
      new CppAdInterface(terminalCostAd, 1 + stateDim_, getNumTerminalParameters(), modelName + "_terminal", modelFolder));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void RelaxedBarrierCost::createModels(bool verbose) {
  intermediateADInterfacePtr_->createModels(CppAdInterface::ApproximationOrder::First, verbose);
  terminalADInterfacePtr_->createModels(CppAdInterface::ApproximationOrder::First, verbose);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void RelaxedBarrierCost::loadModelsIfAvailable(bool verbose) {
  intermediateADInterfacePtr_->loadModelsIfAvailable(CppAdInterface::ApproximationOrder::First, verbose);
  terminalADInterfacePtr_->loadModelsIfAvailable(CppAdInterface::ApproximationOrder::First, verbose);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t RelaxedBarrierCost::getPenaltyFunctionValue(scalar_t h, const Config& config) const {
  if (h > config.delta) {
    return -config.mu * log(h);
  } else {
    return config.mu * (-log(config.delta) + scalar_t(0.5) * pow((h - 2.0 * config.delta) / config.delta, 2.0) - scalar_t(0.5));
  };
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t RelaxedBarrierCost::getPenaltyFunctionDerivative(scalar_t h, const Config& config) const {
  if (h > config.delta) {
    return -config.mu / h;
  } else {
    return config.mu * ((h - 2.0 * config.delta) / (config.delta * config.delta));
  };
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t RelaxedBarrierCost::getPenaltyFunctionSecondDerivative(scalar_t h, const Config& config) const {
  if (h > config.delta) {
    return config.mu / (h * h);
  } else {
    return config.mu / (config.delta * config.delta);
  };
};

}  // namespace ocs2
