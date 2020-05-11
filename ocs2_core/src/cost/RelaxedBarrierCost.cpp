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
RelaxedBarrierCost::RelaxedBarrierCost(Config config, size_t state_dim, size_t input_dim, size_t intermediate_cost_dim,
                                       size_t terminal_cost_dim)
    : RelaxedBarrierCost(std::vector<Config>(intermediate_cost_dim), std::vector<Config>(terminal_cost_dim), state_dim, input_dim) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
RelaxedBarrierCost::RelaxedBarrierCost(std::vector<Config> intermediateConfig, std::vector<Config> terminalConfig, size_t state_dim,
                                       size_t input_dim)
    : CostFunctionBase(),
      state_dim_(state_dim),
      input_dim_(input_dim),
      intermediate_cost_dim_(intermediateConfig.size()),
      terminal_cost_dim_(terminalConfig.size()),
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
      state_dim_(rhs.state_dim_),
      input_dim_(rhs.input_dim_),
      intermediate_cost_dim_(rhs.intermediate_cost_dim_),
      terminal_cost_dim_(rhs.terminal_cost_dim_),
      intermediateADInterfacePtr_(new ad_interface_t(*rhs.intermediateADInterfacePtr_)),
      terminalADInterfacePtr_(new ad_interface_t(*rhs.terminalADInterfacePtr_)),
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
void RelaxedBarrierCost::setCurrentStateAndControl(const scalar_t& t, const vector_t& x, const vector_t& u) {
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
void RelaxedBarrierCost::getIntermediateCost(scalar_t& L) {
  L = 0;
  if (intermediate_cost_dim_ == 0) {
    return;
  }

  if (!intermediateCostValuesComputed_) {
    intermediateCostValues_ = intermediateADInterfacePtr_->getFunctionValue(tapedTimeStateInput_, intermediateParameters_);
    intermediateCostValuesComputed_ = true;
  }
  for (int i = 0; i < intermediate_cost_dim_; i++) {
    L += getPenaltyFunctionValue(intermediateCostValues_[i], intermediateConfig_[i]);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void RelaxedBarrierCost::getIntermediateCostDerivativeTime(scalar_t& dLdt) {
  if (intermediate_cost_dim_ == 0) {
    dLdt = 0;
    return;
  }

  if (!intermediateDerivativesComputed_) {
    intermediateJacobian_ = intermediateADInterfacePtr_->getJacobian(tapedTimeStateInput_, intermediateParameters_);
    intermediateDerivativesComputed_ = true;
  }
  if (!intermediateCostValuesComputed_) {
    intermediateCostValues_ = intermediateADInterfacePtr_->getFunctionValue(tapedTimeStateInput_, intermediateParameters_);
    intermediateCostValuesComputed_ = true;
  }

  vector_t penalityFctDerivative(intermediate_cost_dim_);
  for (int i = 0; i < intermediate_cost_dim_; i++) {
    penalityFctDerivative(i) = getPenaltyFunctionDerivative(intermediateCostValues_[i], intermediateConfig_[i]);
  }
  dLdt = penalityFctDerivative.dot(intermediateJacobian_.col(0));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void RelaxedBarrierCost::getIntermediateCostDerivativeState(vector_t& dLdx) {
  if (intermediate_cost_dim_ == 0) {
    dLdx = vector_t::Zero(state_dim_);
    return;
  }

  if (!intermediateDerivativesComputed_) {
    intermediateJacobian_ = intermediateADInterfacePtr_->getJacobian(tapedTimeStateInput_, intermediateParameters_);
    intermediateDerivativesComputed_ = true;
  }
  if (!intermediateCostValuesComputed_) {
    intermediateCostValues_ = intermediateADInterfacePtr_->getFunctionValue(tapedTimeStateInput_, intermediateParameters_);
    intermediateCostValuesComputed_ = true;
  }

  vector_t penalityFctDerivative(intermediate_cost_dim_);
  for (int i = 0; i < intermediate_cost_dim_; i++) {
    penalityFctDerivative(i) = getPenaltyFunctionDerivative(intermediateCostValues_[i], intermediateConfig_[i]);
  }
  dLdx = intermediateJacobian_.block(0, 1, intermediate_cost_dim_, state_dim_).transpose() * penalityFctDerivative;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void RelaxedBarrierCost::getIntermediateCostSecondDerivativeState(matrix_t& dLdxx) {
  if (intermediate_cost_dim_ == 0) {
    dLdxx = matrix_t::Zero(state_dim_, state_dim_);
    return;
  }

  if (!intermediateDerivativesComputed_) {
    intermediateJacobian_ = intermediateADInterfacePtr_->getJacobian(tapedTimeStateInput_, intermediateParameters_);
    intermediateDerivativesComputed_ = true;
  }
  if (!intermediateCostValuesComputed_) {
    intermediateCostValues_ = intermediateADInterfacePtr_->getFunctionValue(tapedTimeStateInput_, intermediateParameters_);
    intermediateCostValuesComputed_ = true;
  }

  vector_t penalityFctSecondDerivative(intermediate_cost_dim_);
  for (int i = 0; i < intermediate_cost_dim_; i++) {
    penalityFctSecondDerivative(i) = getPenaltyFunctionSecondDerivative(intermediateCostValues_[i], intermediateConfig_[i]);
  }
  dLdxx = intermediateJacobian_.block(0, 1, intermediate_cost_dim_, state_dim_).transpose() * penalityFctSecondDerivative.asDiagonal() *
          intermediateJacobian_.block(0, 1, intermediate_cost_dim_, state_dim_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void RelaxedBarrierCost::getIntermediateCostDerivativeInput(vector_t& dLdu) {
  if (intermediate_cost_dim_ == 0) {
    dLdu = vector_t::Zero(input_dim_);
    return;
  }

  if (!intermediateDerivativesComputed_) {
    intermediateJacobian_ = intermediateADInterfacePtr_->getJacobian(tapedTimeStateInput_, intermediateParameters_);
    intermediateDerivativesComputed_ = true;
  }
  if (!intermediateCostValuesComputed_) {
    intermediateCostValues_ = intermediateADInterfacePtr_->getFunctionValue(tapedTimeStateInput_, intermediateParameters_);
    intermediateCostValuesComputed_ = true;
  }

  vector_t penalityFctDerivative(intermediate_cost_dim_);
  for (int i = 0; i < intermediate_cost_dim_; i++) {
    penalityFctDerivative(i) = getPenaltyFunctionDerivative(intermediateCostValues_[i], intermediateConfig_[i]);
  }
  dLdu = intermediateJacobian_.block(0, 1 + state_dim_, intermediate_cost_dim_, input_dim_).transpose() * penalityFctDerivative;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void RelaxedBarrierCost::getIntermediateCostSecondDerivativeInput(matrix_t& dLduu) {
  if (intermediate_cost_dim_ == 0) {
    dLduu = matrix_t::Zero(input_dim_, input_dim_);
    return;
  }

  if (!intermediateDerivativesComputed_) {
    intermediateJacobian_ = intermediateADInterfacePtr_->getJacobian(tapedTimeStateInput_, intermediateParameters_);
    intermediateDerivativesComputed_ = true;
  }
  if (!intermediateCostValuesComputed_) {
    intermediateCostValues_ = intermediateADInterfacePtr_->getFunctionValue(tapedTimeStateInput_, intermediateParameters_);
    intermediateCostValuesComputed_ = true;
  }

  vector_t penalityFctSecondDerivative(intermediate_cost_dim_);
  for (int i = 0; i < intermediate_cost_dim_; i++) {
    penalityFctSecondDerivative(i) = getPenaltyFunctionSecondDerivative(intermediateCostValues_[i], intermediateConfig_[i]);
  }
  dLduu = intermediateJacobian_.block(0, 1 + state_dim_, intermediate_cost_dim_, input_dim_).transpose() *
          penalityFctSecondDerivative.asDiagonal() * intermediateJacobian_.block(0, 1 + state_dim_, intermediate_cost_dim_, input_dim_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void RelaxedBarrierCost::getIntermediateCostDerivativeInputState(matrix_t& dLdux) {
  if (intermediate_cost_dim_ == 0) {
    dLdux = matrix_t::Zero(input_dim_, state_dim_);
    return;
  }

  if (!intermediateDerivativesComputed_) {
    intermediateJacobian_ = intermediateADInterfacePtr_->getJacobian(tapedTimeStateInput_, intermediateParameters_);
    intermediateDerivativesComputed_ = true;
  }

  vector_t penalityFctSecondDerivative(intermediate_cost_dim_);
  for (int i = 0; i < intermediate_cost_dim_; i++) {
    penalityFctSecondDerivative(i) = getPenaltyFunctionSecondDerivative(intermediateCostValues_[i], intermediateConfig_[i]);
  }
  dLdux = intermediateJacobian_.block(0, 1 + state_dim_, intermediate_cost_dim_, input_dim_).transpose() *
          penalityFctSecondDerivative.asDiagonal() * intermediateJacobian_.block(0, 1, intermediate_cost_dim_, state_dim_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void RelaxedBarrierCost::getTerminalCost(scalar_t& Phi) {
  Phi = 0;
  if (terminal_cost_dim_ == 0) {
    return;
  }

  if (!terminalCostValuesComputed_) {
    terminalCostValues_ = terminalADInterfacePtr_->getFunctionValue(tapedTimeState_, terminalParameters_);
    terminalCostValuesComputed_ = true;
  }

  for (int i = 0; i < terminal_cost_dim_; i++) {
    Phi += getPenaltyFunctionValue(terminalCostValues_[i], terminalConfig_[i]);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void RelaxedBarrierCost::getTerminalCostDerivativeTime(scalar_t& dPhidt) {
  if (terminal_cost_dim_ == 0) {
    dPhidt = 0;
    return;
  }

  if (!terminalDerivativesComputed_) {
    terminalJacobian_ = terminalADInterfacePtr_->getJacobian(tapedTimeState_, terminalParameters_);
    terminalDerivativesComputed_ = true;
  }
  if (!terminalCostValuesComputed_) {
    terminalCostValues_ = terminalADInterfacePtr_->getFunctionValue(tapedTimeState_, terminalParameters_);
    terminalCostValuesComputed_ = true;
  }

  vector_t penalityFctDerivative(terminal_cost_dim_);
  for (int i = 0; i < terminal_cost_dim_; i++) {
    penalityFctDerivative(i) = getPenaltyFunctionDerivative(terminalCostValues_[i], terminalConfig_[i]);
  }
  dPhidt = penalityFctDerivative.transpose() * terminalJacobian_.col(0);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void RelaxedBarrierCost::getTerminalCostDerivativeState(vector_t& dPhidx) {
  if (terminal_cost_dim_ == 0) {
    dPhidx = vector_t::Zero(state_dim_);
    return;
  }

  if (!terminalDerivativesComputed_) {
    terminalJacobian_ = terminalADInterfacePtr_->getJacobian(tapedTimeState_, terminalParameters_);
    terminalDerivativesComputed_ = true;
  }
  if (!terminalCostValuesComputed_) {
    terminalCostValues_ = terminalADInterfacePtr_->getFunctionValue(tapedTimeState_, terminalParameters_);
    terminalCostValuesComputed_ = true;
  }

  vector_t penalityFctDerivative(terminal_cost_dim_);
  for (int i = 0; i < terminal_cost_dim_; i++) {
    penalityFctDerivative(i) = getPenaltyFunctionDerivative(terminalCostValues_[i], terminalConfig_[i]);
  }
  dPhidx = terminalJacobian_.block(0, 1, terminal_cost_dim_, state_dim_).transpose() * penalityFctDerivative;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void RelaxedBarrierCost::getTerminalCostSecondDerivativeState(matrix_t& dPhidxx) {
  if (terminal_cost_dim_ == 0) {
    dPhidxx = matrix_t::Zero(state_dim_, state_dim_);
    return;
  }

  if (!terminalDerivativesComputed_) {
    terminalJacobian_ = terminalADInterfacePtr_->getJacobian(tapedTimeState_, terminalParameters_);
    terminalDerivativesComputed_ = true;
  }
  if (!terminalCostValuesComputed_) {
    terminalCostValues_ = terminalADInterfacePtr_->getFunctionValue(tapedTimeState_, terminalParameters_);
    terminalCostValuesComputed_ = true;
  }

  vector_t penalityFctSecondDerivative(terminal_cost_dim_);
  for (int i = 0; i < terminal_cost_dim_; i++) {
    penalityFctSecondDerivative(i) = getPenaltyFunctionSecondDerivative(terminalCostValues_[i], terminalConfig_[i]);
  }
  dPhidxx = terminalJacobian_.block(0, 1, terminal_cost_dim_, state_dim_).transpose() * penalityFctSecondDerivative.asDiagonal() *
            terminalJacobian_.block(0, 1, terminal_cost_dim_, state_dim_);
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
void RelaxedBarrierCost::terminalCostFunction(ad_scalar_t time, const ad_dynamic_vector_t& state, const ad_dynamic_vector_t& parameters,
                                              ad_dynamic_vector_t& costValues) const {
  costValues = ad_dynamic_vector_t::Zero(terminal_cost_dim_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void RelaxedBarrierCost::setADInterfaces(const std::string& modelName, const std::string& modelFolder) {
  auto intermediateCostAd = [this](const ad_dynamic_vector_t& x, const ad_dynamic_vector_t& p, ad_dynamic_vector_t& y) {
    auto time = x(0);
    auto state = x.segment(1, state_dim_);
    auto input = x.tail(input_dim_);
    this->intermediateCostFunction(time, state, input, p, y);
  };
  intermediateADInterfacePtr_.reset(new ad_interface_t(intermediateCostAd, intermediate_cost_dim_, 1 + state_dim_ + input_dim_,
                                                       getNumIntermediateParameters(), modelName + "_intermediate", modelFolder));

  auto terminalCostAd = [this](const ad_dynamic_vector_t& x, const ad_dynamic_vector_t& p, ad_dynamic_vector_t& y) {
    auto time = x(0);
    auto state = x.tail(state_dim_);
    this->terminalCostFunction(time, state, p, y);
  };
  terminalADInterfacePtr_.reset(new ad_interface_t(terminalCostAd, terminal_cost_dim_, 1 + state_dim_, getNumTerminalParameters(),
                                                   modelName + "_terminal", modelFolder));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void RelaxedBarrierCost::createModels(bool verbose) {
  intermediateADInterfacePtr_->createModels(ad_interface_t::ApproximationOrder::First, verbose);
  terminalADInterfacePtr_->createModels(ad_interface_t::ApproximationOrder::First, verbose);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void RelaxedBarrierCost::loadModelsIfAvailable(bool verbose) {
  intermediateADInterfacePtr_->loadModelsIfAvailable(ad_interface_t::ApproximationOrder::First, verbose);
  terminalADInterfacePtr_->loadModelsIfAvailable(ad_interface_t::ApproximationOrder::First, verbose);
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
