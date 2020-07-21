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
RelaxedBarrierCost::RelaxedBarrierCost(Config config, size_t stateDim, size_t inputDim, size_t intermediateCostDim, size_t finalCostDim)
    : RelaxedBarrierCost(std::vector<Config>(intermediateCostDim, config), std::vector<Config>(finalCostDim, config), stateDim, inputDim) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
RelaxedBarrierCost::RelaxedBarrierCost(std::vector<Config> intermediateConfig, std::vector<Config> finalConfig, size_t stateDim,
                                       size_t inputDim)
    : CostFunctionBase(),
      stateDim_(stateDim),
      inputDim_(inputDim),
      intermediateConfig_(std::move(intermediateConfig)),
      finalConfig_(std::move(finalConfig)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
RelaxedBarrierCost::RelaxedBarrierCost(const RelaxedBarrierCost& rhs)
    : CostFunctionBase(rhs),
      stateDim_(rhs.stateDim_),
      inputDim_(rhs.inputDim_),
      intermediateADInterfacePtr_(new CppAdInterface(*rhs.intermediateADInterfacePtr_)),
      finalADInterfacePtr_(new CppAdInterface(*rhs.finalADInterfacePtr_)),
      intermediateConfig_(rhs.intermediateConfig_),
      finalConfig_(rhs.finalConfig_) {}

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
scalar_t RelaxedBarrierCost::cost(scalar_t t, const vector_t& x, const vector_t& u) {
  if (intermediateConfig_.size() == 0) {
    return 0.0;
  }

  tapedTimeStateInput_.resize(1 + stateDim_ + inputDim_);
  tapedTimeStateInput_ << t, x, u;
  intermediateParameters_ = getIntermediateParameters(t);
  intermediateCostValues_ = intermediateADInterfacePtr_->getFunctionValue(tapedTimeStateInput_, intermediateParameters_);
  scalar_t L = 0;
  for (int i = 0; i < intermediateConfig_.size(); i++) {
    L += getPenaltyFunctionValue(intermediateCostValues_[i], intermediateConfig_[i]);
  }
  return L;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t RelaxedBarrierCost::finalCost(scalar_t t, const vector_t& x) {
  if (finalConfig_.size() == 0) {
    return 0.0;
  }

  tapedTimeState_.resize(1 + stateDim_);
  tapedTimeState_ << t, x;
  finalParameters_ = getFinalParameters(t);
  finalCostValues_ = finalADInterfacePtr_->getFunctionValue(tapedTimeState_, finalParameters_);
  scalar_t Phi = 0;
  for (int i = 0; i < finalConfig_.size(); i++) {
    Phi += getPenaltyFunctionValue(finalCostValues_[i], finalConfig_[i]);
  }
  return Phi;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation RelaxedBarrierCost::costQuadraticApproximation(scalar_t t, const vector_t& x, const vector_t& u) {
  ScalarFunctionQuadraticApproximation L;
  if (intermediateConfig_.size() == 0) {
    L.dfdxx.setZero(stateDim_, stateDim_);
    L.dfdux.setZero(inputDim_, stateDim_);
    L.dfduu.setZero(inputDim_, inputDim_);
    L.dfdx.setZero(stateDim_);
    L.dfdu.setZero(inputDim_);
    L.f = 0.0;
    return L;
  }

  tapedTimeStateInput_.resize(1 + stateDim_ + inputDim_);
  tapedTimeStateInput_ << t, x, u;
  intermediateParameters_ = getIntermediateParameters(t);
  intermediateJacobian_ = intermediateADInterfacePtr_->getJacobian(tapedTimeStateInput_, intermediateParameters_);
  intermediateCostValues_ = intermediateADInterfacePtr_->getFunctionValue(tapedTimeStateInput_, intermediateParameters_);
  vector_t penalityFctDerivative(intermediateConfig_.size());
  vector_t penalityFctSecondDerivative(intermediateConfig_.size());
  scalar_t cost = 0.0;
  for (int i = 0; i < intermediateConfig_.size(); i++) {
    cost += getPenaltyFunctionValue(intermediateCostValues_[i], intermediateConfig_[i]);
    penalityFctDerivative(i) = getPenaltyFunctionDerivative(intermediateCostValues_[i], intermediateConfig_[i]);
    penalityFctSecondDerivative(i) = getPenaltyFunctionSecondDerivative(intermediateCostValues_[i], intermediateConfig_[i]);
  }

  L.dfdxx = intermediateJacobian_.block(0, 1, intermediateConfig_.size(), stateDim_).transpose() *
            penalityFctSecondDerivative.asDiagonal() * intermediateJacobian_.block(0, 1, intermediateConfig_.size(), stateDim_);
  L.dfdux = intermediateJacobian_.block(0, 1 + stateDim_, intermediateConfig_.size(), inputDim_).transpose() *
            penalityFctSecondDerivative.asDiagonal() * intermediateJacobian_.block(0, 1, intermediateConfig_.size(), stateDim_);
  L.dfduu = intermediateJacobian_.block(0, 1 + stateDim_, intermediateConfig_.size(), inputDim_).transpose() *
            penalityFctSecondDerivative.asDiagonal() * intermediateJacobian_.block(0, 1 + stateDim_, intermediateConfig_.size(), inputDim_);
  L.dfdx = intermediateJacobian_.block(0, 1, intermediateConfig_.size(), stateDim_).transpose() * penalityFctDerivative;
  L.dfdu = intermediateJacobian_.block(0, 1 + stateDim_, intermediateConfig_.size(), inputDim_).transpose() * penalityFctDerivative;
  L.f = cost;
  return L;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation RelaxedBarrierCost::finalCostQuadraticApproximation(scalar_t t, const vector_t& x) {
  ScalarFunctionQuadraticApproximation Phi;
  if (finalConfig_.size() == 0) {
    Phi.dfdxx.setZero(stateDim_, stateDim_);
    Phi.dfdx.setZero(stateDim_);
    Phi.f = 0.0;
    return Phi;
  }

  tapedTimeState_.resize(1 + stateDim_);
  tapedTimeState_ << t, x;
  finalParameters_ = getFinalParameters(t);
  finalJacobian_ = finalADInterfacePtr_->getJacobian(tapedTimeState_, finalParameters_);
  finalCostValues_ = finalADInterfacePtr_->getFunctionValue(tapedTimeState_, finalParameters_);
  vector_t penalityFctSecondDerivative(finalConfig_.size());
  vector_t penalityFctDerivative(finalConfig_.size());
  scalar_t cost = 0;
  for (int i = 0; i < finalConfig_.size(); i++) {
    cost += getPenaltyFunctionValue(finalCostValues_[i], finalConfig_[i]);
    penalityFctDerivative(i) = getPenaltyFunctionDerivative(finalCostValues_[i], finalConfig_[i]);
    penalityFctSecondDerivative(i) = getPenaltyFunctionSecondDerivative(finalCostValues_[i], finalConfig_[i]);
  }

  Phi.dfdxx = finalJacobian_.block(0, 1, finalConfig_.size(), stateDim_).transpose() * penalityFctSecondDerivative.asDiagonal() *
              finalJacobian_.block(0, 1, finalConfig_.size(), stateDim_);
  Phi.dfdx = finalJacobian_.block(0, 1, finalConfig_.size(), stateDim_).transpose() * penalityFctDerivative;
  Phi.f = cost;
  return Phi;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t RelaxedBarrierCost::costDerivativeTime(scalar_t t, const vector_t& x, const vector_t& u) {
  if (intermediateConfig_.size() == 0) {
    return 0.0;
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
scalar_t RelaxedBarrierCost::finalCostDerivativeTime(scalar_t t, const vector_t& x) {
  if (finalConfig_.size() == 0) {
    return 0.0;
  }

  vector_t penalityFctDerivative(finalConfig_.size());
  for (int i = 0; i < finalConfig_.size(); i++) {
    penalityFctDerivative(i) = getPenaltyFunctionDerivative(finalCostValues_[i], finalConfig_[i]);
  }
  return penalityFctDerivative.transpose() * finalJacobian_.col(0);
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
vector_t RelaxedBarrierCost::getFinalParameters(scalar_t time) const {
  return vector_t(0);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
size_t RelaxedBarrierCost::getNumFinalParameters() const {
  return 0;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
RelaxedBarrierCost::ad_vector_t RelaxedBarrierCost::finalCostFunction(ad_scalar_t time, const ad_vector_t& state,
                                                                      const ad_vector_t& parameters) const {
  return ad_vector_t::Zero(finalConfig_.size());
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
void RelaxedBarrierCost::createModels(bool verbose) {
  intermediateADInterfacePtr_->createModels(CppAdInterface::ApproximationOrder::First, verbose);
  finalADInterfacePtr_->createModels(CppAdInterface::ApproximationOrder::First, verbose);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void RelaxedBarrierCost::loadModelsIfAvailable(bool verbose) {
  intermediateADInterfacePtr_->loadModelsIfAvailable(CppAdInterface::ApproximationOrder::First, verbose);
  finalADInterfacePtr_->loadModelsIfAvailable(CppAdInterface::ApproximationOrder::First, verbose);
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
