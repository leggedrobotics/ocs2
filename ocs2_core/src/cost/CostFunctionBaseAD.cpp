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
CostFunctionBaseAD::CostFunctionBaseAD(size_t state_dim, size_t input_dim, size_t intermediate_cost_dim, size_t terminal_cost_dim)
    : CostFunctionBase(),
      state_dim_(state_dim),
      input_dim_(input_dim),
      intermediate_cost_dim_(intermediate_cost_dim),
      terminal_cost_dim_(terminal_cost_dim),
      intermediateDerivativesComputed_(false),
      terminalDerivativesComputed_(false) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
CostFunctionBaseAD::CostFunctionBaseAD(const CostFunctionBaseAD& rhs)
    : CostFunctionBase(rhs),
      intermediateADInterfacePtr_(new ad_interface_t(*rhs.intermediateADInterfacePtr_)),
      terminalADInterfacePtr_(new ad_interface_t(*rhs.terminalADInterfacePtr_)),
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
void CostFunctionBaseAD::setCurrentStateAndControl(const scalar_t& t, const vector_t& x, const vector_t& u) {
  CostFunctionBase::setCurrentStateAndControl(t, x, u);

  tapedTimeState_ << t, x;
  tapedTimeStateInput_ << t, x, u;

  intermediateParameters_ = getIntermediateParameters(t);
  terminalParameters_ = getTerminalParameters(t);

  intermediateDerivativesComputed_ = false;
  terminalDerivativesComputed_ = false;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void CostFunctionBaseAD::getIntermediateCost(scalar_t& L) {
  auto costValue = intermediateADInterfacePtr_->getFunctionValue(tapedTimeStateInput_, intermediateParameters_);
  L = costValue(0);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void CostFunctionBaseAD::getIntermediateCostDerivativeTime(scalar_t& dLdt) {
  if (!intermediateDerivativesComputed_) {
    intermediateJacobian_ = intermediateADInterfacePtr_->getJacobian(tapedTimeStateInput_, intermediateParameters_);
    intermediateHessian_ = intermediateADInterfacePtr_->getHessian(0, tapedTimeStateInput_, intermediateParameters_);
    intermediateDerivativesComputed_ = true;
  }
  dLdt = intermediateJacobian_(0);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void CostFunctionBaseAD::getIntermediateCostDerivativeState(vector_t& dLdx) {
  if (!intermediateDerivativesComputed_) {
    intermediateJacobian_ = intermediateADInterfacePtr_->getJacobian(tapedTimeStateInput_, intermediateParameters_);
    intermediateHessian_ = intermediateADInterfacePtr_->getHessian(0, tapedTimeStateInput_, intermediateParameters_);
    intermediateDerivativesComputed_ = true;
  }
  dLdx = intermediateJacobian_.segment(1, state_dim_).transpose();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void CostFunctionBaseAD::getIntermediateCostSecondDerivativeState(matrix_t& dLdxx) {
  if (!intermediateDerivativesComputed_) {
    intermediateJacobian_ = intermediateADInterfacePtr_->getJacobian(tapedTimeStateInput_, intermediateParameters_);
    intermediateHessian_ = intermediateADInterfacePtr_->getHessian(0, tapedTimeStateInput_, intermediateParameters_);
    intermediateDerivativesComputed_ = true;
  }
  dLdxx = intermediateHessian_.block(1, 1, state_dim_, state_dim_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void CostFunctionBaseAD::getIntermediateCostDerivativeInput(vector_t& dLdu) {
  if (!intermediateDerivativesComputed_) {
    intermediateJacobian_ = intermediateADInterfacePtr_->getJacobian(tapedTimeStateInput_, intermediateParameters_);
    intermediateHessian_ = intermediateADInterfacePtr_->getHessian(0, tapedTimeStateInput_, intermediateParameters_);
    intermediateDerivativesComputed_ = true;
  }
  dLdu = intermediateJacobian_.segment(1 + state_dim_, input_dim_).transpose();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void CostFunctionBaseAD::getIntermediateCostSecondDerivativeInput(matrix_t& dLduu) {
  if (!intermediateDerivativesComputed_) {
    intermediateJacobian_ = intermediateADInterfacePtr_->getJacobian(tapedTimeStateInput_, intermediateParameters_);
    intermediateHessian_ = intermediateADInterfacePtr_->getHessian(0, tapedTimeStateInput_, intermediateParameters_);
    intermediateDerivativesComputed_ = true;
  }
  dLduu = intermediateHessian_.block(1 + state_dim_, 1 + state_dim_, input_dim_, input_dim_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void CostFunctionBaseAD::getIntermediateCostDerivativeInputState(matrix_t& dLdux) {
  if (!intermediateDerivativesComputed_) {
    intermediateJacobian_ = intermediateADInterfacePtr_->getJacobian(tapedTimeStateInput_, intermediateParameters_);
    intermediateHessian_ = intermediateADInterfacePtr_->getHessian(0, tapedTimeStateInput_, intermediateParameters_);
    intermediateDerivativesComputed_ = true;
  }
  dLdux = intermediateHessian_.block(1 + state_dim_, 1, input_dim_, state_dim_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void CostFunctionBaseAD::getTerminalCost(scalar_t& Phi) {
  auto costValue = terminalADInterfacePtr_->getFunctionValue(tapedTimeState_, terminalParameters_);
  Phi = costValue(0);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void CostFunctionBaseAD::getTerminalCostDerivativeTime(scalar_t& dPhidt) {
  if (!terminalDerivativesComputed_) {
    terminalJacobian_ = terminalADInterfacePtr_->getJacobian(tapedTimeState_, terminalParameters_);
    terminalHessian_ = terminalADInterfacePtr_->getHessian(0, tapedTimeState_, terminalParameters_);
    terminalDerivativesComputed_ = true;
  }
  dPhidt = terminalJacobian_(0);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void CostFunctionBaseAD::getTerminalCostDerivativeState(vector_t& dPhidx) {
  if (!terminalDerivativesComputed_) {
    terminalJacobian_ = terminalADInterfacePtr_->getJacobian(tapedTimeState_, terminalParameters_);
    terminalHessian_ = terminalADInterfacePtr_->getHessian(0, tapedTimeState_, terminalParameters_);
    terminalDerivativesComputed_ = true;
  }
  dPhidx = terminalJacobian_.segment(1, state_dim_).transpose();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void CostFunctionBaseAD::getTerminalCostSecondDerivativeState(matrix_t& dPhidxx) {
  if (!terminalDerivativesComputed_) {
    terminalJacobian_ = terminalADInterfacePtr_->getJacobian(tapedTimeState_, terminalParameters_);
    terminalHessian_ = terminalADInterfacePtr_->getHessian(0, tapedTimeState_, terminalParameters_);
    terminalDerivativesComputed_ = true;
  }
  dPhidxx = terminalHessian_.block(1, 1, state_dim_, state_dim_);
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
void CostFunctionBaseAD::terminalCostFunction(ad_scalar_t time, const ad_dynamic_vector_t& state, const ad_dynamic_vector_t& parameters,
                                              ad_scalar_t& costValue) const {
  costValue = 0;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void CostFunctionBaseAD::setADInterfaces(const std::string& modelName, const std::string& modelFolder) {
  auto intermediateCostAd = [this](const ad_dynamic_vector_t& x, const ad_dynamic_vector_t& p, ad_dynamic_vector_t& y) {
    auto time = x(0);
    auto state = x.segment(1, state_dim_);
    auto input = x.segment(1 + state_dim_, input_dim_);
    y = ad_dynamic_vector_t(1);
    this->intermediateCostFunction(time, state, input, p, y(0));
  };
  intermediateADInterfacePtr_.reset(new ad_interface_t(intermediateCostAd, 1, 1 + state_dim_ + input_dim_, getNumIntermediateParameters(),
                                                       modelName + "_intermediate", modelFolder));

  auto terminalCostAd = [this](const ad_dynamic_vector_t& x, const ad_dynamic_vector_t& p, ad_dynamic_vector_t& y) {
    auto time = x(0);
    auto state = x.segment(1, state_dim_);
    y = ad_dynamic_vector_t(1);
    this->terminalCostFunction(time, state, p, y(0));
  };
  terminalADInterfacePtr_.reset(
      new ad_interface_t(terminalCostAd, 1, 1 + state_dim_, getNumTerminalParameters(), modelName + "_terminal", modelFolder));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void CostFunctionBaseAD::createModels(bool verbose) {
  intermediateADInterfacePtr_->createModels(ad_interface_t::ApproximationOrder::Second, verbose);
  terminalADInterfacePtr_->createModels(ad_interface_t::ApproximationOrder::Second, verbose);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void CostFunctionBaseAD::loadModelsIfAvailable(bool verbose) {
  intermediateADInterfacePtr_->loadModelsIfAvailable(ad_interface_t::ApproximationOrder::Second, verbose);
  terminalADInterfacePtr_->loadModelsIfAvailable(ad_interface_t::ApproximationOrder::Second, verbose);
}

}  // namespace ocs2
