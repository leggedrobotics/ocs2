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
template <size_t STATE_DIM, size_t INPUT_DIM>
CostFunctionBaseAD<STATE_DIM, INPUT_DIM>::CostFunctionBaseAD()
    : BASE(), intermediateDerivativesComputed_(false), terminalDerivativesComputed_(false) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
CostFunctionBaseAD<STATE_DIM, INPUT_DIM>::CostFunctionBaseAD(const CostFunctionBaseAD& rhs)
    : BASE(rhs),
      intermediateADInterfacePtr_(new ad_interface_t(*rhs.intermediateADInterfacePtr_)),
      terminalADInterfacePtr_(new ad_interface_t(*rhs.terminalADInterfacePtr_)),
      intermediateDerivativesComputed_(false),
      terminalDerivativesComputed_(false) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void CostFunctionBaseAD<STATE_DIM, INPUT_DIM>::initialize(const std::string& modelName, const std::string& modelFolder,
                                                          bool recompileLibraries, bool verbose) {
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
template <size_t STATE_DIM, size_t INPUT_DIM>
void CostFunctionBaseAD<STATE_DIM, INPUT_DIM>::setCurrentStateAndControl(const scalar_t& t, const state_vector_t& x,
                                                                         const input_vector_t& u) {
  BASE::setCurrentStateAndControl(t, x, u);

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
template <size_t STATE_DIM, size_t INPUT_DIM>
void CostFunctionBaseAD<STATE_DIM, INPUT_DIM>::getIntermediateCost(scalar_t& L) {
  auto costValue = intermediateADInterfacePtr_->getFunctionValue(tapedTimeStateInput_, intermediateParameters_);
  L = costValue(0);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void CostFunctionBaseAD<STATE_DIM, INPUT_DIM>::getIntermediateCostDerivativeTime(scalar_t& dLdt) {
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
template <size_t STATE_DIM, size_t INPUT_DIM>
void CostFunctionBaseAD<STATE_DIM, INPUT_DIM>::getIntermediateCostDerivativeState(state_vector_t& dLdx) {
  if (!intermediateDerivativesComputed_) {
    intermediateJacobian_ = intermediateADInterfacePtr_->getJacobian(tapedTimeStateInput_, intermediateParameters_);
    intermediateHessian_ = intermediateADInterfacePtr_->getHessian(0, tapedTimeStateInput_, intermediateParameters_);
    intermediateDerivativesComputed_ = true;
  }
  dLdx = intermediateJacobian_.template segment<STATE_DIM>(1).transpose();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void CostFunctionBaseAD<STATE_DIM, INPUT_DIM>::getIntermediateCostSecondDerivativeState(state_matrix_t& dLdxx) {
  if (!intermediateDerivativesComputed_) {
    intermediateJacobian_ = intermediateADInterfacePtr_->getJacobian(tapedTimeStateInput_, intermediateParameters_);
    intermediateHessian_ = intermediateADInterfacePtr_->getHessian(0, tapedTimeStateInput_, intermediateParameters_);
    intermediateDerivativesComputed_ = true;
  }
  dLdxx = intermediateHessian_.template block<STATE_DIM, STATE_DIM>(1, 1);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void CostFunctionBaseAD<STATE_DIM, INPUT_DIM>::getIntermediateCostDerivativeInput(input_vector_t& dLdu) {
  if (!intermediateDerivativesComputed_) {
    intermediateJacobian_ = intermediateADInterfacePtr_->getJacobian(tapedTimeStateInput_, intermediateParameters_);
    intermediateHessian_ = intermediateADInterfacePtr_->getHessian(0, tapedTimeStateInput_, intermediateParameters_);
    intermediateDerivativesComputed_ = true;
  }
  dLdu = intermediateJacobian_.template segment<INPUT_DIM>(1 + STATE_DIM).transpose();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void CostFunctionBaseAD<STATE_DIM, INPUT_DIM>::getIntermediateCostSecondDerivativeInput(input_matrix_t& dLduu) {
  if (!intermediateDerivativesComputed_) {
    intermediateJacobian_ = intermediateADInterfacePtr_->getJacobian(tapedTimeStateInput_, intermediateParameters_);
    intermediateHessian_ = intermediateADInterfacePtr_->getHessian(0, tapedTimeStateInput_, intermediateParameters_);
    intermediateDerivativesComputed_ = true;
  }
  dLduu = intermediateHessian_.template block<INPUT_DIM, INPUT_DIM>(1 + STATE_DIM, 1 + STATE_DIM);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void CostFunctionBaseAD<STATE_DIM, INPUT_DIM>::getIntermediateCostDerivativeInputState(input_state_matrix_t& dLdux) {
  if (!intermediateDerivativesComputed_) {
    intermediateJacobian_ = intermediateADInterfacePtr_->getJacobian(tapedTimeStateInput_, intermediateParameters_);
    intermediateHessian_ = intermediateADInterfacePtr_->getHessian(0, tapedTimeStateInput_, intermediateParameters_);
    intermediateDerivativesComputed_ = true;
  }
  dLdux = intermediateHessian_.template block<INPUT_DIM, STATE_DIM>(1 + STATE_DIM, 1);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void CostFunctionBaseAD<STATE_DIM, INPUT_DIM>::getTerminalCost(scalar_t& Phi) {
  auto costValue = terminalADInterfacePtr_->getFunctionValue(tapedTimeState_, terminalParameters_);
  Phi = costValue(0);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void CostFunctionBaseAD<STATE_DIM, INPUT_DIM>::getTerminalCostDerivativeTime(scalar_t& dPhidt) {
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
template <size_t STATE_DIM, size_t INPUT_DIM>
void CostFunctionBaseAD<STATE_DIM, INPUT_DIM>::getTerminalCostDerivativeState(state_vector_t& dPhidx) {
  if (!terminalDerivativesComputed_) {
    terminalJacobian_ = terminalADInterfacePtr_->getJacobian(tapedTimeState_, terminalParameters_);
    terminalHessian_ = terminalADInterfacePtr_->getHessian(0, tapedTimeState_, terminalParameters_);
    terminalDerivativesComputed_ = true;
  }
  dPhidx = terminalJacobian_.template segment<STATE_DIM>(1).transpose();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void CostFunctionBaseAD<STATE_DIM, INPUT_DIM>::getTerminalCostSecondDerivativeState(state_matrix_t& dPhidxx) {
  if (!terminalDerivativesComputed_) {
    terminalJacobian_ = terminalADInterfacePtr_->getJacobian(tapedTimeState_, terminalParameters_);
    terminalHessian_ = terminalADInterfacePtr_->getHessian(0, tapedTimeState_, terminalParameters_);
    terminalDerivativesComputed_ = true;
  }
  dPhidxx = terminalHessian_.template block<STATE_DIM, STATE_DIM>(1, 1);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void CostFunctionBaseAD<STATE_DIM, INPUT_DIM>::setADInterfaces(const std::string& modelName, const std::string& modelFolder) {
  auto intermediateCostAd = [this](const ad_dynamic_vector_t& x, const ad_dynamic_vector_t& p, ad_dynamic_vector_t& y) {
    auto time = x(0);
    auto state = x.template segment<STATE_DIM>(1);
    auto input = x.template segment<INPUT_DIM>(1 + STATE_DIM);
    y = ad_dynamic_vector_t(1);
    this->intermediateCostFunction(time, state, input, p, y(0));
  };
  intermediateADInterfacePtr_.reset(new ad_interface_t(intermediateCostAd, 1, 1 + STATE_DIM + INPUT_DIM, getNumIntermediateParameters(),
                                                       modelName + "_intermediate", modelFolder));

  auto terminalCostAd = [this](const ad_dynamic_vector_t& x, const ad_dynamic_vector_t& p, ad_dynamic_vector_t& y) {
    auto time = x(0);
    auto state = x.template segment<STATE_DIM>(1);
    y = ad_dynamic_vector_t(1);
    this->terminalCostFunction(time, state, p, y(0));
  };
  terminalADInterfacePtr_.reset(
      new ad_interface_t(terminalCostAd, 1, 1 + STATE_DIM, getNumTerminalParameters(), modelName + "_terminal", modelFolder));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void CostFunctionBaseAD<STATE_DIM, INPUT_DIM>::createModels(bool verbose) {
  intermediateADInterfacePtr_->createModels(ad_interface_t::ApproximationOrder::Second, verbose);
  terminalADInterfacePtr_->createModels(ad_interface_t::ApproximationOrder::Second, verbose);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void CostFunctionBaseAD<STATE_DIM, INPUT_DIM>::loadModelsIfAvailable(bool verbose) {
  intermediateADInterfacePtr_->loadModelsIfAvailable(ad_interface_t::ApproximationOrder::Second, verbose);
  terminalADInterfacePtr_->loadModelsIfAvailable(ad_interface_t::ApproximationOrder::Second, verbose);
}

}  // namespace ocs2
