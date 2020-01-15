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
template <size_t STATE_DIM, size_t INPUT_DIM, size_t INTERMEDIATE_COST_DIM, size_t TERMINAL_COST_DIM>
QuadraticGaussNewtonCostBaseAD<STATE_DIM, INPUT_DIM, INTERMEDIATE_COST_DIM, TERMINAL_COST_DIM>::QuadraticGaussNewtonCostBaseAD()
    : BASE(),
      intermediateDerivativesComputed_(false),
      intermediateCostValuesComputed_(false),
      terminalDerivativesComputed_(false),
      terminalCostValuesComputed_(false) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, size_t INTERMEDIATE_COST_DIM, size_t TERMINAL_COST_DIM>
QuadraticGaussNewtonCostBaseAD<STATE_DIM, INPUT_DIM, INTERMEDIATE_COST_DIM, TERMINAL_COST_DIM>::QuadraticGaussNewtonCostBaseAD(
    const QuadraticGaussNewtonCostBaseAD& rhs)
    : BASE(rhs),
      intermediateADInterfacePtr_(new ad_interface_t(*rhs.intermediateADInterfacePtr_)),
      terminalADInterfacePtr_(new ad_interface_t(*rhs.terminalADInterfacePtr_)),
      intermediateDerivativesComputed_(false),
      intermediateCostValuesComputed_(false),
      terminalDerivativesComputed_(false),
      terminalCostValuesComputed_(false) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, size_t INTERMEDIATE_COST_DIM, size_t TERMINAL_COST_DIM>
void QuadraticGaussNewtonCostBaseAD<STATE_DIM, INPUT_DIM, INTERMEDIATE_COST_DIM, TERMINAL_COST_DIM>::initialize(
    const std::string& modelName, const std::string& modelFolder, bool recompileLibraries, bool verbose) {
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
template <size_t STATE_DIM, size_t INPUT_DIM, size_t INTERMEDIATE_COST_DIM, size_t TERMINAL_COST_DIM>
void QuadraticGaussNewtonCostBaseAD<STATE_DIM, INPUT_DIM, INTERMEDIATE_COST_DIM, TERMINAL_COST_DIM>::setCurrentStateAndControl(
    const scalar_t& t, const state_vector_t& x, const input_vector_t& u) {
  BASE::setCurrentStateAndControl(t, x, u);

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
template <size_t STATE_DIM, size_t INPUT_DIM, size_t INTERMEDIATE_COST_DIM, size_t TERMINAL_COST_DIM>
void QuadraticGaussNewtonCostBaseAD<STATE_DIM, INPUT_DIM, INTERMEDIATE_COST_DIM, TERMINAL_COST_DIM>::getIntermediateCost(scalar_t& L) {
  if (!intermediateCostValuesComputed_) {
    intermediateCostValues_ = intermediateADInterfacePtr_->getFunctionValue(tapedTimeStateInput_, intermediateParameters_);
    intermediateCostValuesComputed_ = true;
  }
  L = 0.5 * intermediateCostValues_.dot(intermediateCostValues_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, size_t INTERMEDIATE_COST_DIM, size_t TERMINAL_COST_DIM>
void QuadraticGaussNewtonCostBaseAD<STATE_DIM, INPUT_DIM, INTERMEDIATE_COST_DIM, TERMINAL_COST_DIM>::getIntermediateCostDerivativeTime(
    scalar_t& dLdt) {
  if (!intermediateDerivativesComputed_) {
    intermediateJacobian_ = intermediateADInterfacePtr_->getJacobian(tapedTimeStateInput_, intermediateParameters_);
    intermediateDerivativesComputed_ = true;
  }
  if (!intermediateCostValuesComputed_) {
    intermediateCostValues_ = intermediateADInterfacePtr_->getFunctionValue(tapedTimeStateInput_, intermediateParameters_);
    intermediateCostValuesComputed_ = true;
  }
  dLdt = intermediateCostValues_.transpose() * intermediateJacobian_.col(0);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, size_t INTERMEDIATE_COST_DIM, size_t TERMINAL_COST_DIM>
void QuadraticGaussNewtonCostBaseAD<STATE_DIM, INPUT_DIM, INTERMEDIATE_COST_DIM, TERMINAL_COST_DIM>::getIntermediateCostDerivativeState(
    state_vector_t& dLdx) {
  if (!intermediateDerivativesComputed_) {
    intermediateJacobian_ = intermediateADInterfacePtr_->getJacobian(tapedTimeStateInput_, intermediateParameters_);
    intermediateDerivativesComputed_ = true;
  }
  if (!intermediateCostValuesComputed_) {
    intermediateCostValues_ = intermediateADInterfacePtr_->getFunctionValue(tapedTimeStateInput_, intermediateParameters_);
    intermediateCostValuesComputed_ = true;
  }
  dLdx = intermediateJacobian_.template block<INTERMEDIATE_COST_DIM, STATE_DIM>(0, 1).transpose() * intermediateCostValues_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, size_t INTERMEDIATE_COST_DIM, size_t TERMINAL_COST_DIM>
void QuadraticGaussNewtonCostBaseAD<STATE_DIM, INPUT_DIM, INTERMEDIATE_COST_DIM,
                                    TERMINAL_COST_DIM>::getIntermediateCostSecondDerivativeState(state_matrix_t& dLdxx) {
  if (!intermediateDerivativesComputed_) {
    intermediateJacobian_ = intermediateADInterfacePtr_->getJacobian(tapedTimeStateInput_, intermediateParameters_);
    intermediateDerivativesComputed_ = true;
  }
  if (!intermediateCostValuesComputed_) {
    intermediateCostValues_ = intermediateADInterfacePtr_->getFunctionValue(tapedTimeStateInput_, intermediateParameters_);
    intermediateCostValuesComputed_ = true;
  }
  dLdxx = intermediateJacobian_.template block<INTERMEDIATE_COST_DIM, STATE_DIM>(0, 1).transpose() *
          intermediateJacobian_.template block<INTERMEDIATE_COST_DIM, STATE_DIM>(0, 1);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, size_t INTERMEDIATE_COST_DIM, size_t TERMINAL_COST_DIM>
void QuadraticGaussNewtonCostBaseAD<STATE_DIM, INPUT_DIM, INTERMEDIATE_COST_DIM, TERMINAL_COST_DIM>::getIntermediateCostDerivativeInput(
    input_vector_t& dLdu) {
  if (!intermediateDerivativesComputed_) {
    intermediateJacobian_ = intermediateADInterfacePtr_->getJacobian(tapedTimeStateInput_, intermediateParameters_);
    intermediateDerivativesComputed_ = true;
  }
  if (!intermediateCostValuesComputed_) {
    intermediateCostValues_ = intermediateADInterfacePtr_->getFunctionValue(tapedTimeStateInput_, intermediateParameters_);
    intermediateCostValuesComputed_ = true;
  }
  dLdu = intermediateJacobian_.template block<INTERMEDIATE_COST_DIM, INPUT_DIM>(0, 1 + STATE_DIM).transpose() * intermediateCostValues_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, size_t INTERMEDIATE_COST_DIM, size_t TERMINAL_COST_DIM>
void QuadraticGaussNewtonCostBaseAD<STATE_DIM, INPUT_DIM, INTERMEDIATE_COST_DIM,
                                    TERMINAL_COST_DIM>::getIntermediateCostSecondDerivativeInput(input_matrix_t& dLduu) {
  if (!intermediateDerivativesComputed_) {
    intermediateJacobian_ = intermediateADInterfacePtr_->getJacobian(tapedTimeStateInput_, intermediateParameters_);
    intermediateDerivativesComputed_ = true;
  }
  if (!intermediateCostValuesComputed_) {
    intermediateCostValues_ = intermediateADInterfacePtr_->getFunctionValue(tapedTimeStateInput_, intermediateParameters_);
    intermediateCostValuesComputed_ = true;
  }
  dLduu = intermediateJacobian_.template block<INTERMEDIATE_COST_DIM, INPUT_DIM>(0, 1 + STATE_DIM).transpose() *
          intermediateJacobian_.template block<INTERMEDIATE_COST_DIM, INPUT_DIM>(0, 1 + STATE_DIM);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, size_t INTERMEDIATE_COST_DIM, size_t TERMINAL_COST_DIM>
void QuadraticGaussNewtonCostBaseAD<STATE_DIM, INPUT_DIM, INTERMEDIATE_COST_DIM,
                                    TERMINAL_COST_DIM>::getIntermediateCostDerivativeInputState(input_state_matrix_t& dLdux) {
  if (!intermediateDerivativesComputed_) {
    intermediateJacobian_ = intermediateADInterfacePtr_->getJacobian(tapedTimeStateInput_, intermediateParameters_);
    intermediateDerivativesComputed_ = true;
  }
  dLdux = intermediateJacobian_.template block<INTERMEDIATE_COST_DIM, INPUT_DIM>(0, 1 + STATE_DIM).transpose() *
          intermediateJacobian_.template block<INTERMEDIATE_COST_DIM, STATE_DIM>(0, 1);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, size_t INTERMEDIATE_COST_DIM, size_t TERMINAL_COST_DIM>
void QuadraticGaussNewtonCostBaseAD<STATE_DIM, INPUT_DIM, INTERMEDIATE_COST_DIM, TERMINAL_COST_DIM>::getTerminalCost(scalar_t& Phi) {
  if (terminalCostValuesComputed_) {
    terminalCostValues_ = terminalADInterfacePtr_->getFunctionValue(tapedTimeState_, terminalParameters_);
    terminalCostValuesComputed_ = true;
  }
  Phi = 0.5 * terminalCostValues_.dot(terminalCostValues_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, size_t INTERMEDIATE_COST_DIM, size_t TERMINAL_COST_DIM>
void QuadraticGaussNewtonCostBaseAD<STATE_DIM, INPUT_DIM, INTERMEDIATE_COST_DIM, TERMINAL_COST_DIM>::getTerminalCostDerivativeTime(
    scalar_t& dPhidt) {
  if (!terminalDerivativesComputed_) {
    terminalJacobian_ = terminalADInterfacePtr_->getJacobian(tapedTimeState_, terminalParameters_);
    terminalDerivativesComputed_ = true;
  }
  if (terminalCostValuesComputed_) {
    terminalCostValues_ = terminalADInterfacePtr_->getFunctionValue(tapedTimeState_, terminalParameters_);
    terminalCostValuesComputed_ = true;
  }
  dPhidt = terminalCostValues_.transpose() * terminalJacobian_.col(0);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, size_t INTERMEDIATE_COST_DIM, size_t TERMINAL_COST_DIM>
void QuadraticGaussNewtonCostBaseAD<STATE_DIM, INPUT_DIM, INTERMEDIATE_COST_DIM, TERMINAL_COST_DIM>::getTerminalCostDerivativeState(
    state_vector_t& dPhidx) {
  if (!terminalDerivativesComputed_) {
    terminalJacobian_ = terminalADInterfacePtr_->getJacobian(tapedTimeState_, terminalParameters_);
    terminalDerivativesComputed_ = true;
  }
  if (terminalCostValuesComputed_) {
    terminalCostValues_ = terminalADInterfacePtr_->getFunctionValue(tapedTimeState_, terminalParameters_);
    terminalCostValuesComputed_ = true;
  }
  dPhidx = terminalJacobian_.template block<TERMINAL_COST_DIM, STATE_DIM>(0, 1).transpose() * terminalCostValues_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, size_t INTERMEDIATE_COST_DIM, size_t TERMINAL_COST_DIM>
void QuadraticGaussNewtonCostBaseAD<STATE_DIM, INPUT_DIM, INTERMEDIATE_COST_DIM, TERMINAL_COST_DIM>::getTerminalCostSecondDerivativeState(
    state_matrix_t& dPhidxx) {
  if (!terminalDerivativesComputed_) {
    terminalJacobian_ = terminalADInterfacePtr_->getJacobian(tapedTimeState_, terminalParameters_);
    terminalDerivativesComputed_ = true;
  }
  if (terminalCostValuesComputed_) {
    terminalCostValues_ = terminalADInterfacePtr_->getFunctionValue(tapedTimeState_, terminalParameters_);
    terminalCostValuesComputed_ = true;
  }
  dPhidxx = terminalJacobian_.template block<TERMINAL_COST_DIM, STATE_DIM>(0, 1).transpose() *
            terminalJacobian_.template block<TERMINAL_COST_DIM, STATE_DIM>(0, 1);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, size_t INTERMEDIATE_COST_DIM, size_t TERMINAL_COST_DIM>
void QuadraticGaussNewtonCostBaseAD<STATE_DIM, INPUT_DIM, INTERMEDIATE_COST_DIM, TERMINAL_COST_DIM>::setADInterfaces(
    const std::string& modelName, const std::string& modelFolder) {
  auto intermediateCostAd = [this](const ad_dynamic_vector_t& x, const ad_dynamic_vector_t& p, ad_dynamic_vector_t& y) {
    auto time = x(0);
    auto state = x.template segment<STATE_DIM>(1);
    auto input = x.template segment<INPUT_DIM>(1 + STATE_DIM);
    y = ad_dynamic_vector_t(INTERMEDIATE_COST_DIM);
    ad_intermediate_cost_vector_t yStatic;
    this->intermediateCostFunction(time, state, input, p, yStatic);
    y.template head<INTERMEDIATE_COST_DIM>() = yStatic;
  };
  intermediateADInterfacePtr_.reset(new ad_interface_t(intermediateCostAd, INTERMEDIATE_COST_DIM, 1 + STATE_DIM + INPUT_DIM,
                                                       getNumIntermediateParameters(), modelName + "_intermediate", modelFolder));

  auto terminalCostAd = [this](const ad_dynamic_vector_t& x, const ad_dynamic_vector_t& p, ad_dynamic_vector_t& y) {
    auto time = x(0);
    auto state = x.template segment<STATE_DIM>(1);
    y = ad_dynamic_vector_t(TERMINAL_COST_DIM);
    ad_terminal_cost_vector_t yStatic;
    this->terminalCostFunction(time, state, p, yStatic);
    y.template head<TERMINAL_COST_DIM>() = yStatic;
  };
  terminalADInterfacePtr_.reset(new ad_interface_t(terminalCostAd, TERMINAL_COST_DIM, 1 + STATE_DIM, getNumTerminalParameters(),
                                                   modelName + "_terminal", modelFolder));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, size_t INTERMEDIATE_COST_DIM, size_t TERMINAL_COST_DIM>
void QuadraticGaussNewtonCostBaseAD<STATE_DIM, INPUT_DIM, INTERMEDIATE_COST_DIM, TERMINAL_COST_DIM>::createModels(bool verbose) {
  intermediateADInterfacePtr_->createModels(ad_interface_t::ApproximationOrder::First, verbose);
  terminalADInterfacePtr_->createModels(ad_interface_t::ApproximationOrder::First, verbose);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, size_t INTERMEDIATE_COST_DIM, size_t TERMINAL_COST_DIM>
void QuadraticGaussNewtonCostBaseAD<STATE_DIM, INPUT_DIM, INTERMEDIATE_COST_DIM, TERMINAL_COST_DIM>::loadModelsIfAvailable(bool verbose) {
  intermediateADInterfacePtr_->loadModelsIfAvailable(ad_interface_t::ApproximationOrder::First, verbose);
  terminalADInterfacePtr_->loadModelsIfAvailable(ad_interface_t::ApproximationOrder::First, verbose);
}

}  // namespace ocs2
