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
template <size_t STATE_DIM, size_t INPUT_DIM, size_t INTERMEDIATE_COST_DIM, size_t TERMINAL_COST_DIM>
RelaxedBarrierCost<STATE_DIM, INPUT_DIM, INTERMEDIATE_COST_DIM, TERMINAL_COST_DIM>::RelaxedBarrierCost(Config config)
    : BASE(),
      intermediateDerivativesComputed_(false),
      intermediateCostValuesComputed_(false),
      terminalDerivativesComputed_(false),
      terminalCostValuesComputed_(false),
      intermediateConfig_(),
      terminalConfig_() {
  std::fill(intermediateConfig_.begin(), intermediateConfig_.end(), config);
  std::fill(terminalConfig_.begin(), terminalConfig_.end(), config);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, size_t INTERMEDIATE_COST_DIM, size_t TERMINAL_COST_DIM>
RelaxedBarrierCost<STATE_DIM, INPUT_DIM, INTERMEDIATE_COST_DIM, TERMINAL_COST_DIM>::RelaxedBarrierCost(
    std::array<Config, INTERMEDIATE_COST_DIM> intermediateConfig, std::array<Config, TERMINAL_COST_DIM> terminalConfig)
    : BASE(),
      intermediateDerivativesComputed_(false),
      intermediateCostValuesComputed_(false),
      terminalDerivativesComputed_(false),
      terminalCostValuesComputed_(false),
      intermediateConfig_(std::move(intermediateConfig)),
      terminalConfig_(std::move(terminalConfig)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, size_t INTERMEDIATE_COST_DIM, size_t TERMINAL_COST_DIM>
RelaxedBarrierCost<STATE_DIM, INPUT_DIM, INTERMEDIATE_COST_DIM, TERMINAL_COST_DIM>::RelaxedBarrierCost(const RelaxedBarrierCost& rhs)
    : BASE(rhs),
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
template <size_t STATE_DIM, size_t INPUT_DIM, size_t INTERMEDIATE_COST_DIM, size_t TERMINAL_COST_DIM>
void RelaxedBarrierCost<STATE_DIM, INPUT_DIM, INTERMEDIATE_COST_DIM, TERMINAL_COST_DIM>::initialize(const std::string& modelName,
                                                                                                    const std::string& modelFolder,
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
template <size_t STATE_DIM, size_t INPUT_DIM, size_t INTERMEDIATE_COST_DIM, size_t TERMINAL_COST_DIM>
void RelaxedBarrierCost<STATE_DIM, INPUT_DIM, INTERMEDIATE_COST_DIM, TERMINAL_COST_DIM>::setCurrentStateAndControl(
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
void RelaxedBarrierCost<STATE_DIM, INPUT_DIM, INTERMEDIATE_COST_DIM, TERMINAL_COST_DIM>::getIntermediateCost(scalar_t& L) {
  L = 0;
  if (INTERMEDIATE_COST_DIM == 0) {
    return;
  }

  if (!intermediateCostValuesComputed_) {
    intermediateCostValues_ = intermediateADInterfacePtr_->getFunctionValue(tapedTimeStateInput_, intermediateParameters_);
    intermediateCostValuesComputed_ = true;
  }
  for (int i = 0; i < INTERMEDIATE_COST_DIM; i++) {
    L += getPenaltyFunctionValue(intermediateCostValues_[i], intermediateConfig_[i]);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, size_t INTERMEDIATE_COST_DIM, size_t TERMINAL_COST_DIM>
void RelaxedBarrierCost<STATE_DIM, INPUT_DIM, INTERMEDIATE_COST_DIM, TERMINAL_COST_DIM>::getIntermediateCostDerivativeTime(scalar_t& dLdt) {
  if (INTERMEDIATE_COST_DIM == 0) {
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

  Eigen::Matrix<scalar_t, INTERMEDIATE_COST_DIM, 1> penalityFctDerivative;
  for (int i = 0; i < INTERMEDIATE_COST_DIM; i++) {
    penalityFctDerivative(i) = getPenaltyFunctionDerivative(intermediateCostValues_[i], intermediateConfig_[i]);
  }
  dLdt = penalityFctDerivative.dot(intermediateJacobian_.col(0));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, size_t INTERMEDIATE_COST_DIM, size_t TERMINAL_COST_DIM>
void RelaxedBarrierCost<STATE_DIM, INPUT_DIM, INTERMEDIATE_COST_DIM, TERMINAL_COST_DIM>::getIntermediateCostDerivativeState(
    state_vector_t& dLdx) {
  if (INTERMEDIATE_COST_DIM == 0) {
    dLdx.setZero();
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

  Eigen::Matrix<scalar_t, INTERMEDIATE_COST_DIM, 1> penalityFctDerivative;
  for (int i = 0; i < INTERMEDIATE_COST_DIM; i++) {
    penalityFctDerivative(i) = getPenaltyFunctionDerivative(intermediateCostValues_[i], intermediateConfig_[i]);
  }
  dLdx = intermediateJacobian_.template block<INTERMEDIATE_COST_DIM, STATE_DIM>(0, 1).transpose() * penalityFctDerivative;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, size_t INTERMEDIATE_COST_DIM, size_t TERMINAL_COST_DIM>
void RelaxedBarrierCost<STATE_DIM, INPUT_DIM, INTERMEDIATE_COST_DIM, TERMINAL_COST_DIM>::getIntermediateCostSecondDerivativeState(
    state_matrix_t& dLdxx) {
  if (INTERMEDIATE_COST_DIM == 0) {
    dLdxx.setZero();
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

  Eigen::Matrix<scalar_t, INTERMEDIATE_COST_DIM, 1> penalityFctSecondDerivative;
  for (int i = 0; i < INTERMEDIATE_COST_DIM; i++) {
    penalityFctSecondDerivative(i) = getPenaltyFunctionSecondDerivative(intermediateCostValues_[i], intermediateConfig_[i]);
  }
  dLdxx = intermediateJacobian_.template block<INTERMEDIATE_COST_DIM, STATE_DIM>(0, 1).transpose() *
          penalityFctSecondDerivative.asDiagonal() * intermediateJacobian_.template block<INTERMEDIATE_COST_DIM, STATE_DIM>(0, 1);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, size_t INTERMEDIATE_COST_DIM, size_t TERMINAL_COST_DIM>
void RelaxedBarrierCost<STATE_DIM, INPUT_DIM, INTERMEDIATE_COST_DIM, TERMINAL_COST_DIM>::getIntermediateCostDerivativeInput(
    input_vector_t& dLdu) {
  if (INTERMEDIATE_COST_DIM == 0) {
    dLdu.setZero();
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

  Eigen::Matrix<scalar_t, INTERMEDIATE_COST_DIM, 1> penalityFctDerivative;
  for (int i = 0; i < INTERMEDIATE_COST_DIM; i++) {
    penalityFctDerivative(i) = getPenaltyFunctionDerivative(intermediateCostValues_[i], intermediateConfig_[i]);
  }
  dLdu = intermediateJacobian_.template block<INTERMEDIATE_COST_DIM, INPUT_DIM>(0, 1 + STATE_DIM).transpose() * penalityFctDerivative;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, size_t INTERMEDIATE_COST_DIM, size_t TERMINAL_COST_DIM>
void RelaxedBarrierCost<STATE_DIM, INPUT_DIM, INTERMEDIATE_COST_DIM, TERMINAL_COST_DIM>::getIntermediateCostSecondDerivativeInput(
    input_matrix_t& dLduu) {
  if (INTERMEDIATE_COST_DIM == 0) {
    dLduu.setZero();
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

  Eigen::Matrix<scalar_t, INTERMEDIATE_COST_DIM, 1> penalityFctSecondDerivative;
  for (int i = 0; i < INTERMEDIATE_COST_DIM; i++) {
    penalityFctSecondDerivative(i) = getPenaltyFunctionSecondDerivative(intermediateCostValues_[i], intermediateConfig_[i]);
  }
  dLduu = intermediateJacobian_.template block<INTERMEDIATE_COST_DIM, INPUT_DIM>(0, 1 + STATE_DIM).transpose() *
          penalityFctSecondDerivative.asDiagonal() *
          intermediateJacobian_.template block<INTERMEDIATE_COST_DIM, INPUT_DIM>(0, 1 + STATE_DIM);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, size_t INTERMEDIATE_COST_DIM, size_t TERMINAL_COST_DIM>
void RelaxedBarrierCost<STATE_DIM, INPUT_DIM, INTERMEDIATE_COST_DIM, TERMINAL_COST_DIM>::getIntermediateCostDerivativeInputState(
    input_state_matrix_t& dLdux) {
  if (INTERMEDIATE_COST_DIM == 0) {
    dLdux.setZero();
    return;
  }

  if (!intermediateDerivativesComputed_) {
    intermediateJacobian_ = intermediateADInterfacePtr_->getJacobian(tapedTimeStateInput_, intermediateParameters_);
    intermediateDerivativesComputed_ = true;
  }

  Eigen::Matrix<scalar_t, INTERMEDIATE_COST_DIM, 1> penalityFctSecondDerivative;
  for (int i = 0; i < INTERMEDIATE_COST_DIM; i++) {
    penalityFctSecondDerivative(i) = getPenaltyFunctionSecondDerivative(intermediateCostValues_[i], intermediateConfig_[i]);
  }
  dLdux = intermediateJacobian_.template block<INTERMEDIATE_COST_DIM, INPUT_DIM>(0, 1 + STATE_DIM).transpose() *
          penalityFctSecondDerivative.asDiagonal() * intermediateJacobian_.template block<INTERMEDIATE_COST_DIM, STATE_DIM>(0, 1);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, size_t INTERMEDIATE_COST_DIM, size_t TERMINAL_COST_DIM>
void RelaxedBarrierCost<STATE_DIM, INPUT_DIM, INTERMEDIATE_COST_DIM, TERMINAL_COST_DIM>::getTerminalCost(scalar_t& Phi) {
  Phi = 0;
  if (TERMINAL_COST_DIM == 0) {
    return;
  }

  if (!terminalCostValuesComputed_) {
    terminalCostValues_ = terminalADInterfacePtr_->getFunctionValue(tapedTimeState_, terminalParameters_);
    terminalCostValuesComputed_ = true;
  }

  for (int i = 0; i < TERMINAL_COST_DIM; i++) {
    Phi += getPenaltyFunctionValue(terminalCostValues_[i], terminalConfig_[i]);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, size_t INTERMEDIATE_COST_DIM, size_t TERMINAL_COST_DIM>
void RelaxedBarrierCost<STATE_DIM, INPUT_DIM, INTERMEDIATE_COST_DIM, TERMINAL_COST_DIM>::getTerminalCostDerivativeTime(scalar_t& dPhidt) {
  if (TERMINAL_COST_DIM == 0) {
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

  Eigen::Matrix<scalar_t, TERMINAL_COST_DIM, 1> penalityFctDerivative;
  for (int i = 0; i < TERMINAL_COST_DIM; i++) {
    penalityFctDerivative(i) = getPenaltyFunctionDerivative(terminalCostValues_[i], terminalConfig_[i]);
  }
  dPhidt = penalityFctDerivative.transpose() * terminalJacobian_.col(0);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, size_t INTERMEDIATE_COST_DIM, size_t TERMINAL_COST_DIM>
void RelaxedBarrierCost<STATE_DIM, INPUT_DIM, INTERMEDIATE_COST_DIM, TERMINAL_COST_DIM>::getTerminalCostDerivativeState(
    state_vector_t& dPhidx) {
  if (TERMINAL_COST_DIM == 0) {
    dPhidx.setZero();
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

  Eigen::Matrix<scalar_t, TERMINAL_COST_DIM, 1> penalityFctDerivative;
  for (int i = 0; i < TERMINAL_COST_DIM; i++) {
    penalityFctDerivative(i) = getPenaltyFunctionDerivative(terminalCostValues_[i], terminalConfig_[i]);
  }
  dPhidx = terminalJacobian_.template block<TERMINAL_COST_DIM, STATE_DIM>(0, 1).transpose() * penalityFctDerivative;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, size_t INTERMEDIATE_COST_DIM, size_t TERMINAL_COST_DIM>
void RelaxedBarrierCost<STATE_DIM, INPUT_DIM, INTERMEDIATE_COST_DIM, TERMINAL_COST_DIM>::getTerminalCostSecondDerivativeState(
    state_matrix_t& dPhidxx) {
  if (TERMINAL_COST_DIM == 0) {
    dPhidxx.setZero();
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

  Eigen::Matrix<scalar_t, TERMINAL_COST_DIM, 1> penalityFctSecondDerivative;
  for (int i = 0; i < TERMINAL_COST_DIM; i++) {
    penalityFctSecondDerivative(i) = getPenaltyFunctionSecondDerivative(terminalCostValues_[i], terminalConfig_[i]);
  }
  dPhidxx = terminalJacobian_.template block<TERMINAL_COST_DIM, STATE_DIM>(0, 1).transpose() * penalityFctSecondDerivative.asDiagonal() *
            terminalJacobian_.template block<TERMINAL_COST_DIM, STATE_DIM>(0, 1);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, size_t INTERMEDIATE_COST_DIM, size_t TERMINAL_COST_DIM>
void RelaxedBarrierCost<STATE_DIM, INPUT_DIM, INTERMEDIATE_COST_DIM, TERMINAL_COST_DIM>::setADInterfaces(const std::string& modelName,
                                                                                                         const std::string& modelFolder) {
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
void RelaxedBarrierCost<STATE_DIM, INPUT_DIM, INTERMEDIATE_COST_DIM, TERMINAL_COST_DIM>::createModels(bool verbose) {
  intermediateADInterfacePtr_->createModels(ad_interface_t::ApproximationOrder::First, verbose);
  terminalADInterfacePtr_->createModels(ad_interface_t::ApproximationOrder::First, verbose);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, size_t INTERMEDIATE_COST_DIM, size_t TERMINAL_COST_DIM>
void RelaxedBarrierCost<STATE_DIM, INPUT_DIM, INTERMEDIATE_COST_DIM, TERMINAL_COST_DIM>::loadModelsIfAvailable(bool verbose) {
  intermediateADInterfacePtr_->loadModelsIfAvailable(ad_interface_t::ApproximationOrder::First, verbose);
  terminalADInterfacePtr_->loadModelsIfAvailable(ad_interface_t::ApproximationOrder::First, verbose);
}

}  // namespace ocs2
