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

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, size_t NUM_MODES>
SystemDynamicsBaseAD<STATE_DIM, INPUT_DIM, NUM_MODES>::SystemDynamicsBaseAD()
    : BASE(),
      flowJacobian_(state_timeStateInput_matrix_t::Zero()),
      jumpJacobian_(state_timeState_matrix_t::Zero()),
      guardJacobian_(mode_timeState_matrix_t::Zero()){};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, size_t NUM_MODES>
SystemDynamicsBaseAD<STATE_DIM, INPUT_DIM, NUM_MODES>::SystemDynamicsBaseAD(const SystemDynamicsBaseAD& rhs)
    : BASE(rhs),
      flowMapADInterfacePtr_(new ad_interface_t(*rhs.flowMapADInterfacePtr_)),
      jumpMapADInterfacePtr_(new ad_interface_t(*rhs.jumpMapADInterfacePtr_)),
      guardSurfacesADInterfacePtr_(new ad_interface_t(*rhs.guardSurfacesADInterfacePtr_)),
      flowJacobian_(state_timeStateInput_matrix_t::Zero()),
      jumpJacobian_(state_timeState_matrix_t::Zero()),
      guardJacobian_(mode_timeState_matrix_t::Zero()) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, size_t NUM_MODES>
void SystemDynamicsBaseAD<STATE_DIM, INPUT_DIM, NUM_MODES>::initialize(const std::string& modelName, const std::string& modelFolder,
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
template <size_t STATE_DIM, size_t INPUT_DIM, size_t NUM_MODES>
void SystemDynamicsBaseAD<STATE_DIM, INPUT_DIM, NUM_MODES>::computeFlowMap(const scalar_t& time, const state_vector_t& state,
                                                                           const input_vector_t& input, state_vector_t& stateDerivative) {
  dynamic_vector_t tapedInput(1 + STATE_DIM + INPUT_DIM);
  tapedInput << time, state, input;

  stateDerivative = flowMapADInterfacePtr_->getFunctionValue(tapedInput);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, size_t NUM_MODES>
void SystemDynamicsBaseAD<STATE_DIM, INPUT_DIM, NUM_MODES>::computeJumpMap(const scalar_t& time, const state_vector_t& state,
                                                                           state_vector_t& jumpedState) {
  dynamic_vector_t tapedInput(1 + STATE_DIM);
  tapedInput << time, state;

  jumpedState = jumpMapADInterfacePtr_->getFunctionValue(tapedInput);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, size_t NUM_MODES>
void SystemDynamicsBaseAD<STATE_DIM, INPUT_DIM, NUM_MODES>::computeGuardSurfaces(const scalar_t& time, const state_vector_t& state,
                                                                                 dynamic_vector_t& guardSurfacesValue) {
  dynamic_vector_t tapedInput(1 + STATE_DIM);
  tapedInput << time, state;

  guardSurfacesValue = guardSurfacesADInterfacePtr_->getFunctionValue(tapedInput);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, size_t NUM_MODES>
void SystemDynamicsBaseAD<STATE_DIM, INPUT_DIM, NUM_MODES>::setCurrentStateAndControl(const scalar_t& time, const state_vector_t& state,
                                                                                      const input_vector_t& input) {
  BASE::setCurrentStateAndControl(time, state, input);

  dynamic_vector_t tapedTimeStateInput(1 + STATE_DIM + INPUT_DIM);
  tapedTimeStateInput << time, state, input;

  dynamic_vector_t tapedTimeState(1 + STATE_DIM);
  tapedTimeState << time, state;

  flowJacobian_ = flowMapADInterfacePtr_->getJacobian(tapedTimeStateInput);
  jumpJacobian_ = jumpMapADInterfacePtr_->getJacobian(tapedTimeState);
  guardJacobian_ = guardSurfacesADInterfacePtr_->getJacobian(tapedTimeState);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, size_t NUM_MODES>
void SystemDynamicsBaseAD<STATE_DIM, INPUT_DIM, NUM_MODES>::setADInterfaces(const std::string& modelName, const std::string& modelFolder) {
  auto tapedFlowMap = [this](const ad_dynamic_vector_t& x, ad_dynamic_vector_t& y) {
    auto time = x(0);
    auto state = x.template segment<STATE_DIM>(1);
    auto input = x.template segment<INPUT_DIM>(1 + STATE_DIM);
    this->systemFlowMap(time, state, input, y);
  };
  flowMapADInterfacePtr_.reset(
      new ad_interface_t(tapedFlowMap, STATE_DIM, 1 + STATE_DIM + INPUT_DIM, modelName + "_flow_map", modelFolder));

  auto tapedJumpMap = [this](const ad_dynamic_vector_t& x, ad_dynamic_vector_t& y) {
    auto time = x(0);
    auto state = x.template segment<STATE_DIM>(1);
    this->systemJumpMap(time, state, y);
  };
  jumpMapADInterfacePtr_.reset(new ad_interface_t(tapedJumpMap, STATE_DIM, 1 + STATE_DIM, modelName + "_jump_map", modelFolder));

  auto tapedGuardSurfaces = [this](const ad_dynamic_vector_t& x, ad_dynamic_vector_t& y) {
    auto time = x(0);
    auto state = x.template segment<STATE_DIM>(1);
    this->systemGuardSurfaces(time, state, y);
  };
  guardSurfacesADInterfacePtr_.reset(
      new ad_interface_t(tapedGuardSurfaces, NUM_MODES, 1 + STATE_DIM, modelName + "_guard_surfaces", modelFolder));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, size_t NUM_MODES>
void SystemDynamicsBaseAD<STATE_DIM, INPUT_DIM, NUM_MODES>::createModels(bool verbose) {
  flowMapADInterfacePtr_->createModels(ad_interface_t::ApproximationOrder::First, verbose);
  jumpMapADInterfacePtr_->createModels(ad_interface_t::ApproximationOrder::First, verbose);
  guardSurfacesADInterfacePtr_->createModels(ad_interface_t::ApproximationOrder::First, verbose);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, size_t NUM_MODES>
void SystemDynamicsBaseAD<STATE_DIM, INPUT_DIM, NUM_MODES>::loadModelsIfAvailable(bool verbose) {
  flowMapADInterfacePtr_->loadModelsIfAvailable(ad_interface_t::ApproximationOrder::First, verbose);
  jumpMapADInterfacePtr_->loadModelsIfAvailable(ad_interface_t::ApproximationOrder::First, verbose);
  guardSurfacesADInterfacePtr_->loadModelsIfAvailable(ad_interface_t::ApproximationOrder::First, verbose);
}

}  // namespace ocs2
