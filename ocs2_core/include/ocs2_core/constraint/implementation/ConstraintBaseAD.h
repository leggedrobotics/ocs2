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
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM>
ConstraintBaseAD<Derived, STATE_DIM, INPUT_DIM>::ConstraintBaseAD() : BASE(){};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM>
ConstraintBaseAD<Derived, STATE_DIM, INPUT_DIM>::ConstraintBaseAD(const ConstraintBaseAD& rhs)

    : BASE(rhs),
      stateInputADInterfacePtr_(new ad_interface_t(*rhs.stateInputADInterfacePtr_)),
      stateOnlyADInterfacePtr_(new ad_interface_t(*rhs.stateOnlyADInterfacePtr_)),
      stateOnlyFinalADInterfacePtr_(new ad_interface_t(*rhs.stateOnlyFinalADInterfacePtr_)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM>
void ConstraintBaseAD<Derived, STATE_DIM, INPUT_DIM>::initialize(const std::string& modelName, const std::string& modelFolder,
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
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM>
void ConstraintBaseAD<Derived, STATE_DIM, INPUT_DIM>::setCurrentStateAndControl(const scalar_t& t, const state_vector_t& x,
                                                                                const input_vector_t& u) {
  BASE::setCurrentStateAndControl(t, x, u);

  dynamic_vector_t tapedTimeStateInput(1 + STATE_DIM + INPUT_DIM);
  tapedTimeStateInput << t, x, u;

  dynamic_vector_t tapedTimeState(1 + STATE_DIM);
  tapedTimeState << t, x;

  stateInputValues_ = stateInputADInterfacePtr_->getFunctionValue(tapedTimeStateInput);
  stateOnlyValues_ = stateOnlyADInterfacePtr_->getFunctionValue(tapedTimeState);
  stateOnlyFinalValues_ = stateOnlyFinalADInterfacePtr_->getFunctionValue(tapedTimeState);

  stateInputJacobian_ = stateInputADInterfacePtr_->getJacobian(tapedTimeStateInput);
  stateOnlyJacobian_ = stateOnlyADInterfacePtr_->getJacobian(tapedTimeState);
  stateOnlyFinalJacobian_ = stateOnlyFinalADInterfacePtr_->getJacobian(tapedTimeState);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM>
void ConstraintBaseAD<Derived, STATE_DIM, INPUT_DIM>::setADInterfaces(const std::string& modelName, const std::string& modelFolder) {
  auto stateInputConstraintAD = [this](const ad_dynamic_vector_t& x, ad_dynamic_vector_t& y) {
    auto time = x(0);
    auto state = x.template segment<state_dim_>(1);
    auto input = x.template segment<input_dim_>(1 + state_dim_);
    this->stateInputConstraint(time, state, input, y);
  };
  stateInputADInterfacePtr_.reset(
      new ad_interface_t(stateInputConstraintAD, MAX_CONSTRAINT_DIM_, 1 + state_dim_ + input_dim_, modelName + "_stateInput", modelFolder));

  auto stateOnlyConstraintAD = [this](const ad_dynamic_vector_t& x, ad_dynamic_vector_t& y) {
    auto time = x(0);
    auto state = x.template segment<state_dim_>(1);
    this->stateOnlyConstraint(time, state, y);
  };
  stateOnlyADInterfacePtr_.reset(
      new ad_interface_t(stateOnlyConstraintAD, MAX_CONSTRAINT_DIM_, 1 + state_dim_, modelName + "_stateOnly", modelFolder));

  auto stateOnlyConstraintFinalAD = [this](const ad_dynamic_vector_t& x, ad_dynamic_vector_t& y) {
    auto time = x(0);
    auto state = x.template segment<state_dim_>(1);
    this->stateOnlyFinalConstraint(time, state, y);
  };
  stateOnlyFinalADInterfacePtr_.reset(
      new ad_interface_t(stateOnlyConstraintFinalAD, MAX_CONSTRAINT_DIM_, 1 + state_dim_, modelName + "_stateOnlyFinal", modelFolder));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM>
void ConstraintBaseAD<Derived, STATE_DIM, INPUT_DIM>::createModels(bool verbose) {
  stateInputADInterfacePtr_->createModels(ad_interface_t::ApproximationOrder::First, verbose);
  stateOnlyADInterfacePtr_->createModels(ad_interface_t::ApproximationOrder::First, verbose);
  stateOnlyFinalADInterfacePtr_->createModels(ad_interface_t::ApproximationOrder::First, verbose);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class Derived, size_t STATE_DIM, size_t INPUT_DIM>
void ConstraintBaseAD<Derived, STATE_DIM, INPUT_DIM>::loadModelsIfAvailable(bool verbose) {
  stateInputADInterfacePtr_->loadModelsIfAvailable(ad_interface_t::ApproximationOrder::First, verbose);
  stateOnlyADInterfacePtr_->loadModelsIfAvailable(ad_interface_t::ApproximationOrder::First, verbose);
  stateOnlyFinalADInterfacePtr_->loadModelsIfAvailable(ad_interface_t::ApproximationOrder::First, verbose);
}

}  // namespace ocs2
