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

#include <ocs2_core/constraint/ConstraintBaseAD.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ConstraintBaseAD::ConstraintBaseAD(size_t stateDim, size_t inputDim) : ConstraintBase(stateDim, inputDim){};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ConstraintBaseAD::ConstraintBaseAD(const ConstraintBaseAD& rhs)

    : ConstraintBase(rhs),
      stateInputADInterfacePtr_(new CppAdInterface(*rhs.stateInputADInterfacePtr_)),
      stateOnlyADInterfacePtr_(new CppAdInterface(*rhs.stateOnlyADInterfacePtr_)),
      stateOnlyFinalADInterfacePtr_(new CppAdInterface(*rhs.stateOnlyFinalADInterfacePtr_)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void ConstraintBaseAD::initialize(const std::string& modelName, const std::string& modelFolder, bool recompileLibraries, bool verbose) {
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
void ConstraintBaseAD::setCurrentStateAndControl(const scalar_t& t, const vector_t& x, const vector_t& u) {
  ConstraintBase::setCurrentStateAndControl(t, x, u);

  vector_t tapedTimeStateInput(1 + stateDim_ + inputDim_);
  tapedTimeStateInput << t, x, u;

  vector_t tapedTimeState(1 + stateDim_);
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
vector_t ConstraintBaseAD::getStateInputEqualityConstraint() {
  return stateInputValues_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t ConstraintBaseAD::getStateEqualityConstraint() {
  return stateOnlyValues_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t ConstraintBaseAD::getFinalStateEqualityConstraint() {
  return stateOnlyFinalValues_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_t ConstraintBaseAD::getStateInputEqualityConstraintDerivativesState() {
  return stateInputJacobian_.middleCols(1, stateDim_);
}
/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

matrix_t ConstraintBaseAD::getStateInputEqualityConstraintDerivativesInput() {
  return stateInputJacobian_.rightCols(inputDim_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_t ConstraintBaseAD::getStateEqualityConstraintDerivativesState() {
  return stateOnlyJacobian_.rightCols(stateDim_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_t ConstraintBaseAD::getFinalStateEqualityConstraintDerivativesState() {
  return stateOnlyFinalJacobian_.rightCols(stateDim_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void ConstraintBaseAD::stateInputConstraint(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& input,
                                            ad_vector_t& constraintVector) const {
  constraintVector = ad_vector_t(0);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void ConstraintBaseAD::stateOnlyConstraint(ad_scalar_t time, const ad_vector_t& state, ad_vector_t& constraintVector) const {
  constraintVector = ad_vector_t(0);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void ConstraintBaseAD::stateOnlyFinalConstraint(ad_scalar_t time, const ad_vector_t& state, ad_vector_t& constraintVector) const {
  constraintVector = ad_vector_t(0);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void ConstraintBaseAD::setADInterfaces(const std::string& modelName, const std::string& modelFolder) {
  auto stateInputConstraintAD = [this](const ad_vector_t& x, ad_vector_t& y) {
    auto time = x(0);
    auto state = x.segment(1, stateDim_);
    auto input = x.segment(1 + stateDim_, inputDim_);
    this->stateInputConstraint(time, state, input, y);
  };
  stateInputADInterfacePtr_.reset(
      new CppAdInterface(stateInputConstraintAD, inputDim_, 1 + stateDim_ + inputDim_, modelName + "_stateInput", modelFolder));

  auto stateOnlyConstraintAD = [this](const ad_vector_t& x, ad_vector_t& y) {
    auto time = x(0);
    auto state = x.segment(1, stateDim_);
    this->stateOnlyConstraint(time, state, y);
  };
  stateOnlyADInterfacePtr_.reset(
      new CppAdInterface(stateOnlyConstraintAD, inputDim_, 1 + stateDim_, modelName + "_stateOnly", modelFolder));

  auto stateOnlyConstraintFinalAD = [this](const ad_vector_t& x, ad_vector_t& y) {
    auto time = x(0);
    auto state = x.segment(1, stateDim_);
    this->stateOnlyFinalConstraint(time, state, y);
  };
  stateOnlyFinalADInterfacePtr_.reset(
      new CppAdInterface(stateOnlyConstraintFinalAD, inputDim_, 1 + stateDim_, modelName + "_stateOnlyFinal", modelFolder));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void ConstraintBaseAD::createModels(bool verbose) {
  stateInputADInterfacePtr_->createModels(CppAdInterface::ApproximationOrder::First, verbose);
  stateOnlyADInterfacePtr_->createModels(CppAdInterface::ApproximationOrder::First, verbose);
  stateOnlyFinalADInterfacePtr_->createModels(CppAdInterface::ApproximationOrder::First, verbose);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void ConstraintBaseAD::loadModelsIfAvailable(bool verbose) {
  stateInputADInterfacePtr_->loadModelsIfAvailable(CppAdInterface::ApproximationOrder::First, verbose);
  stateOnlyADInterfacePtr_->loadModelsIfAvailable(CppAdInterface::ApproximationOrder::First, verbose);
  stateOnlyFinalADInterfacePtr_->loadModelsIfAvailable(CppAdInterface::ApproximationOrder::First, verbose);
}

}  // namespace ocs2
