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
ConstraintBaseAD::ConstraintBaseAD(size_t stateDim, size_t inputDim) : stateDim_(stateDim), inputDim_(inputDim){};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ConstraintBaseAD::ConstraintBaseAD(const ConstraintBaseAD& rhs)

    : ConstraintBase(rhs),
      stateDim_(rhs.stateDim_),
      inputDim_(rhs.inputDim_),
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
vector_t ConstraintBaseAD::stateInputEqualityConstraint(scalar_t t, const vector_t& x, const vector_t& u) {
  tapedTimeStateInput_.resize(1 + stateDim_ + inputDim_);
  tapedTimeStateInput_ << t, x, u;
  return stateInputADInterfacePtr_->getFunctionValue(tapedTimeStateInput_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t ConstraintBaseAD::stateEqualityConstraint(scalar_t t, const vector_t& x) {
  tapedTimeState_.resize(1 + stateDim_);
  tapedTimeState_ << t, x;
  return stateOnlyADInterfacePtr_->getFunctionValue(tapedTimeState_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t ConstraintBaseAD::finalStateEqualityConstraint(scalar_t t, const vector_t& x) {
  tapedTimeState_.resize(1 + stateDim_);
  tapedTimeState_ << t, x;
  return stateOnlyFinalADInterfacePtr_->getFunctionValue(tapedTimeState_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation ConstraintBaseAD::stateInputEqualityConstraintLinearApproximation(scalar_t t, const vector_t& x,
                                                                                                    const vector_t& u) {
  tapedTimeStateInput_.resize(1 + stateDim_ + inputDim_);
  tapedTimeStateInput_ << t, x, u;
  stateInputJacobian_ = stateInputADInterfacePtr_->getJacobian(tapedTimeStateInput_);

  VectorFunctionLinearApproximation g;
  g.f = stateInputADInterfacePtr_->getFunctionValue(tapedTimeStateInput_);
  g.dfdx = stateInputJacobian_.middleCols(1, stateDim_);
  g.dfdu = stateInputJacobian_.rightCols(inputDim_);
  return g;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation ConstraintBaseAD::stateEqualityConstraintLinearApproximation(scalar_t t, const vector_t& x) {
  tapedTimeState_.resize(1 + stateDim_);
  tapedTimeState_ << t, x;
  stateOnlyJacobian_ = stateOnlyADInterfacePtr_->getJacobian(tapedTimeState_);

  VectorFunctionLinearApproximation g;
  g.f = stateOnlyADInterfacePtr_->getFunctionValue(tapedTimeState_);
  g.dfdx = stateOnlyJacobian_.rightCols(stateDim_);
  return g;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation ConstraintBaseAD::finalStateEqualityConstraintLinearApproximation(scalar_t t, const vector_t& x) {
  tapedTimeState_.resize(1 + stateDim_);
  tapedTimeState_ << t, x;
  stateOnlyFinalJacobian_ = stateOnlyFinalADInterfacePtr_->getJacobian(tapedTimeState_);

  VectorFunctionLinearApproximation gf;
  gf.f = stateOnlyFinalADInterfacePtr_->getFunctionValue(tapedTimeState_);
  gf.dfdx = stateOnlyFinalJacobian_.rightCols(stateDim_);
  return gf;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ad_vector_t ConstraintBaseAD::stateInputConstraint(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& input) const {
  return ad_vector_t(0);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ad_vector_t ConstraintBaseAD::stateOnlyConstraint(ad_scalar_t time, const ad_vector_t& state) const {
  return ad_vector_t(0);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ad_vector_t ConstraintBaseAD::stateOnlyFinalConstraint(ad_scalar_t time, const ad_vector_t& state) const {
  return ad_vector_t(0);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void ConstraintBaseAD::setADInterfaces(const std::string& modelName, const std::string& modelFolder) {
  auto stateInputConstraintAD = [this](const ad_vector_t& x, ad_vector_t& y) {
    auto time = x(0);
    auto state = x.segment(1, stateDim_);
    auto input = x.segment(1 + stateDim_, inputDim_);
    y = this->stateInputConstraint(time, state, input);
  };
  stateInputADInterfacePtr_.reset(
      new CppAdInterface(stateInputConstraintAD, 1 + stateDim_ + inputDim_, modelName + "_stateInput", modelFolder));

  auto stateOnlyConstraintAD = [this](const ad_vector_t& x, ad_vector_t& y) {
    auto time = x(0);
    auto state = x.segment(1, stateDim_);
    y = this->stateOnlyConstraint(time, state);
  };
  stateOnlyADInterfacePtr_.reset(new CppAdInterface(stateOnlyConstraintAD, 1 + stateDim_, modelName + "_stateOnly", modelFolder));

  auto stateOnlyConstraintFinalAD = [this](const ad_vector_t& x, ad_vector_t& y) {
    auto time = x(0);
    auto state = x.segment(1, stateDim_);
    y = this->stateOnlyFinalConstraint(time, state);
  };
  stateOnlyFinalADInterfacePtr_.reset(
      new CppAdInterface(stateOnlyConstraintFinalAD, 1 + stateDim_, modelName + "_stateOnlyFinal", modelFolder));
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
