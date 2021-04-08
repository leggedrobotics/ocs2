/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

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

#include <ocs2_core/cost/StateInputGaussNewtonCostAd.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
StateInputCostGaussNewtonAd::StateInputCostGaussNewtonAd(size_t stateDim, size_t inputDim)
    : StateInputCost(), stateDim_(stateDim), inputDim_(inputDim) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
StateInputCostGaussNewtonAd::StateInputCostGaussNewtonAd(const StateInputCostGaussNewtonAd& rhs)
    : StateInputCost(rhs),
      stateDim_(rhs.stateDim_),
      inputDim_(rhs.inputDim_),
      costVectorFunctionADInterfacePtr_(new CppAdInterface(*rhs.costVectorFunctionADInterfacePtr_)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void StateInputCostGaussNewtonAd::initialize(const std::string& modelName, const std::string& modelFolder, bool recompileLibraries,
                                             bool verbose) {
  setADInterfaces(modelName, modelFolder);
  if (recompileLibraries) {
    costVectorFunctionADInterfacePtr_->createModels(CppAdInterface::ApproximationOrder::First, verbose);
  } else {
    costVectorFunctionADInterfacePtr_->loadModelsIfAvailable(CppAdInterface::ApproximationOrder::First, verbose);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t StateInputCostGaussNewtonAd::getValue(scalar_t time, const vector_t& state, const vector_t& input,
                                               const CostDesiredTrajectories& desiredTrajectory) const {
  vector_t timeStateInput(1 + stateDim_ + inputDim_);
  timeStateInput << time, state, input;
  const auto parameters = getParameters(time, desiredTrajectory);
  const auto costVector = costVectorFunctionADInterfacePtr_->getFunctionValue(timeStateInput, parameters);
  return 0.5 * costVector.squaredNorm();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation StateInputCostGaussNewtonAd::getQuadraticApproximation(
    scalar_t time, const vector_t& state, const vector_t& input, const CostDesiredTrajectories& desiredTrajectory) const {
  vector_t timeStateInput(1 + stateDim_ + inputDim_);
  timeStateInput << time, state, input;
  const auto parameters = getParameters(time, desiredTrajectory);
  const auto costVector = costVectorFunctionADInterfacePtr_->getFunctionValue(timeStateInput, parameters);
  const auto costVectorJacobian = costVectorFunctionADInterfacePtr_->getJacobian(timeStateInput, parameters);

  ScalarFunctionQuadraticApproximation L;
  L.f = 0.5 * costVector.squaredNorm();
  L.dfdx.noalias() = costVectorJacobian.middleCols(1, stateDim_).transpose() * costVector;
  L.dfdxx.noalias() = costVectorJacobian.middleCols(1, stateDim_).transpose() * costVectorJacobian.middleCols(1, stateDim_);
  L.dfdu.noalias() = costVectorJacobian.rightCols(inputDim_).transpose() * costVector;
  L.dfduu.noalias() = costVectorJacobian.rightCols(inputDim_).transpose() * costVectorJacobian.rightCols(inputDim_);
  L.dfdux.noalias() = costVectorJacobian.rightCols(inputDim_).transpose() * costVectorJacobian.middleCols(1, stateDim_);
  return L;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void StateInputCostGaussNewtonAd::setADInterfaces(const std::string& modelName, const std::string& modelFolder) {
  auto costVector = [this](const ad_vector_t& x, const ad_vector_t& p, ad_vector_t& y) {
    auto time = x(0);
    auto state = x.segment(1, stateDim_);
    auto input = x.tail(inputDim_);
    y = this->costVectorFunction(time, state, input, p);
  };
  costVectorFunctionADInterfacePtr_.reset(
      new CppAdInterface(costVector, 1 + stateDim_ + inputDim_, getNumParameters(), modelName + "_GN_cost_vector", modelFolder));
}

}  // namespace ocs2