/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

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

#include <ocs2_core/cost/StateInputCostCppAd.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void StateInputCostCppAd::initialize(size_t stateDim, size_t inputDim, size_t parameterDim, const std::string& modelName,
                                     const std::string& modelFolder, bool recompileLibraries, bool verbose) {
  auto costAd = [=](const ad_vector_t& x, const ad_vector_t& p, ad_vector_t& y) {
    assert(x.rows() == 1 + stateDim + inputDim);
    const ad_scalar_t time = x(0);
    const ad_vector_t state = x.segment(1, stateDim);
    const ad_vector_t input = x.tail(inputDim);
    y = ad_vector_t(1);
    y(0) = this->costFunction(time, state, input, p);
  };
  adInterfacePtr_.reset(new ocs2::CppAdInterface(costAd, 1 + stateDim + inputDim, parameterDim, modelName, modelFolder));

  if (recompileLibraries) {
    adInterfacePtr_->createModels(ocs2::CppAdInterface::ApproximationOrder::Second, verbose);
  } else {
    adInterfacePtr_->loadModelsIfAvailable(ocs2::CppAdInterface::ApproximationOrder::Second, verbose);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
StateInputCostCppAd::StateInputCostCppAd(const StateInputCostCppAd& rhs)
    : StateInputCost(rhs), adInterfacePtr_(new ocs2::CppAdInterface(*rhs.adInterfacePtr_)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t StateInputCostCppAd::getValue(scalar_t time, const vector_t& state, const vector_t& input,
                                       const TargetTrajectories& targetTrajectories, const PreComputation&) const {
  vector_t tapedTimeStateInput(1 + state.rows() + input.rows());
  tapedTimeStateInput << time, state, input;
  return adInterfacePtr_->getFunctionValue(tapedTimeStateInput, getParameters(time, targetTrajectories))(0);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation StateInputCostCppAd::getQuadraticApproximation(scalar_t time, const vector_t& state,
                                                                                    const vector_t& input,
                                                                                    const TargetTrajectories& targetTrajectories,
                                                                                    const PreComputation&) const {
  ScalarFunctionQuadraticApproximation cost;

  const size_t stateDim = state.rows();
  const size_t inputDim = input.rows();
  const vector_t params = getParameters(time, targetTrajectories);
  vector_t tapedTimeStateInput(1 + stateDim + inputDim);
  tapedTimeStateInput << time, state, input;

  cost.f = adInterfacePtr_->getFunctionValue(tapedTimeStateInput, params)(0);

  const matrix_t J = adInterfacePtr_->getJacobian(tapedTimeStateInput, params);
  cost.dfdx = J.middleCols(1, stateDim).transpose();
  cost.dfdu = J.rightCols(inputDim).transpose();

  const matrix_t H = adInterfacePtr_->getHessian(0, tapedTimeStateInput, params);
  cost.dfdxx = H.block(1, 1, stateDim, stateDim);
  cost.dfdux = H.block(1 + stateDim, 1, inputDim, stateDim);
  cost.dfduu = H.bottomRightCorner(inputDim, inputDim);

  return cost;
}

}  // namespace ocs2
