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
void StateInputCostGaussNewtonAd::initialize(size_t stateDim, size_t inputDim, size_t parameterDim, const std::string& modelName,
                                             const std::string& modelFolder, bool recompileLibraries, bool verbose) {
  auto costVectorAd = [=](const ad_vector_t& x, const ad_vector_t& p, ad_vector_t& y) {
    assert(x.rows() == 1 + stateDim + inputDim);
    const ad_scalar_t time = x(0);
    const ad_vector_t state = x.segment(1, stateDim);
    const ad_vector_t input = x.tail(inputDim);
    y = this->costVectorFunction(time, state, input, p);
  };
  adInterfacePtr_.reset(new ocs2::CppAdInterface(costVectorAd, 1 + stateDim + inputDim, parameterDim, modelName, modelFolder));

  if (recompileLibraries) {
    adInterfacePtr_->createModels(ocs2::CppAdInterface::ApproximationOrder::First, verbose);
  } else {
    adInterfacePtr_->loadModelsIfAvailable(ocs2::CppAdInterface::ApproximationOrder::First, verbose);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
StateInputCostGaussNewtonAd::StateInputCostGaussNewtonAd(const StateInputCostGaussNewtonAd& rhs)
    : StateInputCost(rhs), adInterfacePtr_(new ocs2::CppAdInterface(*rhs.adInterfacePtr_)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t StateInputCostGaussNewtonAd::getValue(scalar_t time, const vector_t& state, const vector_t& input,
                                               const TargetTrajectories& targetTrajectories, const PreComputation&) const {
  vector_t timeStateInput(1 + state.rows() + input.rows());
  timeStateInput << time, state, input;
  const auto parameters = getParameters(time, targetTrajectories);
  const auto costVector = adInterfacePtr_->getFunctionValue(timeStateInput, parameters);
  return 0.5 * costVector.squaredNorm();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation StateInputCostGaussNewtonAd::getQuadraticApproximation(scalar_t time, const vector_t& state,
                                                                                            const vector_t& input,
                                                                                            const TargetTrajectories& targetTrajectories,
                                                                                            const PreComputation&) const {
  const auto stateDim = state.rows();
  const auto inputDim = input.rows();
  vector_t timeStateInput(1 + stateDim + inputDim);
  timeStateInput << time, state, input;
  const auto parameters = getParameters(time, targetTrajectories);
  const auto gnApproximation = adInterfacePtr_->getGaussNewtonApproximation(timeStateInput, parameters);

  ScalarFunctionQuadraticApproximation L;
  L.f = gnApproximation.f;
  L.dfdx.noalias() = gnApproximation.dfdx.middleRows(1, stateDim);
  L.dfdu.noalias() = gnApproximation.dfdx.bottomRows(inputDim);
  L.dfdxx = gnApproximation.dfdxx.block(1, 1, stateDim, stateDim);
  L.dfdux.noalias() = gnApproximation.dfdxx.block(1 + stateDim, 1, inputDim, stateDim);
  L.dfduu.noalias() = gnApproximation.dfdxx.block(1 + stateDim, 1 + stateDim, inputDim, inputDim);
  return L;
}

}  // namespace ocs2
