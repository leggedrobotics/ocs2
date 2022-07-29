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

#include <ocs2_core/constraint/StateInputConstraintCppAd.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void StateInputConstraintCppAd::initialize(size_t stateDim, size_t inputDim, size_t parameterDim, const std::string& modelName,
                                           const std::string& modelFolder, bool recompileLibraries, bool verbose) {
  auto constraintAd = [=](const ad_vector_t& x, const ad_vector_t& p, ad_vector_t& y) {
    assert(x.rows() == 1 + stateDim + inputDim);
    const ad_scalar_t time = x(0);
    const ad_vector_t state = x.segment(1, stateDim);
    const ad_vector_t input = x.tail(inputDim);
    y = this->constraintFunction(time, state, input, p);
  };
  adInterfacePtr_.reset(new ocs2::CppAdInterface(constraintAd, 1 + stateDim + inputDim, parameterDim, modelName, modelFolder));

  ocs2::CppAdInterface::ApproximationOrder orderCppAd;
  if (getOrder() == ConstraintOrder::Linear) {
    orderCppAd = ocs2::CppAdInterface::ApproximationOrder::First;
  } else {
    orderCppAd = ocs2::CppAdInterface::ApproximationOrder::Second;
  }

  if (recompileLibraries) {
    adInterfacePtr_->createModels(orderCppAd, verbose);
  } else {
    adInterfacePtr_->loadModelsIfAvailable(orderCppAd, verbose);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
StateInputConstraintCppAd::StateInputConstraintCppAd(const StateInputConstraintCppAd& rhs)
    : StateInputConstraint(rhs), adInterfacePtr_(new ocs2::CppAdInterface(*rhs.adInterfacePtr_)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t StateInputConstraintCppAd::getValue(scalar_t time, const vector_t& state, const vector_t& input,
                                             const PreComputation& preComputation) const {
  vector_t tapedTimeStateInput(1 + state.rows() + input.rows());
  tapedTimeStateInput << time, state, input;
  return adInterfacePtr_->getFunctionValue(tapedTimeStateInput, getParameters(time, preComputation));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation StateInputConstraintCppAd::getLinearApproximation(scalar_t time, const vector_t& state,
                                                                                    const vector_t& input,
                                                                                    const PreComputation& preComputation) const {
  VectorFunctionLinearApproximation constraint;

  const size_t stateDim = state.rows();
  const size_t inputDim = input.rows();
  const vector_t params = getParameters(time, preComputation);
  vector_t tapedTimeStateInput(1 + stateDim + inputDim);
  tapedTimeStateInput << time, state, input;

  constraint.f = adInterfacePtr_->getFunctionValue(tapedTimeStateInput, params);
  const matrix_t J = adInterfacePtr_->getJacobian(tapedTimeStateInput, params);
  constraint.dfdx = J.middleCols(1, stateDim);
  constraint.dfdu = J.rightCols(inputDim);

  return constraint;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionQuadraticApproximation StateInputConstraintCppAd::getQuadraticApproximation(scalar_t time, const vector_t& state,
                                                                                          const vector_t& input,
                                                                                          const PreComputation& preComputation) const {
  if (getOrder() != ConstraintOrder::Quadratic) {
    throw std::runtime_error("[StateInputConstraintCppAd] Quadratic approximation not supported!");
  }

  VectorFunctionQuadraticApproximation constraint;

  const size_t stateDim = state.rows();
  const size_t inputDim = input.rows();
  const vector_t params = getParameters(time, preComputation);
  vector_t tapedTimeStateInput(1 + stateDim + inputDim);
  tapedTimeStateInput << time, state, input;

  constraint.f = adInterfacePtr_->getFunctionValue(tapedTimeStateInput, params);
  const matrix_t J = adInterfacePtr_->getJacobian(tapedTimeStateInput, params);
  constraint.dfdx = J.middleCols(1, stateDim);
  constraint.dfdu = J.rightCols(inputDim);

  const size_t numConstraints = constraint.f.rows();
  constraint.dfdxx.resize(numConstraints);
  constraint.dfdux.resize(numConstraints);
  constraint.dfduu.resize(numConstraints);
  for (int i = 0; i < numConstraints; i++) {
    const matrix_t H = adInterfacePtr_->getHessian(i, tapedTimeStateInput, params);
    constraint.dfdxx[i] = H.block(1, 1, stateDim, stateDim);
    constraint.dfdux[i] = H.block(1 + stateDim, 1, inputDim, stateDim);
    constraint.dfduu[i] = H.bottomRightCorner(inputDim, inputDim);
  }

  return constraint;
}

}  // namespace ocs2
