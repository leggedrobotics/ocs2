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

#include <ocs2_core/constraint/StateConstraintCppAd.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void StateConstraintCppAd::initialize(size_t stateDim, size_t parameterDim, const std::string& modelName, const std::string& modelFolder,
                                      bool recompileLibraries, bool verbose) {
  auto constraintAd = [=](const ad_vector_t& x, const ad_vector_t& p, ad_vector_t& y) {
    assert(x.rows() == 1 + stateDim);
    const ad_scalar_t time = x(0);
    const ad_vector_t state = x.tail(stateDim);
    y = this->constraintFunction(time, state, p);
  };
  adInterfacePtr_.reset(new ocs2::CppAdInterface(constraintAd, 1 + stateDim, parameterDim, modelName, modelFolder));

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
StateConstraintCppAd::StateConstraintCppAd(const StateConstraintCppAd& rhs)
    : StateConstraint(rhs), adInterfacePtr_(new ocs2::CppAdInterface(*rhs.adInterfacePtr_)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t StateConstraintCppAd::getValue(scalar_t time, const vector_t& state, const PreComputation&) const {
  vector_t tapedTimeState(1 + state.rows());
  tapedTimeState << time, state;
  return adInterfacePtr_->getFunctionValue(tapedTimeState, getParameters(time));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation StateConstraintCppAd::getLinearApproximation(scalar_t time, const vector_t& state,
                                                                               const PreComputation&) const {
  VectorFunctionLinearApproximation constraint;

  const size_t stateDim = state.rows();
  const vector_t params = getParameters(time);
  vector_t tapedTimeState(1 + stateDim);
  tapedTimeState << time, state;

  constraint.f = adInterfacePtr_->getFunctionValue(tapedTimeState, params);
  const matrix_t J = adInterfacePtr_->getJacobian(tapedTimeState, params);
  constraint.dfdx = J.rightCols(stateDim);

  return constraint;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionQuadraticApproximation StateConstraintCppAd::getQuadraticApproximation(scalar_t time, const vector_t& state,
                                                                                     const PreComputation&) const {
  if (getOrder() != ConstraintOrder::Quadratic) {
    throw std::runtime_error("[StateConstraintCppAd] Quadratic approximation not supported!");
  }

  VectorFunctionQuadraticApproximation constraint;

  const size_t stateDim = state.rows();
  const vector_t params = getParameters(time);
  vector_t tapedTimeState(1 + stateDim);
  tapedTimeState << time, state;

  constraint.f = adInterfacePtr_->getFunctionValue(tapedTimeState, params);
  const matrix_t J = adInterfacePtr_->getJacobian(tapedTimeState, params);
  constraint.dfdx = J.rightCols(stateDim);

  const size_t numConstraints = constraint.f.rows();
  constraint.dfdxx.resize(numConstraints);
  constraint.dfdux.resize(numConstraints);
  constraint.dfduu.resize(numConstraints);
  for (int i = 0; i < numConstraints; i++) {
    const matrix_t H = adInterfacePtr_->getHessian(i, tapedTimeState, params);
    constraint.dfdxx[i] = H.bottomRightCorner(stateDim, stateDim);
  }

  return constraint;
}

}  // namespace ocs2
