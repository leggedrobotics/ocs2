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

#include "ocs2_perceptive/end_effector/EndEffectorDistanceConstraintCppAd.h"

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
EndEffectorDistanceConstraintCppAd::EndEffectorDistanceConstraintCppAd(size_t stateDim, size_t inputDim, Config config,
                                                                       std::unique_ptr<EndEffectorKinematics<ad_scalar_t>> adKinematicsPtr)
    : StateInputConstraint(ConstraintOrder::Linear),
      stateDim_(stateDim),
      inputDim_(inputDim),
      config_(std::move(config)),
      adKinematicsPtr_(std::move(adKinematicsPtr)) {
  auto surrogateKinematicsAD = [this](const ad_vector_t& x, ad_vector_t& y) {
    const size_t numEEs = adKinematicsPtr_->getIds().size();
    const auto eePositions = adKinematicsPtr_->getPosition(x);
    y.resize(3 * numEEs);
    for (size_t i = 0; i < numEEs; i++) {
      y.segment<3>(3 * i) = eePositions[i];
    }  // end of i loop
  };

  std::string libName = "ee_kinemtaics";
  for (const auto& id : adKinematicsPtr_->getIds()) {
    libName += "_" + id;
  }
  kinematicsModelPtr_.reset(new CppAdInterface(surrogateKinematicsAD, stateDim_, libName));
  constexpr auto order = CppAdInterface::ApproximationOrder::First;
  if (config_.generateModel) {
    kinematicsModelPtr_->createModels(order, config_.verbose);
  } else {
    kinematicsModelPtr_->loadModelsIfAvailable(order, config_.verbose);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
EndEffectorDistanceConstraintCppAd::EndEffectorDistanceConstraintCppAd(const EndEffectorDistanceConstraintCppAd& other)
    : StateInputConstraint(other),
      stateDim_(other.stateDim_),
      inputDim_(other.inputDim_),
      config_(other.config_),
      adKinematicsPtr_(other.adKinematicsPtr_->clone()),
      kinematicsModelPtr_(new CppAdInterface(*other.kinematicsModelPtr_)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void EndEffectorDistanceConstraintCppAd::set(scalar_t clearance, const DistanceTransformInterface& distanceTransform) {
  const auto numEEs = adKinematicsPtr_->getIds().size();
  clearances_ = vector_t::Ones(numEEs) * clearance;
  distanceTransformPtr_ = &distanceTransform;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void EndEffectorDistanceConstraintCppAd::set(vector_t clearances, const DistanceTransformInterface& distanceTransform) {
  const auto numEEs = adKinematicsPtr_->getIds().size();
  if (clearances.size() != numEEs) {
    throw std::runtime_error(
        "[EndEffectorDistanceConstraint] The size of the input clearance vector doesn't match the number of end-effectors!");
  }

  clearances_ = clearances;
  distanceTransformPtr_ = &distanceTransform;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t EndEffectorDistanceConstraintCppAd::getValue(scalar_t time, const vector_t& state, const vector_t& input,
                                                      const PreComputation& preComp) const {
  const auto numEEs = adKinematicsPtr_->getIds().size();
  const auto eePositions = kinematicsModelPtr_->getFunctionValue(state);
  assert(eePositions.size() == 3 * numEEs);

  vector_t g(numEEs);
  for (size_t i = 0; i < numEEs; i++) {
    g(i) = config_.weight * (distanceTransformPtr_->getValue(eePositions.segment<3>(3 * i)) - clearances_[i]);
  }  // end of i loop

  return g;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation EndEffectorDistanceConstraintCppAd::getLinearApproximation(scalar_t time, const vector_t& state,
                                                                                             const vector_t& input,
                                                                                             const PreComputation& preComp) const {
  const auto numEEs = adKinematicsPtr_->getIds().size();
  const auto eePositions = kinematicsModelPtr_->getFunctionValue(state);
  const auto eeJacobians = kinematicsModelPtr_->getJacobian(state);
  assert(eePositions.size() == 3 * numEEs);
  assert(eeJacobians.rows() == 3 * numEEs);
  assert(eeJacobians.cols() == stateDim_);

  VectorFunctionLinearApproximation approx = VectorFunctionLinearApproximation::Zero(numEEs, stateDim_, inputDim_);
  for (size_t i = 0; i < numEEs; i++) {
    const auto distanceValueGradient = distanceTransformPtr_->getLinearApproximation(eePositions.segment<3>(3 * i));
    approx.f(i) = config_.weight * (distanceValueGradient.first - clearances_[i]);
    approx.dfdx.row(i).noalias() = config_.weight * (distanceValueGradient.second.transpose() * eeJacobians.middleRows<3>(3 * i));
  }  // end of i loop

  return approx;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionQuadraticApproximation EndEffectorDistanceConstraintCppAd::getQuadraticApproximation(scalar_t time, const vector_t& state,
                                                                                                   const vector_t& input,
                                                                                                   const PreComputation& preComp) const {
  const auto numEEs = adKinematicsPtr_->getIds().size();
  auto linearApprox = getLinearApproximation(time, state, input, preComp);

  VectorFunctionQuadraticApproximation quadraticApproximation;
  quadraticApproximation.f = std::move(linearApprox.f);
  quadraticApproximation.dfdx = std::move(linearApprox.dfdx);
  quadraticApproximation.dfdu = std::move(linearApprox.dfdu);
  quadraticApproximation.dfdxx.assign(numEEs, matrix_t::Zero(stateDim_, stateDim_));
  quadraticApproximation.dfdux.assign(numEEs, matrix_t::Zero(inputDim_, stateDim_));
  quadraticApproximation.dfduu.assign(numEEs, matrix_t::Zero(inputDim_, inputDim_));

  return quadraticApproximation;
}

}  // namespace ocs2
