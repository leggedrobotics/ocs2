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

#include <ocs2_centroidal_model/PinocchioCentroidalDynamicsAD.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PinocchioCentroidalDynamicsAD::PinocchioCentroidalDynamicsAD(const PinocchioInterface& pinocchioInterface,
                                                             const CentroidalModelPinocchioMapping<ad_scalar_t>& mapping,
                                                             const std::string& modelName, const std::string& modelFolder,
                                                             bool recompileLibraries, bool verbose) {
  // initialize CppAD interface
  auto pinocchioInterfaceCppAd = pinocchioInterface.toCppAd();

  // set pinocchioInterface to mapping
  std::unique_ptr<CentroidalModelPinocchioMapping<ad_scalar_t>> mappingPtr(mapping.clone());
  mappingPtr->setPinocchioInterface(pinocchioInterfaceCppAd);
  const auto& info = mappingPtr->getCentroidalModelInfo();

  auto systemFlowMapFunc = [&, this](const ad_vector_t& x, ad_vector_t& y) {
    ad_vector_t state = x.head(info.stateDim);
    ad_vector_t input = x.tail(info.inputDim);
    y = getValueCppAd(pinocchioInterfaceCppAd, *mappingPtr, state, input);
  };
  systemFlowMapCppAdInterfacePtr_.reset(
      new CppAdInterface(systemFlowMapFunc, info.stateDim + info.inputDim, modelName + "_systemFlowMap", modelFolder));

  if (recompileLibraries) {
    systemFlowMapCppAdInterfacePtr_->createModels(CppAdInterface::ApproximationOrder::First, verbose);
  } else {
    systemFlowMapCppAdInterfacePtr_->loadModelsIfAvailable(CppAdInterface::ApproximationOrder::First, verbose);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ad_vector_t PinocchioCentroidalDynamicsAD::getValueCppAd(PinocchioInterfaceCppAd& pinocchioInterfaceCppAd,
                                                         const CentroidalModelPinocchioMapping<ad_scalar_t>& mapping,
                                                         const ad_vector_t& state, const ad_vector_t& input) {
  const auto& info = mapping.getCentroidalModelInfo();
  assert(info.stateDim == state.rows());

  const ad_vector_t qPinocchio = mapping.getPinocchioJointPosition(state);
  updateCentroidalDynamics(pinocchioInterfaceCppAd, info, qPinocchio);

  ad_vector_t stateDerivative(info.stateDim);

  // compute center of mass acceleration and derivative of the normalized angular momentum
  stateDerivative.head(6) = mapping.getNormalizedCentroidalMomentumRate(input);

  // derivatives of the floating base variables + joint velocities
  stateDerivative.tail(info.generalizedCoordinatesNum) = mapping.getPinocchioJointVelocity(state, input);

  return stateDerivative;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t PinocchioCentroidalDynamicsAD::getValue(scalar_t time, const vector_t& state, const vector_t& input) const {
  vector_t stateInput(state.rows() + input.rows());
  stateInput << state, input;
  return systemFlowMapCppAdInterfacePtr_->getFunctionValue(stateInput);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation PinocchioCentroidalDynamicsAD::getLinearApproximation(scalar_t time, const vector_t& state,
                                                                                        const vector_t& input) const {
  vector_t stateInput(state.rows() + input.rows());
  stateInput << state, input;
  const vector_t dynamicsValues = systemFlowMapCppAdInterfacePtr_->getFunctionValue(stateInput);
  const matrix_t dynamicsJacobian = systemFlowMapCppAdInterfacePtr_->getJacobian(stateInput);
  VectorFunctionLinearApproximation dynamics;
  dynamics.f = dynamicsValues;
  dynamics.dfdx = dynamicsJacobian.leftCols(state.rows());
  dynamics.dfdu = dynamicsJacobian.rightCols(input.rows());
  return dynamics;
}

}  // namespace ocs2