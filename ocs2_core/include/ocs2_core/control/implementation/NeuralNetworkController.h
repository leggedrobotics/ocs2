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

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
NeuralNetworkController::NeuralNetworkController(const std::string& networkFilePath, state_in_transform_fct_t state_in_transform_fct,
                                                 control_out_transform_fct_t control_out_transform_fct)
    : state_in_transform_fct_(std::move(state_in_transform_fct)), control_out_transform_fct_(std::move(control_out_transform_fct)) {
  loadNetwork(networkFilePath);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void NeuralNetworkController::loadNetwork(const std::string& filePath) {
  policyNet_ = torch::jit::load(filePath);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t NeuralNetworkController::computeInput(scalar_t t, const vector_t& x) {
  Eigen::VectorXf net_input_float = state_in_transform_fct_(t, x);
  auto input_torch = torch::from_blob(net_input_float.data(), net_input_float.size());
  std::vector<torch::jit::IValue> inputs;
  inputs.push_back(input_torch);

  auto output = policyNet_.forward(inputs);
  return control_out_transform_fct_(output);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void NeuralNetworkController::concatenate(const ControllerBase* nextController, int index, int length) {
  throw std::runtime_error("not implemented.");
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ControllerType NeuralNetworkController::getType() const {
  return ControllerType::NEURAL_NETWORK;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void NeuralNetworkController::clear() {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void NeuralNetworkController::setZero() {
  throw std::runtime_error("not implemented.");
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool NeuralNetworkController::empty() const {
  return false;
}

}  // namespace ocs2
